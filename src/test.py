#!/usr/bin/env python3

import math
from typing import List, Tuple, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

# Импорт сервиса
from amr.srv import FindRack


class LegDetector:
    # Заменяет некорректные/бесконечные значения дальности на range_max.
    def normalize_ranges(self, ranges: List[float], range_max: float) -> List[float]:
        data = []
        for r in ranges:
            if r is None or not math.isfinite(r) or r <= 0.0:
                data.append(range_max)
            else:
                data.append(r)
        return data

    # Считает разность между соседними лучами лидара.
    def compute_diff(self, ranges: List[float]) -> List[float]:
        if not ranges:
            return []
        diff = [ranges[i + 1] - ranges[i] for i in range(len(ranges) - 1)]
        diff.append(0.0)
        return diff

    def find_leg_edges_from_diff(
        self, diff: List[float], threshold: float, max_width_rays: int = 30
    ) -> List[Tuple[int, int]]:
        # Находит пары индексов (start, end), где по скачкам дальности видны ножки.
        legs = []
        n = len(diff)

        i = 0
        while i < n:
            # ВХОД на ножку
            if diff[i] < -threshold:
                start = i + 1

                end = None
                for j in range(start, min(start + max_width_rays, n)):
                    if diff[j] > threshold:
                        end = j
                        break

                if end is not None:
                    legs.append((start, end))
                    i = end  # прыгаем дальше, чтобы не ловить то же самое
                else:
                    i += 1
            else:
                i += 1

        return legs

    def leg_center_xy_simple(
        self,
        ranges: List[float],
        edges: List[Tuple[int, int]],
        angle_min: float,
        angle_increment: float,
    ) -> List[Tuple[float, float]]:
        # Переводит края ножек в их центры в плоскости (x, y) лидара.
        centers = []

        for start, end in edges:
            r1 = ranges[start]
            r2 = ranges[end]

            theta1 = angle_min + start * angle_increment
            theta2 = angle_min + end * angle_increment

            x1 = r1 * math.cos(theta1)
            y1 = r1 * math.sin(theta1)

            x2 = r2 * math.cos(theta2)
            y2 = r2 * math.sin(theta2)

            x = 0.5 * (x1 + x2)
            y = 0.5 * (y1 + y2)

            centers.append((x, y))

        return centers

    def detect_centers(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_max: float,
        diff_threshold: float,
        max_width_rays: int,
        max_legs: int,
    ) -> List[Tuple[float, float]]:
        # Полный пайплайн: нормализация, diff, поиск ножек и их центров.
        normalized = self.normalize_ranges(ranges, range_max)
        diff = self.compute_diff(normalized)
        edges = self.find_leg_edges_from_diff(
            diff, diff_threshold, max_width_rays=max_width_rays
        )
        centers = self.leg_center_xy_simple(
            normalized, edges, angle_min, angle_increment
        )
        if max_legs > 0:
            centers = centers[:max_legs]
        return centers

    def compute_leg_width(
        self,
        start_idx: int,
        end_idx: int,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
    ) -> float:
        # Вычисляет ширину ножки в метрах.
        r1 = ranges[start_idx]
        r2 = ranges[end_idx]
        theta1 = angle_min + start_idx * angle_increment
        theta2 = angle_min + end_idx * angle_increment

        x1 = r1 * math.cos(theta1)
        y1 = r1 * math.sin(theta1)
        x2 = r2 * math.cos(theta2)
        y2 = r2 * math.sin(theta2)

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_leg_edges(
        self,
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        range_max: float,
        diff_threshold: float,
        max_width_rays: int,
    ) -> List[Tuple[int, int]]:
        # Возвращает список (start_idx, end_idx) для каждой найденной ножки.
        normalized = self.normalize_ranges(ranges, range_max)
        diff = self.compute_diff(normalized)
        return self.find_leg_edges_from_diff(diff, diff_threshold, max_width_rays)


class RackFinderNode(Node):
    # ROS2-нода, которая по данным лидара ищет стеллаж и отправляет Nav2 goal.

    # Инициализирует ноду, параметры, подписки, TF и сервис.
    def __init__(self) -> None:
        super().__init__("rack_finder_service")

        self._detector = LegDetector()
        self._last_scan: Optional[LaserScan] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Action-клиент Nav2 для отправки цели навигации
        self._nav_to_pose_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        # Параметры детекции
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("diff_threshold", 0.15)
        self.declare_parameter("max_width_rays", 30)
        self.declare_parameter("max_legs", 6)
        self.declare_parameter("range_max", 0.0)

        # Подписка на LaserScan для сохранения последнего сообщения
        scan_topic = (
            self.get_parameter("scan_topic")
            .get_parameter_value()
            .string_value
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        # Сервис поиска стеллажа
        self.create_service(
            FindRack,
            "find_rack",
            self._handle_find_rack,
        )

        self.get_logger().info("Rack finder node is ready")

    # Callback лидара: просто сохраняет последнее сообщение LaserScan.
    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _find_leg_pair(
        self,
        leg_centers: List[Tuple[float, float]],
        leg_edges: List[Tuple[int, int]],
        ranges: List[float],
        angle_min: float,
        angle_increment: float,
        leg_width: float,
        leg_spacing: float,
    ) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        # По наборам ножек ищет пару, подходящую под ширину и шаг стеллажа.
        num_legs = len(leg_centers)
        if num_legs < 2:
            return None

        # Допуски
        width_tolerance = leg_width * 0.2  # ±20%
        spacing_tolerance = leg_spacing * 0.1  # ±10%

        best_pair = None
        best_score = float("inf")

        # Логика для разных количеств ножек
        if num_legs == 2:
            # Просто проверяем соответствие параметрам
            leg1, leg2 = leg_centers[0], leg_centers[1]
            spacing = math.sqrt(
                (leg2[0] - leg1[0]) ** 2 + (leg2[1] - leg1[1]) ** 2
            )

            # Проверяем ширину ножек
            width1 = self._detector.compute_leg_width(
                leg_edges[0][0],
                leg_edges[0][1],
                ranges,
                angle_min,
                angle_increment,
            )
            width2 = self._detector.compute_leg_width(
                leg_edges[1][0],
                leg_edges[1][1],
                ranges,
                angle_min,
                angle_increment,
            )

            if (
                abs(spacing - leg_spacing) <= spacing_tolerance
                and abs(width1 - leg_width) <= width_tolerance
                and abs(width2 - leg_width) <= width_tolerance
            ):
                return (leg1, leg2)

        elif num_legs == 3:
            # Ищем пару с наилучшим соответствием
            for i in range(num_legs):
                for j in range(i + 1, num_legs):
                    leg1, leg2 = leg_centers[i], leg_centers[j]
                    spacing = math.sqrt(
                        (leg2[0] - leg1[0]) ** 2 + (leg2[1] - leg1[1]) ** 2
                    )

                    width1 = self._detector.compute_leg_width(
                        leg_edges[i][0],
                        leg_edges[i][1],
                        ranges,
                        angle_min,
                        angle_increment,
                    )
                    width2 = self._detector.compute_leg_width(
                        leg_edges[j][0],
                        leg_edges[j][1],
                        ranges,
                        angle_min,
                        angle_increment,
                    )

                    spacing_error = abs(spacing - leg_spacing)
                    width_error1 = abs(width1 - leg_width)
                    width_error2 = abs(width2 - leg_width)

                    if (
                        spacing_error <= spacing_tolerance
                        and width_error1 <= width_tolerance
                        and width_error2 <= width_tolerance
                    ):
                        score = spacing_error + width_error1 + width_error2
                        if score < best_score:
                            best_score = score
                            best_pair = (leg1, leg2)

        elif num_legs >= 4:
            # Выбираем ближайшие две ножки к роботу
            # Среди всех подходящих пар выбираем ту, где среднее расстояние до робота минимально
            for i in range(num_legs):
                for j in range(i + 1, num_legs):
                    leg1 = leg_centers[i]
                    leg2 = leg_centers[j]
                    spacing = math.sqrt(
                        (leg2[0] - leg1[0]) ** 2
                        + (leg2[1] - leg1[1]) ** 2
                    )

                    width1 = self._detector.compute_leg_width(
                        leg_edges[i][0],
                        leg_edges[i][1],
                        ranges,
                        angle_min,
                        angle_increment,
                    )
                    width2 = self._detector.compute_leg_width(
                        leg_edges[j][0],
                        leg_edges[j][1],
                        ranges,
                        angle_min,
                        angle_increment,
                    )

                    spacing_error = abs(spacing - leg_spacing)
                    width_error1 = abs(width1 - leg_width)
                    width_error2 = abs(width2 - leg_width)

                    if (
                        spacing_error <= spacing_tolerance
                        and width_error1 <= width_tolerance
                        and width_error2 <= width_tolerance
                    ):
                        # Вычисляем среднее расстояние до робота для этой пары
                        distance1 = math.sqrt(leg1[0] ** 2 + leg1[1] ** 2)
                        distance2 = math.sqrt(leg2[0] ** 2 + leg2[1] ** 2)
                        avg_distance = (distance1 + distance2) / 2.0

                        # Используем среднее расстояние как часть score (меньше = лучше)
                        # Нормализуем расстояние (примерно 0.01 на метр)
                        distance_score = avg_distance * 0.01
                        score = spacing_error + width_error1 + width_error2 + distance_score

                        if score < best_score:
                            best_score = score
                            best_pair = (leg1, leg2)

        return best_pair

    def _calculate_target_pose(
        self,
        leg1: Tuple[float, float],
        leg2: Tuple[float, float],
        entry_depth: float,
    ) -> Pose:
        """
        Вычисляет целевую позу между двумя ножками на глубине entry_depth.
        Ориентация перпендикулярна линии между ножками.
        """
        # Центр между ножками
        center_x = 0.5 * (leg1[0] + leg2[0])
        center_y = 0.5 * (leg1[1] + leg2[1])

        # Направление от центра к стеллажу (перпендикулярно линии между ножками)
        dx = leg2[0] - leg1[0]
        dy = leg2[1] - leg1[1]
        leg_distance = math.sqrt(dx ** 2 + dy ** 2)

        if leg_distance < 1e-6:
            # Ножки слишком близко, используем направление по умолчанию
            direction_x = 1.0
            direction_y = 0.0
        else:
            # Перпендикулярное направление (поворот на 90 градусов)
            direction_x = -dy / leg_distance
            direction_y = dx / leg_distance

        # Целевая позиция: центр + entry_depth в направлении к стеллажу
        target_x = center_x + entry_depth * direction_x
        target_y = center_y + entry_depth * direction_y

        # Ориентация: перпендикулярно линии между ножками
        # yaw = atan2(dy, dx) + π/2
        yaw = math.atan2(dy, dx) + math.pi / 2.0

        # Создаем Pose
        pose = Pose()
        pose.position.x = target_x
        pose.position.y = target_y
        pose.position.z = 0.0

        # Кватернион из yaw
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def _handle_find_rack(
        self, request: FindRack.Request, response: FindRack.Response
    ) -> FindRack.Response:
        # Сервис: найти стеллаж, вычислить целевую позу и отправить Nav2 goal.
        if self._last_scan is None:
            response.success = False
            response.message = "No LaserScan data available"
            return response

        scan = self._last_scan

        diff_threshold, max_width_rays, max_legs, range_max = (
            self._get_detection_params(scan)
        )

        leg_centers, leg_edges = self._detect_legs(
            scan, diff_threshold, max_width_rays, max_legs
        )

        if len(leg_centers) < 2:
            response.success = False
            response.message = f"Not enough legs detected: {len(leg_centers)}"
            return response

        leg_pair = self._find_leg_pair(
            leg_centers,
            leg_edges,
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            request.leg_width,
            request.leg_spacing,
        )

        if leg_pair is None:
            response.success = False
            response.message = "No leg pair found matching the specified parameters"
            return response

        target_pose = self._calculate_target_pose(
            leg_pair[0], leg_pair[1], request.entry_depth
        )

        # Формируем цель прямо в фрейме лидара (Nav2 сам использует TF)
        pose_stamped_lidar = PoseStamped()
        pose_stamped_lidar.header.stamp = scan.header.stamp
        pose_stamped_lidar.header.frame_id = scan.header.frame_id
        pose_stamped_lidar.pose = target_pose

        nav_ok, nav_message = self._send_nav2_goal(pose_stamped_lidar)
        if not nav_ok:
            response.success = False
            response.message = nav_message
            return response

        response.target_pose = pose_stamped_lidar
        response.success = True
        response.message = (
            "Rack found, target pose calculated and navigation goal sent"
        )
        return response

    def _get_detection_params(
        self, scan: LaserScan
    ) -> Tuple[float, int, int, float]:
        # Считывает параметры детекции из параметров ноды.
        diff_threshold = (
            self.get_parameter("diff_threshold")
            .get_parameter_value()
            .double_value
        )
        max_width_rays = (
            self.get_parameter("max_width_rays")
            .get_parameter_value()
            .integer_value
        )
        max_legs = (
            self.get_parameter("max_legs")
            .get_parameter_value()
            .integer_value
        )
        range_max_override = (
            self.get_parameter("range_max")
            .get_parameter_value()
            .double_value
        )
        range_max = (
            range_max_override if range_max_override > 0.0 else scan.range_max
        )
        return diff_threshold, max_width_rays, max_legs, range_max

    def _detect_legs(
        self,
        scan: LaserScan,
        diff_threshold: float,
        max_width_rays: int,
        max_legs: int,
    ) -> Tuple[List[Tuple[float, float]], List[Tuple[int, int]]]:
        # Детектирует ножки и возвращает их центры и индексы краёв.
        leg_edges = self._detector.get_leg_edges(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_max,
            diff_threshold,
            max_width_rays,
        )

        leg_centers = self._detector.leg_center_xy_simple(
            scan.ranges, leg_edges, scan.angle_min, scan.angle_increment
        )

        if max_legs > 0:
            leg_centers = leg_centers[:max_legs]
            leg_edges = leg_edges[:max_legs]

        return leg_centers, leg_edges

    def _send_nav2_goal(
        self, pose_stamped_odom: PoseStamped
    ) -> Tuple[bool, str]:
        # Отправляет goal в Nav2 (NavigateToPose).
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "Nav2 NavigateToPose action server not available"
            )
            return False, (
                "Target pose calculated, but NavigateToPose action server "
                "is not available"
            )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped_odom

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected")
            return False, "Target pose calculated, but goal was rejected"

        self.get_logger().info(
            "Rack found, target pose calculated and NavigateToPose goal sent"
        )
        return True, "Navigation goal sent successfully"


def main() -> None:
    rclpy.init()
    node = RackFinderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
