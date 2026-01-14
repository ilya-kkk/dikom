cmake_minimum_required(VERSION 3.5)

project(amr)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindRack.srv"
  DEPENDENCIES geometry_msgs
)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
