#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

cd /workspace
if [ -f /workspace/install/.colcon_install_layout ] && grep -q "^isolated$" /workspace/install/.colcon_install_layout; then
  rm -rf /workspace/install /workspace/build /workspace/log
fi
colcon build --symlink-install --merge-install
source /workspace/install/setup.bash

if [ "$#" -eq 0 ]; then
  set -- bash
fi

exec "$@"
