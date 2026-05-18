#!/usr/bin/env bash
set -e
echo "Building ROS2 workspace inside Docker..."
docker compose run --rm dev bash -c \
  "source /opt/ros/humble/setup.bash && \
   colcon build --symlink-install \
   --packages-select aruco_detector urc_autonomy visual_servo \
   search_behavior urc_localization && \
   echo 'Build complete.'"
