#!/usr/bin/env bash
set -e
# chmod +x this file after creation

BAG_NAME="urc_data_$(date +%Y%m%d_%H%M%S)"
echo "Recording to $BAG_NAME"
echo "Point camera at target objects. Press Ctrl+C to stop."

source /opt/ros/humble/setup.bash

ros2 bag record \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw \
  /camera/depth/image_rect_raw \
  /fix \
  /imu/data \
  --output "$BAG_NAME" \
  --max-bag-duration 300

echo "Bag saved: $BAG_NAME"
echo "To extract frames for labeling:"
echo "  ros2 bag play $BAG_NAME"
echo "  ros2 run image_tools showimage --ros-args -r /image:=/camera/color/image_raw"
