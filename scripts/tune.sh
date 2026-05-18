#!/usr/bin/env bash
# chmod +x this file
# Usage: ./tune.sh <node> <param> <value>
# Example: ./tune.sh servo kp_bearing 0.04
# Example: ./tune.sh aruco confirm_frames 2

set -e
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../ros2_ws/install/setup.bash" 2>/dev/null || true

NODE_ALIAS="$1"
PARAM="$2"
VALUE="$3"

[ -z "$NODE_ALIAS" ] || [ -z "$PARAM" ] || [ -z "$VALUE" ] && {
  echo "Usage: $0 <node_alias> <param> <value>"
  echo "  node aliases: servo, aruco, search, mission, detector, estop"
  exit 1
}

case "$NODE_ALIAS" in
  servo)    ROS_NODE="/visual_servo" ;;
  aruco)    ROS_NODE="/aruco_detector" ;;
  search)   ROS_NODE="/search_behavior" ;;
  mission)  ROS_NODE="/mission_node" ;;
  detector) ROS_NODE="/object_detector" ;;
  estop)    ROS_NODE="/estop_node" ;;
  *)
    echo "Unknown node alias: $NODE_ALIAS"
    echo "Valid aliases: servo, aruco, search, mission, detector, estop"
    exit 1
    ;;
esac

echo "Setting $ROS_NODE $PARAM = $VALUE"
ros2 param set "$ROS_NODE" "$PARAM" "$VALUE"
