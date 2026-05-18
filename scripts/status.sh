#!/usr/bin/env bash
# chmod +x this file
set -e
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../ros2_ws/install/setup.bash" 2>/dev/null || true

echo "=== URC 2026 ROVER STATUS ==="
echo ""

echo "--- Active Nodes ---"
ros2 node list 2>/dev/null || echo "(none)"
echo ""

echo "--- Mission State ---"
timeout 2 ros2 topic echo /mission/state std_msgs/msg/String \
  --once 2>/dev/null || echo "(no data)"
echo ""

echo "--- Last ArUco Detection ---"
timeout 2 ros2 topic echo /aruco/detection std_msgs/msg/String \
  --once 2>/dev/null || echo "(no detection)"
echo ""

echo "--- E-Stop Status ---"
timeout 2 ros2 topic echo /estop/status std_msgs/msg/String \
  --once 2>/dev/null || echo "(no data)"
echo ""

echo "--- Topic Hz ---"
echo "ArUco detections:"
timeout 3 ros2 topic hz /aruco/detection 2>/dev/null || echo "  (no data)"
echo "Object detections:"
timeout 3 ros2 topic hz /objects/detections 2>/dev/null || echo "  (no data)"
