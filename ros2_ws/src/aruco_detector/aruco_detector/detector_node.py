import json
import math
from dataclasses import dataclass, field

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

MARKER_SIZE_M = 0.20        # 20 cm face, per URC 2026 rulebook
ARUCO_DICT = aruco.DICT_4X4_50
BEARING_CONSISTENCY_DEG = 5.0
DEPTH_MISMATCH_RATIO = 0.15


@dataclass
class MarkerGateState:
    streak: int = 0
    confirmed: bool = False
    lost_streak: int = 0
    recent_bearings: list[float] = field(default_factory=list)


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('use_clahe', True)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('loss_frames', 5)

        self._use_clahe = self.get_parameter('use_clahe').value
        self._confirm_frames = int(self.get_parameter('confirm_frames').value)
        self._loss_frames = int(self.get_parameter('loss_frames').value)

        self._bridge = CvBridge()
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._depth_image: np.ndarray | None = None

        self._aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        self._aruco_params = aruco.DetectorParameters()
        # Tighten for the 5–20 m operating range: larger min perimeter filters
        # noise while still catching the marker at distance.
        self._aruco_params.minMarkerPerimeterRate = 0.02
        self._aruco_params.maxMarkerPerimeterRate = 4.0
        self._detector = aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        # Half the face length along each axis for solvePnP object points.
        h = MARKER_SIZE_M / 2.0
        self._obj_points = np.array([
            [-h,  h, 0],
            [ h,  h, 0],
            [ h, -h, 0],
            [-h, -h, 0],
        ], dtype=np.float32)

        self._tracked_ids: set[int] = set()
        self._gate_state: dict[int, MarkerGateState] = {}

        self._sub_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self._camera_info_cb,
            10,
        )
        self._sub_image = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._image_cb,
            10,
        )
        self._sub_depth = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self._depth_cb,
            10,
        )
        self._pub_detection = self.create_publisher(String, '/aruco/detection', 10)
        self._pub_debug = self.create_publisher(Image, '/aruco/debug_image', 10)

        self.get_logger().info('ArUco detector node started — waiting for camera info.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._camera_matrix is not None:
            return  # only need it once

        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.get_logger().info('Camera intrinsics received — detector active.')

    def _depth_cb(self, msg: Image) -> None:
        self._depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def _image_cb(self, msg: Image) -> None:
        if self._camera_matrix is None:
            return  # nothing useful without intrinsics

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self._detect_markers(gray)
        detected_ids: set[int] = set()
        confirmed_ids: set[int] = set()

        for marker_id, corners in detections.items():
            detected_ids.add(marker_id)
            distance_m, bearing_deg = self._estimate_pose(corners)
            color_shape = (frame.shape[0], frame.shape[1])
            depth_m, depth_validated = self._validate_depth(
                corners, distance_m, color_shape,
            )
            confidence = self._update_gate(marker_id, bearing_deg, seen=True)
            if confidence == 'confirmed':
                confirmed_ids.add(marker_id)

            self._draw_overlay(
                frame,
                corners,
                marker_id,
                distance_m,
                bearing_deg,
                confidence,
                depth_m,
                depth_validated,
            )

            if confidence == 'confirmed':
                payload = json.dumps({
                    'id': marker_id,
                    'distance_m': round(float(distance_m), 3),
                    'bearing_deg': round(float(bearing_deg), 2),
                    'depth_m': round(depth_m, 3) if depth_m is not None else None,
                    'depth_validated': depth_validated,
                    'confidence': confidence,
                })
                self._pub_detection.publish(String(data=payload))

        self._update_gate_for_lost(detected_ids)
        self._update_tracking(confirmed_ids)
        self._pub_debug.publish(self._bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

    # ------------------------------------------------------------------
    # Detection (raw + optional CLAHE)
    # ------------------------------------------------------------------

    def _detect_markers(self, gray: np.ndarray) -> dict[int, np.ndarray]:
        """Return marker_id -> corners, preferring raw over CLAHE when both hit."""
        corners_raw, ids_raw, _ = self._detector.detectMarkers(gray)
        merged: dict[int, np.ndarray] = {}

        if ids_raw is not None:
            for i, marker_id in enumerate(ids_raw.flatten()):
                merged[int(marker_id)] = corners_raw[i]

        if not self._use_clahe:
            return merged

        gray_clahe = self._clahe.apply(gray)
        corners_clahe, ids_clahe, _ = self._detector.detectMarkers(gray_clahe)

        if ids_clahe is None:
            return merged

        for i, marker_id in enumerate(ids_clahe.flatten()):
            mid = int(marker_id)
            if mid not in merged:
                merged[mid] = corners_clahe[i]

        return merged

    # ------------------------------------------------------------------
    # Pose estimation
    # ------------------------------------------------------------------

    def _estimate_pose(self, corners: np.ndarray) -> tuple[float, float]:
        """Return (distance_m, bearing_deg) for a single detected marker."""
        img_points = corners.reshape(4, 2).astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            self._obj_points,
            img_points,
            self._camera_matrix,
            self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            return 0.0, 0.0

        # tvec is [x, y, z] in camera frame: z = depth, x = lateral offset.
        x, _, z = tvec.flatten()
        distance_m = float(np.linalg.norm(tvec))
        bearing_deg = float(math.degrees(math.atan2(x, z)))

        return distance_m, bearing_deg

    # ------------------------------------------------------------------
    # Depth fusion
    # ------------------------------------------------------------------

    def _depth_at_pixel(self, cx: int, cy: int, color_shape: tuple[int, int] | None = None) -> float | None:
        depth = self._depth_image
        if depth is None:
            return None

        if color_shape is not None:
            color_h, color_w = color_shape
            depth_h, depth_w = depth.shape[:2]
            if (depth_h, depth_w) != (color_h, color_w):
                cx = int(round(cx * depth_w / color_w))
                cy = int(round(cy * depth_h / color_h))

        if cy < 0 or cx < 0 or cy >= depth.shape[0] or cx >= depth.shape[1]:
            return None

        depth_mm = int(depth[cy, cx])
        if depth_mm == 0:
            return None

        return depth_mm / 1000.0

    def _validate_depth(
        self,
        corners: np.ndarray,
        solvepnp_distance_m: float,
        color_shape: tuple[int, int],
    ) -> tuple[float | None, bool]:
        center = corners.reshape(4, 2).mean(axis=0)
        cx = int(round(center[0]))
        cy = int(round(center[1]))
        depth_m = self._depth_at_pixel(cx, cy, color_shape)

        if depth_m is None:
            return None, True

        if solvepnp_distance_m <= 0.0:
            return depth_m, True

        mismatch = abs(depth_m - solvepnp_distance_m)
        if mismatch > DEPTH_MISMATCH_RATIO * solvepnp_distance_m:
            return depth_m, False

        return depth_m, True

    # ------------------------------------------------------------------
    # Multi-frame confidence gating
    # ------------------------------------------------------------------

    @staticmethod
    def _angle_diff_deg(a: float, b: float) -> float:
        return abs((a - b + 180.0) % 360.0 - 180.0)

    def _bearings_consistent(self, bearings: list[float]) -> bool:
        if len(bearings) < 2:
            return True
        ref = bearings[0]
        return all(
            self._angle_diff_deg(b, ref) < BEARING_CONSISTENCY_DEG
            for b in bearings[1:]
        )

    def _update_gate(self, marker_id: int, bearing_deg: float, seen: bool) -> str:
        if not seen:
            return 'pending'

        state = self._gate_state.setdefault(marker_id, MarkerGateState())

        if state.confirmed:
            state.lost_streak = 0
            return 'confirmed'

        state.streak += 1
        state.recent_bearings.append(bearing_deg)
        if len(state.recent_bearings) > self._confirm_frames:
            state.recent_bearings = state.recent_bearings[-self._confirm_frames:]

        window = state.recent_bearings[-self._confirm_frames:]
        if state.streak >= self._confirm_frames and len(window) >= self._confirm_frames:
            if self._bearings_consistent(window):
                state.confirmed = True
                return 'confirmed'
            state.streak = 1
            state.recent_bearings = [bearing_deg]

        return 'pending'

    def _update_gate_for_lost(self, detected_ids: set[int]) -> None:
        for marker_id in list(self._gate_state):
            if marker_id in detected_ids:
                continue

            state = self._gate_state[marker_id]
            if state.confirmed:
                state.lost_streak += 1
                if state.lost_streak >= self._loss_frames:
                    del self._gate_state[marker_id]
            else:
                del self._gate_state[marker_id]

    # ------------------------------------------------------------------
    # Debug overlay
    # ------------------------------------------------------------------

    def _draw_overlay(
        self,
        frame: np.ndarray,
        corners: np.ndarray,
        marker_id: int,
        distance_m: float,
        bearing_deg: float,
        confidence: str,
        depth_m: float | None,
        depth_validated: bool,
    ) -> None:
        aruco.drawDetectedMarkers(frame, [corners])

        color = (0, 255, 0) if confidence == 'confirmed' else (0, 200, 255)
        if confidence == 'confirmed' and not depth_validated:
            color = (0, 165, 255)

        center = corners.reshape(4, 2).mean(axis=0).astype(int)
        depth_str = f'{depth_m:.2f}m' if depth_m is not None else 'n/a'
        valid_str = '' if depth_validated else ' !depth'
        label = (
            f'ID {marker_id} [{confidence}]  {distance_m:.2f}m  '
            f'{bearing_deg:+.1f}deg  d={depth_str}{valid_str}'
        )
        cv2.putText(
            frame,
            label,
            (center[0] - 80, center[1] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )

    # ------------------------------------------------------------------
    # Detection state tracking
    # ------------------------------------------------------------------

    def _update_tracking(self, detected_ids: set[int]) -> None:
        newly_detected = detected_ids - self._tracked_ids
        newly_lost = self._tracked_ids - detected_ids

        for mid in newly_detected:
            self.get_logger().info(f'Marker detected: ID {mid}')
        for mid in newly_lost:
            self.get_logger().info(f'Marker lost: ID {mid}')

        self._tracked_ids = detected_ids


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
