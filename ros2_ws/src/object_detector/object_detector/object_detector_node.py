"""YOLOv8 object detection for URC 2026 task objects (mallet, rock pick, water bottle)."""

import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO

CLASS_NAMES: dict[int, str] = {
    0: 'mallet',
    1: 'rock_pick',
    2: 'water_bottle',
}

BOX_COLORS: dict[int, tuple[int, int, int]] = {
    0: (0, 140, 255),   # orange mallet
    1: (180, 105, 255),  # rock pick
    2: (255, 200, 0),    # water bottle
}


class ObjectDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('object_detector')

        self.declare_parameter('model_path', 'models/urc_objects.pt')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('publish_rate_hz', 15.0)

        model_path = str(self.get_parameter('model_path').value)
        self._conf_threshold = float(self.get_parameter('confidence_threshold').value)
        requested_device = str(self.get_parameter('device').value)
        self._device = self._resolve_device(requested_device)
        if requested_device == 'cuda' and self._device == 'cpu':
            self.get_logger().warn('CUDA unavailable — falling back to CPU inference.')
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._bridge = CvBridge()
        self._focal_length_x: float | None = None
        self._principal_point_x: float | None = None
        self._depth_image: np.ndarray | None = None
        self._latest_color: np.ndarray | None = None

        import os
        self._model: 'YOLO | None' = None
        if not os.path.isfile(model_path):
            self.get_logger().warn(
                f'Model not found at "{model_path}" — detector will publish '
                'empty detections. Set model_path param and restart.'
            )
        else:
            self.get_logger().info(
                f'Loading YOLO model from {model_path} on {self._device}...'
            )
            self._model = YOLO(model_path)

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
        self._pub_detections = self.create_publisher(String, '/objects/detections', 10)
        self._pub_debug = self.create_publisher(Image, '/objects/debug_image', 10)

        timer_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 1.0 / 15.0
        self._timer = self.create_timer(timer_period, self._timer_cb)

        self.get_logger().info(
            f'Object detector ready — publishing at {publish_rate_hz:.1f} Hz '
            f'(conf >= {self._conf_threshold:.2f}).',
        )

    # ------------------------------------------------------------------
    # Device / intrinsics
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_device(requested: str) -> str:
        if requested != 'cuda':
            return requested

        try:
            import torch
        except ImportError:
            return 'cpu'

        if torch.cuda.is_available():
            return 'cuda'

        return 'cpu'

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._focal_length_x is not None:
            return

        k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._focal_length_x = float(k[0, 0])
        self._principal_point_x = float(k[0, 2])
        self.get_logger().info('Camera intrinsics received — bearing estimation active.')

    # ------------------------------------------------------------------
    # Frame buffering
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        try:
            self._latest_color = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Color image conversion failed: {exc}')

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as exc:
            self.get_logger().warn(f'Depth image conversion failed: {exc}')

    # ------------------------------------------------------------------
    # Depth + bearing
    # ------------------------------------------------------------------

    def _depth_at_pixel(
        self,
        cx: int,
        cy: int,
        color_shape: tuple[int, int] | None = None,
    ) -> float | None:
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

    def _bearing_deg(self, cx: int) -> float:
        if self._focal_length_x is None or self._principal_point_x is None:
            return 0.0

        x = (cx - self._principal_point_x) / self._focal_length_x
        return float(math.degrees(math.atan2(x, 1.0)))

    # ------------------------------------------------------------------
    # Timer-driven inference
    # ------------------------------------------------------------------

    def _timer_cb(self) -> None:
        if self._latest_color is None or self._focal_length_x is None:
            return
        if self._model is None:
            self._pub_detections.publish(String(data='[]'))
            return

        frame = self._latest_color.copy()
        color_shape = frame.shape[:2]

        results = self._model.predict(
            frame,
            conf=self._conf_threshold,
            verbose=False,
            device=self._device,
        )

        detections: list[dict] = []
        result = results[0]
        boxes = result.boxes

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                class_id = int(box.cls.item())
                confidence = round(float(box.conf.item()), 2)
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                distance_m = self._depth_at_pixel(cx, cy, color_shape)
                bearing_deg = round(self._bearing_deg(cx), 2)
                class_name = CLASS_NAMES.get(class_id, f'class_{class_id}')

                detections.append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'confidence': confidence,
                    'bbox': [x1, y1, x2, y2],
                    'distance_m': round(distance_m, 3) if distance_m is not None else None,
                    'bearing_deg': bearing_deg,
                })

                self._draw_detection(
                    frame,
                    class_id,
                    class_name,
                    confidence,
                    x1,
                    y1,
                    x2,
                    y2,
                    distance_m,
                    bearing_deg,
                )

        self._pub_detections.publish(String(data=json.dumps(detections)))
        self._pub_debug.publish(self._bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

    # ------------------------------------------------------------------
    # Debug overlay
    # ------------------------------------------------------------------

    def _draw_detection(
        self,
        frame: np.ndarray,
        class_id: int,
        class_name: str,
        confidence: float,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        distance_m: float | None,
        bearing_deg: float,
    ) -> None:
        color = BOX_COLORS.get(class_id, (0, 255, 0))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        depth_str = f'{distance_m:.2f}m' if distance_m is not None else 'n/a'
        label = f'{class_name} {confidence:.2f}  {bearing_deg:+.1f}deg  d={depth_str}'
        label_y = max(y1 - 8, 16)
        cv2.putText(
            frame,
            label,
            (x1, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
