import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS_M = 6_371_000


class GpsOdomNode(Node):
    """Convert NavSatFix messages to map-frame Odometry for robot_localization."""

    def __init__(self) -> None:
        super().__init__('gps_odom_node')

        self.declare_parameter('gps_covariance_m2', 5.0)

        self._anchor_lat: float | None = None
        self._anchor_lon: float | None = None

        self._pub = self.create_publisher(Odometry, '/gps/odom', 10)
        self.create_subscription(NavSatFix, '/fix', self._fix_cb, 10)

        self.get_logger().info('GPS odometry node started; waiting for first valid fix.')

    def _position_variance(self) -> float:
        return float(self.get_parameter('gps_covariance_m2').value)

    @staticmethod
    def _is_valid_fix(msg: NavSatFix) -> bool:
        if msg.status.status < 0:
            return False
        return math.isfinite(msg.latitude) and math.isfinite(msg.longitude)

    def _make_odometry(self, msg: NavSatFix, x: float, y: float) -> Odometry:
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        variance = self._position_variance()
        pose_cov = [0.0] * 36
        pose_cov[0] = variance
        pose_cov[7] = variance
        pose_cov[14] = variance
        odom.pose.covariance = pose_cov

        odom.twist.covariance = [0.0] * 36
        return odom

    def _fix_cb(self, msg: NavSatFix) -> None:
        if not self._is_valid_fix(msg):
            return

        if self._anchor_lat is None:
            self._anchor_lat = msg.latitude
            self._anchor_lon = msg.longitude
            self.get_logger().info(
                f'GPS anchor: lat={self._anchor_lat:.8f}, lon={self._anchor_lon:.8f}'
            )
            return

        lat0 = self._anchor_lat
        lon0 = self._anchor_lon
        lat_rad = math.radians(lat0)

        x = math.radians(msg.longitude - lon0) * math.cos(lat_rad) * EARTH_RADIUS_M
        y = math.radians(msg.latitude - lat0) * EARTH_RADIUS_M

        self._pub.publish(self._make_odometry(msg, x, y))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
