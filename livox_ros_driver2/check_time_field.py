#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class TimeFieldChecker(Node):
    def __init__(self):
        super().__init__('time_field_checker')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',  # LiDAR 토픽
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Listening to /rslidar_points and printing 'time' field...")

    def listener_callback(self, msg):
        try:
            # PointCloud2 메시지를 읽어서 time 필드만 출력
            for point in pc2.read_points(msg, field_names=("time",), skip_nans=True):
                print(f"time: {point[0]:.6f}")
                break  # 첫 포인트만 출력 (속도 위해)
        except Exception as e:
            self.get_logger().error(f"Error reading point cloud: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TimeFieldChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
