#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class FakeSensors(Node):

    def __init__(self):
        super().__init__('fake_sensors')

        self.heading_pub = self.create_publisher(
            Float32,
            '/sara/heading',
            10
        )

        self.depth_pub = self.create_publisher(
            Float32,
            '/sara/depth',
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            '/sara/distance',
            10
        )

        self.heading = 0.0
        self.depth = 2.0
        self.distance = 0.0

        self.speed = 2.0

        self.timer = self.create_timer(
            0.1,
            self.publish_fake_data
        )

        self.get_logger().info('Sahte sensör node başlatıldı.')

    def publish_fake_data(self):

        # Sahte mesafe artışı
        self.distance += self.speed * 0.1

        heading_msg = Float32()
        heading_msg.data = self.heading

        depth_msg = Float32()
        depth_msg.data = self.depth

        distance_msg = Float32()
        distance_msg.data = self.distance

        self.heading_pub.publish(heading_msg)
        self.depth_pub.publish(depth_msg)
        self.distance_pub.publish(distance_msg)

        self.get_logger().info(
            f'Heading: {self.heading:.1f}° | '
            f'Derinlik: {self.depth:.1f} m | '
            f'Mesafe: {self.distance:.1f} m',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)

    node = FakeSensors()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()