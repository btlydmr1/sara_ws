import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
import math

class FinController(Node):
    def __init__(self):
        super().__init__('fin_controller')

        self.sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile_sensor_data)

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/rocket/fins',
            10)

        self.target_pitch = 0.0

    def imu_callback(self, msg):
        q = msg.orientation

        # quaternion -> pitch (asin için clamp şart, yoksa kart eğilince çöker)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)

        error = self.target_pitch - pitch

        # PID yerine basit P kontrol
        kp = 5.0
        control = kp * error

        fin_msg = Float32MultiArray()

        # 4 kanatçık simülasyonu
        fin_msg.data = [
            control,
            -control,
            control,
            -control
        ]

        self.pub.publish(fin_msg)

        self.get_logger().info(f"Pitch:{pitch:.2f} Control:{control:.2f}")

def main():
    rclpy.init()
    node = FinController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()