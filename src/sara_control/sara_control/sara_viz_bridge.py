#!/usr/bin/env python3
"""
SARA görselleştirme köprüsü.

İki işi yapar:
  1) /rocket/fins (Float32MultiArray, 4 eleman) -> /joint_states
     Böylece robot_state_publisher kanatçıkları döndürür (kanatçıklar AYRI oynar).
  2) /mavros/imu/data (Imu) -> TF: world -> base_link
     Böylece roket GÖVDESİ gerçek yönelime göre döner.

RViz'de Fixed Frame = world seçildiğinde hem gövde dönüşü hem kanatçık hareketi
aynı anda görünür.

Kanatçık index eşlemesi (/rocket/fins.data sırası):
  data[0] -> fin_px_joint
  data[1] -> fin_nx_joint
  data[2] -> fin_py_joint
  data[3] -> fin_ny_joint
fin_controller'daki yayın sırasıyla bu eşleme tutarlı olmalı.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# /rocket/fins dizisindeki index -> URDF joint adı
FIN_JOINTS = ["fin_px_joint", "fin_nx_joint", "fin_py_joint", "fin_ny_joint"]

# Kontrol değerini kanatçık açısına ölçekleme.
# fin_controller şu an kp*error (radyan ölçeğinde) yayınlıyor; gerekirse
# burayı değiştirerek görsel açı genliğini ayarlayabilirsin.
FIN_SCALE = 1.0
# URDF limitleriyle uyumlu güvenli sınır (rad)
FIN_LIMIT = 0.6


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class VizBridge(Node):
    def __init__(self):
        super().__init__('sara_viz_bridge')

        # --- Kanatçık komutları -> joint_states ---
        self.last_fins = [0.0, 0.0, 0.0, 0.0]
        self.create_subscription(
            Float32MultiArray, '/rocket/fins', self.fins_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # --- IMU -> base_link TF ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_orientation = None
        self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_cb, qos_profile_sensor_data)

        # joint_states ve TF'yi düzenli (50 Hz) yayınla.
        # Komut/IMU gelmese bile son değeri tekrar yayınlamak RViz'i stabil tutar.
        self.timer = self.create_timer(0.02, self.publish_all)

        self.get_logger().info(
            "sara_viz_bridge başladı: /rocket/fins -> /joint_states, "
            "/mavros/imu/data -> TF(world->base_link)")

    def fins_cb(self, msg):
        d = list(msg.data)
        for i in range(min(4, len(d))):
            self.last_fins[i] = clamp(d[i] * FIN_SCALE, -FIN_LIMIT, FIN_LIMIT)

    def imu_cb(self, msg):
        self.last_orientation = msg.orientation

    def publish_all(self):
        now = self.get_clock().now().to_msg()

        # 1) Kanatçık eklemleri
        js = JointState()
        js.header.stamp = now
        js.name = FIN_JOINTS
        js.position = [float(x) for x in self.last_fins]
        self.joint_pub.publish(js)

        # 2) Gövde yönelimi (world -> base_link)
        if self.last_orientation is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'
            # Roket yerinde döner; konum sabit
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation = self.last_orientation
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = VizBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
