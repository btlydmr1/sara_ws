#!/usr/bin/env python3
"""
mock_sensors.py
=================
SARA platformu - Sahte Sensor Yayincisi (SADECE TEST/GELISTIRME icin)

Hicbir donanim (IMU, basinc sensoru, SEN0368) bagli degilken, navigation.py
dugumunu uctan uca test edebilmek icin gercekci bir gorev senaryosu simule
eder ve navigation.py'nin bekledigi TAM AYNI topic/mesaj tiplerine yayin
yapar:

    /mavros/imu/data          (sensor_msgs/Imu)
    /sara/pressure             (sensor_msgs/FluidPressure)
    /sara/water_detect_1       (std_msgs/Bool)
    /sara/water_detect_2       (std_msgs/Bool)

Simule edilen senaryo (basit dalis profili, Tablo 12 fazlarina benzer):
    0   - dive_start_s      : YUZEYDE  (derinlik ~0, su sensorleri "su yok")
    dive_start_s - dive_end_s : DALIS  (derinlik 0 -> target_depth lineer)
    dive_end_s - cruise_end_s : DUZ SEYIR (derinlik sabit, heading donuyor)
    cruise_end_s - ascend_end_s : YUZEYE CIKIS (derinlik target_depth -> 0)
    ascend_end_s sonrasi     : YUZEYDE, senaryo tekrar baslar (loop)

Gercek donanim baglandiginda bu node KAPATILIR, navigation.py hicbir
degisiklik gerektirmeden gercek sensor topic'lerini dinlemeye devam eder.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, FluidPressure


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class MockSensorPublisher(Node):

    def __init__(self):
        super().__init__('mock_sensors')

        # ---------------- Senaryo parametreleri ----------------
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('target_depth', 2.0)       # [m]
        self.declare_parameter('dive_start_s', 5.0)         # yuzeyde bekleme suresi
        self.declare_parameter('dive_duration_s', 10.0)      # dalis suresi
        self.declare_parameter('cruise_duration_s', 25.0)    # duz seyir suresi
        self.declare_parameter('ascend_duration_s', 10.0)    # yuzeye cikis suresi
        self.declare_parameter('yaw_rate_cruise', 0.05)      # duz seyirde donus hizi [rad/s]
        self.declare_parameter('rho', 1000.0)
        self.declare_parameter('g', 9.80665)
        self.declare_parameter('surface_pressure_pa', 101325.0)  # P0 - deniz seviyesi ~1 atm
        self.declare_parameter('pressure_noise_std', 15.0)   # [Pa]
        self.declare_parameter('imu_noise_std', 0.01)         # [rad] / [rad/s]
        self.declare_parameter('loop_scenario', True)          # senaryo bitince basa dons0n mu

        self.target_depth = self.get_parameter('target_depth').value
        self.dive_start = self.get_parameter('dive_start_s').value
        self.dive_dur = self.get_parameter('dive_duration_s').value
        self.cruise_dur = self.get_parameter('cruise_duration_s').value
        self.ascend_dur = self.get_parameter('ascend_duration_s').value
        self.yaw_rate_cruise = self.get_parameter('yaw_rate_cruise').value
        self.rho = self.get_parameter('rho').value
        self.g = self.get_parameter('g').value
        self.p0 = self.get_parameter('surface_pressure_pa').value
        self.pressure_noise = self.get_parameter('pressure_noise_std').value
        self.imu_noise = self.get_parameter('imu_noise_std').value
        self.loop_scenario = self.get_parameter('loop_scenario').value

        self.dive_end = self.dive_start + self.dive_dur
        self.cruise_end = self.dive_end + self.cruise_dur
        self.ascend_end = self.cruise_end + self.ascend_dur
        self.scenario_total = self.ascend_end + 5.0  # sonunda birazyuzeyde bekle

        pub_rate = float(self.get_parameter('publish_rate_hz').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._imu_pub = self.create_publisher(Imu, '/mavros/imu/data', qos)
        self._pressure_pub = self.create_publisher(FluidPressure, '/sara/pressure', qos)
        self._water1_pub = self.create_publisher(Bool, '/sara/water_detect_1', qos)
        self._water2_pub = self.create_publisher(Bool, '/sara/water_detect_2', qos)

        self._start_time = self.get_clock().now()
        self._heading = 0.0

        self.create_timer(1.0 / pub_rate, self._on_timer)

        self.get_logger().info(
            'mock_sensors baslatildi. Senaryo: '
            f'{self.dive_start:.0f}s yuzeyde -> {self.dive_dur:.0f}s dalis '
            f'(0->{self.target_depth:.1f}m) -> {self.cruise_dur:.0f}s duz seyir -> '
            f'{self.ascend_dur:.0f}s yuzeye cikis. Bu SADECE test yayinicidir, '
            'gercek sensor baglaninca kapatilmalidir.'
        )

    def _elapsed(self) -> float:
        t = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if self.loop_scenario and self.scenario_total > 0.0:
            t = t % self.scenario_total
        return t

    def _depth_for_time(self, t: float) -> float:
        """Senaryo fazina gore hedef derinligi dondurur (basit lineer profil)."""
        if t < self.dive_start:
            return 0.0
        elif t < self.dive_end:
            ratio = (t - self.dive_start) / self.dive_dur
            return self.target_depth * ratio
        elif t < self.cruise_end:
            return self.target_depth
        elif t < self.ascend_end:
            ratio = (t - self.cruise_end) / self.ascend_dur
            return self.target_depth * (1.0 - ratio)
        else:
            return 0.0

    def _on_timer(self):
        t = self._elapsed()
        depth = self._depth_for_time(t)
        now_msg = self.get_clock().now().to_msg()

        # --- Basinc sensoru simulasyonu ---
        # h = (P - P0)/(rho*g)  ->  P = P0 + rho*g*h
        pressure = self.p0 + self.rho * self.g * depth
        pressure += random.gauss(0.0, self.pressure_noise)

        pmsg = FluidPressure()
        pmsg.header.stamp = now_msg
        pmsg.header.frame_id = 'sara_pressure_sensor'
        pmsg.fluid_pressure = pressure
        pmsg.variance = self.pressure_noise ** 2
        self._pressure_pub.publish(pmsg)

        # --- IMU simulasyonu ---
        # Duz seyir fazinda yavasca donsun, dalis/cikis fazinda hafif pitch versin
        if self.dive_start <= t < self.dive_end:
            pitch = -0.15  # burun asagi (dalis)
            yaw_rate = 0.0
        elif self.dive_end <= t < self.cruise_end:
            pitch = 0.0
            yaw_rate = self.yaw_rate_cruise
        elif self.cruise_end <= t < self.ascend_end:
            pitch = 0.15  # burun yukari (cikis)
            yaw_rate = 0.0
        else:
            pitch = 0.0
            yaw_rate = 0.0

        self._heading += yaw_rate / self._pub_hz_safe()
        roll = random.gauss(0.0, self.imu_noise)
        pitch_noisy = pitch + random.gauss(0.0, self.imu_noise)

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch_noisy, self._heading)

        imu = Imu()
        imu.header.stamp = now_msg
        imu.header.frame_id = 'sara_imu'
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.x = random.gauss(0.0, self.imu_noise)
        imu.angular_velocity.y = random.gauss(0.0, self.imu_noise)
        imu.angular_velocity.z = yaw_rate + random.gauss(0.0, self.imu_noise)
        # Sakin seyirde ivme ~ sadece yercekimi (motion_consistent=True olmasi icin)
        imu.linear_acceleration.x = random.gauss(0.0, self.imu_noise)
        imu.linear_acceleration.y = random.gauss(0.0, self.imu_noise)
        imu.linear_acceleration.z = self.g + random.gauss(0.0, self.imu_noise)
        self._imu_pub.publish(imu)

        # --- Su var/yok sensoru simulasyonu (SEN0368 x2) ---
        # True = su algilandi (su altinda), False = su yok (yuzeyde)
        submerged = depth > 0.05
        w1 = Bool()
        w1.data = submerged
        w2 = Bool()
        w2.data = submerged
        self._water1_pub.publish(w1)
        self._water2_pub.publish(w2)

    def _pub_hz_safe(self):
        # heading entegrasyonu icin sabit yayin frekansini kullanir
        return max(1.0, float(self.get_parameter('publish_rate_hz').value))


def main(args=None):
    rclpy.init(args=args)
    node = MockSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()