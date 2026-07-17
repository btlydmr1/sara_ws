#!/usr/bin/env python3
"""
vehicle_sim.py
===============
SARA platformu - KAPALI CEVRIM Arac Simulatoru (mock_sensors.py'nin yerini alir)

mock_sensors.py ONCEDEN YAZILMIS bir senaryoyu zamana gore oynatiyordu ve
hicbir komutu DINLEMIYORDU (acik cevrim). Bu node ise safety_node'un
urettigi NIHAI komutlari (*_command) okuyup BASIT BIR FIZIK MODELIYLE
aracin derinlik/heading/pitch durumunu gercekten gunceller, sonra bu
simule edilmis duruma gore sahte sensor verisi (basinc, IMU, su var/yok)
uretir. Boylece TUM zincir (navigation -> guidance -> autopilot -> safety
-> [bu node] -> navigation ...) gercek KAPALI CEVRIM olarak test edilebilir.

Fizik modeli (BILEREK BASIT TUTULMUSTUR - gercek arac dinamigi degildir,
sadece kontrol mantigini kapali cevrim test etmek icindir):

    yaw_rate     = k_yaw * fin_yaw_command
    heading      += yaw_rate * dt

    pitch_rate   = k_pitch * fin_pitch_command  (kanatcik, HIZLI etki)
                 + k_buoyancy_pitch * buoyancy_command (sephiye de hafif pitch uretir)
    pitch        += pitch_rate * dt  (yumusatilmis, sinirlandirilmis)

    forward_speed = k_thrust * thrust_command
    depth_rate_from_buoyancy = +k_buoyancy_depth * buoyancy_command  (pozitif=dal, YAVAS/UZUN SURELI)
    depth_rate_from_pitch    = -forward_speed * sin(pitch)            (pozitif pitch=burun yukari=yuzeye cikis)
    depth       += (depth_rate_from_buoyancy + depth_rate_from_pitch) * dt

    x += forward_speed * cos(heading) * dt
    y += forward_speed * sin(heading) * dt

Girdi (safety_node'un NIHAI komutlari):
    /sara/control/thrust_command      (std_msgs/Float64)
    /sara/control/fin_command         (geometry_msgs/Vector3)   x=pitch, y=yaw
    /sara/control/buoyancy_command    (std_msgs/Float64)

Cikti (navigation_node'un bekledigi TAM AYNI sensor topic'leri):
    /mavros/imu/data              (sensor_msgs/Imu)
    /sara/pressure                 (sensor_msgs/FluidPressure)
    /sara/water_detect_1           (std_msgs/Bool)
    /sara/water_detect_2           (std_msgs/Bool)

NOT: mock_sensors.py ile AYNI ANDA calistirilmamalidir (ikisi de ayni
topic'lere yayin yapar, cakisir). Kapali cevrim test icin mock_sensors
yerine BUNU calistirin.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu, FluidPressure
from geometry_msgs.msg import Vector3


def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class VehicleSimNode(Node):

    def __init__(self):
        super().__init__('vehicle_sim')

        # ================= Fizik model katsayilari (TODO: gercek arac testleriyle iyilestirilecek) =================
        self.declare_parameter('k_yaw', 0.6)                  # fin_yaw_command -> yaw_rate [rad/s per unit]
        self.declare_parameter('k_pitch', 0.5)                 # fin_pitch_command -> pitch_rate [rad/s per unit]
        self.declare_parameter('k_buoyancy_pitch', 0.05)        # buoyancy_command'in pitch'e yan etkisi
        self.declare_parameter('k_thrust', 0.8)                  # thrust_command -> ileri hiz [m/s per unit]
        self.declare_parameter('k_buoyancy_depth', 0.25)          # buoyancy_command -> derinlik degisim hizi [m/s per unit]
        self.declare_parameter('pitch_limit_rad', 1.2)              # fiziksel pitch siniri (~69 derece)
        self.declare_parameter('max_depth', 10.0)                    # havuz/deniz azami derinligi (guvenlik siniri, simulasyonda)

        # ================= Sensor simulasyon parametreleri (mock_sensors.py ile ayni) =================
        self.declare_parameter('rho', 1000.0)
        self.declare_parameter('g', 9.80665)
        self.declare_parameter('surface_pressure_pa', 101325.0)
        self.declare_parameter('pressure_noise_std', 15.0)
        self.declare_parameter('imu_noise_std', 0.01)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.k_yaw = self.get_parameter('k_yaw').value
        self.k_pitch = self.get_parameter('k_pitch').value
        self.k_buoyancy_pitch = self.get_parameter('k_buoyancy_pitch').value
        self.k_thrust = self.get_parameter('k_thrust').value
        self.k_buoyancy_depth = self.get_parameter('k_buoyancy_depth').value
        self.pitch_limit = self.get_parameter('pitch_limit_rad').value
        self.max_depth = self.get_parameter('max_depth').value

        self.rho = self.get_parameter('rho').value
        self.g = self.get_parameter('g').value
        self.p0 = self.get_parameter('surface_pressure_pa').value
        self.pressure_noise = self.get_parameter('pressure_noise_std').value
        self.imu_noise = self.get_parameter('imu_noise_std').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        # ================= Simule edilen arac durumu (baslangic: yuzeyde, referans heading 0) =================
        self._depth = 0.0
        self._heading = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._x = 0.0
        self._y = 0.0

        # Son alinan komutlar (gelmezse guvenli varsayilan: 0)
        self._thrust_cmd = 0.0
        self._fin_pitch_cmd = 0.0
        self._fin_yaw_cmd = 0.0
        self._buoyancy_cmd = 0.0

        self._last_sim_time = self.get_clock().now()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # ================= Abonelikler - safety_node'un NIHAI komutlari =================
        self.create_subscription(Float64, '/sara/control/thrust_command', self._on_thrust, 10)
        self.create_subscription(Vector3, '/sara/control/fin_command', self._on_fin, 10)
        self.create_subscription(Float64, '/sara/control/buoyancy_command', self._on_buoyancy, 10)

        # ================= Yayinlar - navigation_node'un bekledigi sensor topic'leri =================
        self._imu_pub = self.create_publisher(Imu, '/mavros/imu/data', qos)
        self._pressure_pub = self.create_publisher(FluidPressure, '/sara/pressure', qos)
        self._water1_pub = self.create_publisher(Bool, '/sara/water_detect_1', qos)
        self._water2_pub = self.create_publisher(Bool, '/sara/water_detect_2', qos)

        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            'vehicle_sim baslatildi (KAPALI CEVRIM). safety_node komutlarini dinliyor, '
            'basit fizik modeliyle aracin durumunu gerceklestirip sahte sensor uretiyor. '
            'mock_sensors.py ile AYNI ANDA CALISTIRMAYIN.'
        )

    # ======================================================================
    def _on_thrust(self, msg: Float64):
        self._thrust_cmd = msg.data

    def _on_fin(self, msg: Vector3):
        self._fin_pitch_cmd = msg.x
        self._fin_yaw_cmd = msg.y

    def _on_buoyancy(self, msg: Float64):
        self._buoyancy_cmd = msg.data

    # ======================================================================
    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_sim_time).nanoseconds * 1e-9
        self._last_sim_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.0

        # --- Basit fizik modeli ---
        yaw_rate = self.k_yaw * self._fin_yaw_cmd
        self._heading = (self._heading + yaw_rate * dt) % (2.0 * math.pi)

        pitch_rate = self.k_pitch * self._fin_pitch_cmd + self.k_buoyancy_pitch * self._buoyancy_cmd
        self._pitch = max(-self.pitch_limit, min(self.pitch_limit, self._pitch + pitch_rate * dt))

        forward_speed = self.k_thrust * self._thrust_cmd

        depth_rate = (
            self.k_buoyancy_depth * self._buoyancy_cmd     # sephiye: pozitif komut = dal (derinlik artar), uzun sureli/yavas
            - forward_speed * math.sin(self._pitch)          # pozitif pitch (burun yukari) = yuzeye cikis yonunde etki
        )
        self._depth = max(0.0, min(self.max_depth, self._depth + depth_rate * dt))

        self._x += forward_speed * math.cos(self._heading) * dt
        self._y += forward_speed * math.sin(self._heading) * dt

        self._publish_sensors()

    # ======================================================================
    def _publish_sensors(self):
        now_msg = self.get_clock().now().to_msg()

        pressure = self.p0 + self.rho * self.g * self._depth
        pmsg = FluidPressure()
        pmsg.header.stamp = now_msg
        pmsg.header.frame_id = 'sara_pressure_sensor'
        pmsg.fluid_pressure = pressure
        pmsg.variance = self.pressure_noise ** 2
        self._pressure_pub.publish(pmsg)

        qx, qy, qz, qw = euler_to_quaternion(self._roll, self._pitch, self._heading)
        imu = Imu()
        imu.header.stamp = now_msg
        imu.header.frame_id = 'sara_imu'
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.z = self.k_yaw * self._fin_yaw_cmd
        imu.angular_velocity.y = self.k_pitch * self._fin_pitch_cmd
        imu.linear_acceleration.z = self.g
        self._imu_pub.publish(imu)

        submerged = self._depth > 0.05
        w1 = Bool()
        w1.data = submerged
        w2 = Bool()
        w2.data = submerged
        self._water1_pub.publish(w1)
        self._water2_pub.publish(w2)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimNode()
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