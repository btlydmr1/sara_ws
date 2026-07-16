#!/usr/bin/env python3
"""
navigation.py
==============
SARA platformu - Navigasyon Katmani (sara_control paketi icinde)
Ontasarim Raporu Tablo 13 uygulamasi - SADECE STANDART ROS2 MESAJLARI

Su an aktif olan girdi : Pixhawk 6X IMU  (/mavros/imu/data)
Ileride baglanacak     : Basinc sensoru  (/sara/pressure)
                          SEN0368 su var/yok x2 (/sara/water_detect_1, _2)
Bu sensorler bagli olmasa bile dugum calisir; ilgili ciktilar
"gecersiz/beklemede" olarak isaretlenir (depth_valid=False vb.),
kablolama tamamlaninca otomatik devreye girer - kod degisikligi gerekmez.

Veri akisi (Tablo 13 semasi):
    Veri Toplama & Kalibrasyon
        -> Filtreleme & Sensor Fuzyonu (hareketli ortalama, LPF)
        -> Derinlik Kestirimi h=(P-P0)/(rho*g)
           Yonelim Kestirimi (roll, pitch, yaw/heading, acisal hiz)
        -> Yaklasik Ilerleme / Hareket Tutarliligi (MUTLAK KONUM DEGIL,
           GPS/DVL yok - sadece gorev fazi/guvenli atis bolgesi karari
           icin yardimci kestirim)
        -> Durum Bilgisi Uretimi -> nav_msgs/Odometry + Bool + DiagnosticStatus

Cikti topic'leri:
    /sara/navigation/odom     (nav_msgs/Odometry)
        pose.position.z    = derinlik [m] (pozitif = su altina dogru)
        pose.position.x/y  = yaklasik ilerleme x_yaklasik / y_yaklasik [m]
        pose.orientation   = quaternion (roll, pitch, heading/yaw)
        twist.linear.x     = v_kalibre (hareket tutarliyken), aksi halde 0
        twist.angular      = filtrelenmis acisal hizlar (roll/pitch/yaw rate)
    /sara/navigation/surface_detected  (std_msgs/Bool)
    /sara/navigation/status            (diagnostic_msgs/DiagnosticStatus)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def wrap_0_2pi(angle: float) -> float:
    """Heading'i [0, 2*pi) araligina sarar (0/360 gecis hatalarini onler)."""
    return angle % (2.0 * math.pi)


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """(roll, pitch, yaw) [rad] -> quaternion (x, y, z, w), aerospace ZYX sirasi."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    """Quaternion -> (roll, pitch, yaw) [rad], aerospace (ZYX) sirasi."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class LowPassFilter:
    """Basit eksponansiyel alcak geciren filtre (LPF)."""

    def __init__(self, alpha: float):
        self.alpha = alpha
        self._y = None

    def update(self, x: float) -> float:
        self._y = x if self._y is None else self.alpha * x + (1.0 - self.alpha) * self._y
        return self._y


class MovingAverage:
    """Sabit pencereli hareketli ortalama filtresi."""

    def __init__(self, window: int):
        self._buf = deque(maxlen=max(1, window))

    def update(self, x: float) -> float:
        self._buf.append(x)
        return sum(self._buf) / len(self._buf)


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # ---------------- Parametreler ----------------
        self.declare_parameter('rho', 1000.0)                    # su yogunlugu [kg/m^3]
        self.declare_parameter('g', 9.80665)
        self.declare_parameter('pressure_calib_samples', 50)
        self.declare_parameter('lpf_alpha_pressure', 0.2)
        self.declare_parameter('imu_gyro_window', 5)
        self.declare_parameter('v_kalibre', 0.5)                  # [m/s] - testle guncellenecek
        self.declare_parameter('enable_xy_estimate', True)
        self.declare_parameter('pixhawk_timeout_sec', 1.0)
        self.declare_parameter('pressure_timeout_sec', 2.0)
        self.declare_parameter('water_sensor_timeout_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('water_fusion_mode', 'or')         # 'or' -> temkinli
        self.declare_parameter('accel_still_threshold', 0.6)
        self.declare_parameter('gyro_consistent_threshold', 2.0)

        self.rho = self.get_parameter('rho').value
        self.g = self.get_parameter('g').value
        self.calib_samples_needed = int(self.get_parameter('pressure_calib_samples').value)
        self.v_kalibre = self.get_parameter('v_kalibre').value
        self.enable_xy = self.get_parameter('enable_xy_estimate').value
        self.pixhawk_timeout = self.get_parameter('pixhawk_timeout_sec').value
        self.pressure_timeout = self.get_parameter('pressure_timeout_sec').value
        self.water_timeout = self.get_parameter('water_sensor_timeout_sec').value
        self.water_fusion_mode = self.get_parameter('water_fusion_mode').value
        self.accel_still_th = self.get_parameter('accel_still_threshold').value
        self.gyro_consistent_th = self.get_parameter('gyro_consistent_threshold').value
        pub_rate = float(self.get_parameter('publish_rate_hz').value)

        # ---------------- Filtreler ----------------
        self._pressure_lpf = LowPassFilter(self.get_parameter('lpf_alpha_pressure').value)
        gyro_win = int(self.get_parameter('imu_gyro_window').value)
        self._gyro_x_avg = MovingAverage(gyro_win)
        self._gyro_y_avg = MovingAverage(gyro_win)
        self._gyro_z_avg = MovingAverage(gyro_win)

        # ---------------- Kalibrasyon ----------------
        self._p0 = None
        self._calib_buffer = []
        self._calibrated = False

        # ---------------- Anlik durum ----------------
        self._depth = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._heading = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._yaw_rate = 0.0
        self._motion_consistent = False

        self._water_1 = None
        self._water_2 = None
        self._surface_detected = False

        self._last_imu_stamp = None
        self._last_pressure_stamp = None
        self._last_water_stamp = None

        self._approx_distance = 0.0
        self._approx_x = 0.0
        self._approx_y = 0.0
        self._last_tick_time = self.get_clock().now()

        # ---------------- QoS ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- Abonelikler (Girdi) ----------------
        # AKTIF: Pixhawk 6X dahili IMU
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu, sensor_qos)

        # HAZIR-BEKLEMEDE: ileride kablolanacak sensorler.
        # Yayin baslamadan hic mesaj gelmez, node calismaya devam eder;
        # ilgili ciktilar (depth_valid, surface_detected vb.) buna gore isaretlenir.
        self.create_subscription(FluidPressure, '/sara/pressure', self._on_pressure, sensor_qos)
        self.create_subscription(Bool, '/sara/water_detect_1', self._on_water_1, sensor_qos)
        self.create_subscription(Bool, '/sara/water_detect_2', self._on_water_2, sensor_qos)

        # Gorev yonetiminden "referanslama / kalibrasyonu yenile" komutu (Tablo 12)
        self.create_subscription(Empty, '/sara/navigation/recalibrate', self._on_recalibrate, 10)

        # ---------------- Yayinlar (Cikti) ----------------
        self._odom_pub = self.create_publisher(Odometry, '/sara/navigation/odom', 10)
        self._surface_pub = self.create_publisher(Bool, '/sara/navigation/surface_detected', 10)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/navigation/status', 10)

        # ---------------- Zamanlayici ----------------
        self.create_timer(1.0 / pub_rate, self._on_timer)

        self.get_logger().info(
            'navigation_node baslatildi. IMU aktif (/mavros/imu/data). '
            f'Basinc kalibrasyonu icin {self.calib_samples_needed} ornek bekleniyor '
            '(basinc sensoru baglanmadiysa depth_valid=False kalacaktir).'
        )

        # Abonelik/yayin listesini acikca logla - "Subscribers bos gorunuyor"
        # gibi build/cache kaynakli sorunlari hizli teshis etmek icin.
        subs = [
            '/mavros/imu/data [sensor_msgs/Imu]',
            '/sara/pressure [sensor_msgs/FluidPressure]',
            '/sara/water_detect_1 [std_msgs/Bool]',
            '/sara/water_detect_2 [std_msgs/Bool]',
            '/sara/navigation/recalibrate [std_msgs/Empty]',
        ]
        pubs = [
            '/sara/navigation/odom [nav_msgs/Odometry]',
            '/sara/navigation/surface_detected [std_msgs/Bool]',
            '/sara/navigation/status [diagnostic_msgs/DiagnosticStatus]',
        ]
        self.get_logger().info('Abone olunan topicler: ' + ', '.join(subs))
        self.get_logger().info('Yayinlanan topicler: ' + ', '.join(pubs))

    # ======================================================================
    # Kalibrasyon
    # ======================================================================
    def _on_recalibrate(self, _msg: Empty):
        self.get_logger().warn('Yeniden kalibrasyon istegi alindi (P0 sifirlaniyor).')
        self._p0 = None
        self._calib_buffer = []
        self._calibrated = False
        self._approx_distance = 0.0
        self._approx_x = 0.0
        self._approx_y = 0.0

    # ======================================================================
    # Sensor callback'leri
    # ======================================================================
    def _on_imu(self, msg: Imu):
        self._last_imu_stamp = self.get_clock().now()

        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self._roll = roll
        self._pitch = pitch
        self._heading = wrap_0_2pi(yaw)

        self._roll_rate = self._gyro_x_avg.update(msg.angular_velocity.x)
        self._pitch_rate = self._gyro_y_avg.update(msg.angular_velocity.y)
        self._yaw_rate = self._gyro_z_avg.update(msg.angular_velocity.z)

        accel = msg.linear_acceleration
        accel_norm = math.sqrt(accel.x ** 2 + accel.y ** 2 + accel.z ** 2)
        accel_dev = abs(accel_norm - self.g)
        gyro_mag = math.sqrt(self._roll_rate ** 2 + self._pitch_rate ** 2 + self._yaw_rate ** 2)

        # Basit sezgisel esik - lab/su ici testlerle iyilestirilecek (rapor - PID bolumu)
        self._motion_consistent = (
            accel_dev < self.accel_still_th * 3.0
            and gyro_mag < self.gyro_consistent_th
        )

    def _on_pressure(self, msg: FluidPressure):
        self._last_pressure_stamp = self.get_clock().now()
        filtered = self._pressure_lpf.update(msg.fluid_pressure)

        if not self._calibrated:
            self._calib_buffer.append(filtered)
            if len(self._calib_buffer) >= self.calib_samples_needed:
                self._p0 = sum(self._calib_buffer) / len(self._calib_buffer)
                self._calibrated = True
                self.get_logger().info(f'Yuzey referans basinci P0 = {self._p0:.1f} Pa belirlendi.')
            return

        self._depth = max(0.0, (filtered - self._p0) / (self.rho * self.g))

    def _on_water_1(self, msg: Bool):
        self._last_water_stamp = self.get_clock().now()
        self._water_1 = msg.data
        self._fuse_water_sensors()

    def _on_water_2(self, msg: Bool):
        self._last_water_stamp = self.get_clock().now()
        self._water_2 = msg.data
        self._fuse_water_sensors()

    def _fuse_water_sensors(self):
        if self._water_1 is None or self._water_2 is None:
            return
        if self.water_fusion_mode == 'and':
            water_present = self._water_1 and self._water_2
        else:  # 'or' (varsayilan, guvenli taraf)
            water_present = self._water_1 or self._water_2
        self._surface_detected = not water_present

    # ======================================================================
    # Zamanlayici - Yaklasik Ilerleme + Durum Bilgisi Uretimi
    # ======================================================================
    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick_time).nanoseconds * 1e-9
        self._last_tick_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0

        # --- Yaklasik Ilerleme / Hareket Tutarliligi ---
        if self._motion_consistent and dt > 0.0:
            self._approx_distance += self.v_kalibre * dt
            if self.enable_xy:
                self._approx_x += self.v_kalibre * math.cos(self._heading) * dt
                self._approx_y += self.v_kalibre * math.sin(self._heading) * dt

        # --- Sensor "canlilik" kontrolleri (henuz baglanmamis sensorler icin) ---
        pixhawk_connected = self._is_fresh(self._last_imu_stamp, self.pixhawk_timeout)
        pressure_active = self._is_fresh(self._last_pressure_stamp, self.pressure_timeout)
        water_active = self._is_fresh(self._last_water_stamp, self.water_timeout)

        # --- Odometry mesaji ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'sara_odom'
        odom.child_frame_id = 'sara_base_link'

        odom.pose.pose.position.x = float(self._approx_x)
        odom.pose.pose.position.y = float(self._approx_y)
        odom.pose.pose.position.z = float(self._depth)  # derinlik, pozitif = su altina dogru

        qx, qy, qz, qw = euler_to_quaternion(self._roll, self._pitch, self._heading)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(self.v_kalibre) if self._motion_consistent else 0.0
        odom.twist.twist.angular.x = float(self._roll_rate)
        odom.twist.twist.angular.y = float(self._pitch_rate)
        odom.twist.twist.angular.z = float(self._yaw_rate)

        # covariance[0] alanini depth_valid/motion_consistent gostergesi olarak
        # KULLANMIYORUZ (yanlis okunmaya acik) - onun yerine ayri status mesaji var.
        self._odom_pub.publish(odom)

        # --- Yuzey konumu ---
        surface_msg = Bool()
        surface_msg.data = bool(self._surface_detected)
        self._surface_pub.publish(surface_msg)

        # --- Durum / tani mesaji ---
        status = DiagnosticStatus()
        status.name = 'sara_navigation'
        status.hardware_id = 'jetson_orin_nano'

        if not pixhawk_connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Pixhawk IMU baglantisi yok / zaman asimi'
        elif not self._calibrated:
            status.level = DiagnosticStatus.WARN
            status.message = 'Derinlik kalibrasyonu bekleniyor (basinc sensoru baglantisini kontrol edin)'
        elif not water_active:
            status.level = DiagnosticStatus.WARN
            status.message = 'SEN0368 su var/yok sensorlerinden veri gelmiyor'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Nominal'

        status.values = [
            KeyValue(key='pixhawk_connected', value=str(pixhawk_connected)),
            KeyValue(key='depth_valid', value=str(self._calibrated)),
            KeyValue(key='pressure_sensor_active', value=str(pressure_active)),
            KeyValue(key='water_sensors_active', value=str(water_active)),
            KeyValue(key='motion_consistent', value=str(self._motion_consistent)),
            KeyValue(key='surface_detected', value=str(self._surface_detected)),
            KeyValue(key='approx_distance_m', value=f'{self._approx_distance:.2f}'),
        ]
        self._status_pub.publish(status)

    def _is_fresh(self, stamp, timeout_sec) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < timeout_sec


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
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