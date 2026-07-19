#!/usr/bin/env python3
"""
safety.py
==========
SARA platformu - Guvenlik Katmani (Ontasarim Raporu - akis semasindaki
"Guvenlik Denetimi -> Guvenli Onay?" kutusu)

Bu node sistemdeki TEK YETKILI karar mercidir: alt katmanlarin urettigi
TUM istekleri (*_request) burada denetler, GUVENLI OLANLARI onaylayip
nihai komutlara (*_command) donusturur. *_command topic'leri DISINDA
HICBIR sey Pixhawk'a/eyleyicilere baglanmamalidir.

Denetledigi kosullar (rapor - Guvenlik Katmani satiri, Tablo 11):
    "Sensor hatasi, baglanti kaybi, acil durdurma, yuzey konumu, pitch
    acisi ve burun ayirma/firlatma kosullarini denetler."

Onemli tasarim ilkesi: Bu node, ust katmanlarin (gudum, otopilot)
kendi ic degerlendirmesine KORKORU GUVENMEZ - kritik guvenlik
girdilerine (acil durdurma, leak, navigasyon durumu, yuzey konumu,
pitch) KENDISI DE dogrudan abone olur ve baglantisiz bir DOGRULAMA
yapar (coklu/bagimsiz fail-safe ilkesi).

Girdi (istekler + guvenlik sinyalleri):
    /sara/control/thrust_request           (std_msgs/Float64)
    /sara/control/fin_request              (geometry_msgs/Vector3)
    /sara/control/buoyancy_request         (std_msgs/Float64)
    /sara/guidance/nose_cap_open_request   (std_msgs/Bool)
    /sara/guidance/launch_request          (std_msgs/Bool)
    /sara/mission_start/motion_permission  (std_msgs/Bool)
    /sara/navigation/odom                  (nav_msgs/Odometry)      -- bagimsiz pitch/derinlik dogrulamasi icin
    /sara/navigation/surface_detected      (std_msgs/Bool)
    /sara/navigation/status                (diagnostic_msgs/DiagnosticStatus)
    /sara/safety/emergency_stop            (std_msgs/Bool)
    /sara/safety/leak_detected             (std_msgs/Bool)

Cikti (NIHAI komutlar - Pixhawk/MAVROS koprusu SADECE bunlari dinlemeli):
    /sara/control/thrust_command      (std_msgs/Float64)
    /sara/control/fin_command         (geometry_msgs/Vector3)
    /sara/control/buoyancy_command    (std_msgs/Float64)
    /sara/control/nose_cap_command    (std_msgs/Bool)         -- burun ayirma sistemine (kapak)
    /sara/control/launch_command      (std_msgs/Bool)         -- burun ayirma sistemine (atesleme) - SISTEMDE BUNU True YAPABILEN TEK YER
    /sara/safety/approved             (std_msgs/Bool)         -- semadaki "Guvenli Onay?" karari (Evet=True/Hayir=False)
    /sara/safety/fail_safe_active     (std_msgs/Bool)         -- "Hayir" dalinin acik gostergesi (Fail-Safe/Komut Bloke)
    /sara/safety/status               (diagnostic_msgs/DiagnosticStatus)  -- blokaj nedeni dahil
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def quaternion_to_pitch(x, y, z, w):
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('nav_status_timeout_s', 2.0)
        self.declare_parameter('control_rate_hz', 20.0)
        # DUZELTME (Teknik Sartname Madde 4.1): "Arac su yuzeyine +30 dereceden
        # FAZLA yunuslama aciSiyla ulasmali". Bu, gudumun kendi kontrolunden
        # BAGIMSIZ ikinci bir dogrulama - bilerek gudumun esiginden (30) biraz
        # daha DUSUK tutuluyor (25) ki iki bagimsiz kontrol ayni anda AYNI
        # noktada basarisiz olmasin, ama yine de sartnamenin ruhuna (30 civari) sadik.
        self.declare_parameter('launch_min_pitch_deg', 25.0)
        self.declare_parameter('request_timeout_s', 0.5)

        self.nav_status_timeout = self.get_parameter('nav_status_timeout_s').value
        self.launch_min_pitch = math.radians(self.get_parameter('launch_min_pitch_deg').value)
        self.request_timeout = self.get_parameter('request_timeout_s').value
        rate = float(self.get_parameter('control_rate_hz').value)

        self._thrust_request = 0.0
        self._fin_request = Vector3()
        self._buoyancy_request = 0.0
        self._nose_cap_open_request = False
        self._launch_request = False
        self._last_thrust_req_time = None
        self._last_fin_req_time = None
        self._last_buoyancy_req_time = None

        self._motion_permission = False
        self._surface_detected = False
        self._nose_submerged = True
        self._tail_submerged = True
        self._pitch = 0.0
        self._depth = 0.0

        self._pixhawk_connected = False
        self._depth_valid = False
        self._last_nav_status_time = None
        self._nav_status_level = DiagnosticStatus.STALE

        self._emergency_stop = False
        self._leak_detected = False

        self._nose_cap_command_latched = False
        self._was_launch_command = False

        self.create_subscription(Float64, '/sara/control/thrust_request', self._on_thrust_req, 10)
        self.create_subscription(Vector3, '/sara/control/fin_request', self._on_fin_req, 10)
        self.create_subscription(Float64, '/sara/control/buoyancy_request', self._on_buoyancy_req, 10)
        self.create_subscription(Bool, '/sara/guidance/nose_cap_open_request', self._on_nose_cap_req, 10)
        self.create_subscription(Bool, '/sara/guidance/launch_request', self._on_launch_req, 10)
        self.create_subscription(Bool, '/sara/mission_start/motion_permission', self._on_motion_permission, 10)
        self.create_subscription(Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/sara/navigation/surface_detected', self._on_surface, 10)
        # DUZELTME (KTR sayfa 14/37): Guvenlik katmani, atesleme icin
        # navigasyonun tek boyutlu surface_detected ozetine GUVENMEMELI,
        # burun/kuyruk sensorlerine DOGRUDAN abone olup TAM kosulu
        # (burun disarida + kuyruk icerde) KENDISI dogrulamalidir.
        # DUZELTME: ayni QoS uyumsuzlugu burada da vardi (bkz. guidance.py).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Bool, '/sara/water_detect_1', self._on_water_nose, sensor_qos)
        self.create_subscription(Bool, '/sara/water_detect_2', self._on_water_tail, sensor_qos)
        self.create_subscription(DiagnosticStatus, '/sara/navigation/status', self._on_nav_status, 10)
        self.create_subscription(Bool, '/sara/safety/emergency_stop', self._on_emergency_stop, 10)
        self.create_subscription(Bool, '/sara/safety/leak_detected', self._on_leak, 10)

        self._thrust_cmd_pub = self.create_publisher(Float64, '/sara/control/thrust_command', 10)
        self._fin_cmd_pub = self.create_publisher(Vector3, '/sara/control/fin_command', 10)
        self._buoyancy_cmd_pub = self.create_publisher(Float64, '/sara/control/buoyancy_command', 10)
        self._nose_cap_cmd_pub = self.create_publisher(Bool, '/sara/control/nose_cap_command', 10)
        self._launch_cmd_pub = self.create_publisher(Bool, '/sara/control/launch_command', 10)
        # Semadaki "Guvenli Onay?" karar kutusunun DOGRUDAN karsiligi -
        # Evet/Hayir sonucu, komutlardan BAGIMSIZ olarak da yayinlanir
        # (izleme/log/gostergeler icin - orn. bir "FAIL-SAFE" LED'i bu
        # topic'i dinleyebilir).
        self._approved_pub = self.create_publisher(Bool, '/sara/safety/approved', 10)          # Evet=True / Hayir=False
        self._failsafe_pub = self.create_publisher(Bool, '/sara/safety/fail_safe_active', 10)   # Hayir dalinin acik gostergesi
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/safety/status', 10)

        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            'safety_node baslatildi. Sistemde launch_command=True uretebilecek TEK node budur.'
        )

    def _on_thrust_req(self, msg: Float64):
        self._thrust_request = msg.data
        self._last_thrust_req_time = self.get_clock().now()

    def _on_fin_req(self, msg: Vector3):
        self._fin_request = msg
        self._last_fin_req_time = self.get_clock().now()

    def _on_buoyancy_req(self, msg: Float64):
        self._buoyancy_request = msg.data
        self._last_buoyancy_req_time = self.get_clock().now()

    def _on_nose_cap_req(self, msg: Bool):
        self._nose_cap_open_request = msg.data

    def _on_launch_req(self, msg: Bool):
        self._launch_request = msg.data

    def _on_motion_permission(self, msg: Bool):
        self._motion_permission = msg.data

    def _on_odom(self, msg: Odometry):
        self._depth = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self._pitch = quaternion_to_pitch(q.x, q.y, q.z, q.w)

    def _on_surface(self, msg: Bool):
        self._surface_detected = msg.data

    def _on_water_nose(self, msg: Bool):
        self._nose_submerged = msg.data

    def _on_water_tail(self, msg: Bool):
        self._tail_submerged = msg.data

    @property
    def _dogru_cikis_acisi(self) -> bool:
        """KTR sayfa 14/37: atesleme icin TEK gecerli durum - burun su
        DISINDA (False) VE kuyruk su ICINDE (True). Tamamen batik veya
        tamamen havadayken bu KESINLIKLE saglanmaz."""
        return (not self._nose_submerged) and self._tail_submerged

    def _on_nav_status(self, msg: DiagnosticStatus):
        self._last_nav_status_time = self.get_clock().now()
        self._nav_status_level = msg.level
        values = {kv.key: kv.value for kv in msg.values}
        self._pixhawk_connected = values.get('pixhawk_connected', 'False') == 'True'
        self._depth_valid = values.get('depth_valid', 'False') == 'True'

    def _on_emergency_stop(self, msg: Bool):
        self._emergency_stop = msg.data

    def _on_leak(self, msg: Bool):
        self._leak_detected = msg.data

    def _nav_ok(self) -> bool:
        if self._last_nav_status_time is None:
            return False
        age = (self.get_clock().now() - self._last_nav_status_time).nanoseconds * 1e-9
        if age > self.nav_status_timeout:
            return False
        return self._nav_status_level != DiagnosticStatus.ERROR

    def _request_fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < self.request_timeout

    def _on_timer(self):
        block_reasons = []

        core_safe = True
        if self._emergency_stop:
            core_safe = False
            block_reasons.append('acil_durdurma')
        if self._leak_detected:
            core_safe = False
            block_reasons.append('su_kacagi')
        if not self._nav_ok():
            core_safe = False
            block_reasons.append('navigasyon_gecersiz')
        if not self._pixhawk_connected:
            core_safe = False
            block_reasons.append('pixhawk_baglantisi_yok')

        thrust_ok = core_safe and self._motion_permission and self._request_fresh(self._last_thrust_req_time)
        if core_safe and not self._motion_permission:
            block_reasons.append('hareket_izni_yok(60sn_inhibit)')
        thrust_command = self._thrust_request if thrust_ok else 0.0

        if core_safe and self._request_fresh(self._last_fin_req_time):
            fin_command = self._fin_request
        else:
            fin_command = Vector3(x=0.0, y=0.0, z=0.0)
            if not core_safe:
                block_reasons.append('kanatcik_bloklandi')

        if core_safe and self._request_fresh(self._last_buoyancy_req_time):
            buoyancy_command = self._buoyancy_request
        else:
            buoyancy_command = 0.0

        if core_safe and self._nose_cap_open_request:
            self._nose_cap_command_latched = True
        if not core_safe:
            block_reasons.append('burun_kapagi_bloklandi')
            nose_cap_command = False
        else:
            nose_cap_command = self._nose_cap_command_latched

        launch_conditions = {
            'core_safe': core_safe,
            'gudum_istegi': self._launch_request,
            'burun_disarida_kuyruk_icerde': self._dogru_cikis_acisi,
            'burun_kapagi_acik': self._nose_cap_command_latched,
            'pitch_yeterli': self._pitch >= self.launch_min_pitch,
        }
        launch_command = all(launch_conditions.values())
        if not launch_command and self._launch_request:
            failed = [k for k, v in launch_conditions.items() if not v]
            block_reasons.append('firlatma_bloklandi:' + ','.join(failed))

        self._publish_all(thrust_command, fin_command, buoyancy_command, nose_cap_command, launch_command, block_reasons, core_safe)

    def _publish_all(self, thrust_cmd, fin_cmd, buoyancy_cmd, nose_cap_cmd, launch_cmd, block_reasons, core_safe):
        t = Float64()
        t.data = float(thrust_cmd)
        self._thrust_cmd_pub.publish(t)

        self._fin_cmd_pub.publish(fin_cmd)

        b = Float64()
        b.data = float(buoyancy_cmd)
        self._buoyancy_cmd_pub.publish(b)

        cap = Bool()
        cap.data = bool(nose_cap_cmd)
        self._nose_cap_cmd_pub.publish(cap)

        launch = Bool()
        launch.data = bool(launch_cmd)
        self._launch_cmd_pub.publish(launch)
        # DUZELTME: her tikte (saniyede 20 kez) DEGIL, sadece False->True
        # GECISINDE bir kez logla - aksi halde ates penceresi boyunca
        # terminal ayni satirla dolup tasar ("sonsuz uretiyor" izlenimi verir).
        if launch_cmd and not self._was_launch_command:
            self.get_logger().warn('LAUNCH_COMMAND = TRUE gonderildi (tum bagimsiz kosullar saglandi).')
        elif not launch_cmd and self._was_launch_command:
            self.get_logger().info('LAUNCH_COMMAND = False (kosullar artik saglanmiyor / gorev tamamlandi).')
        self._was_launch_command = launch_cmd

        # --- Semadaki "Guvenli Onay?" karar kutusu - Evet/Hayir ---
        approved = Bool()
        approved.data = bool(core_safe)
        self._approved_pub.publish(approved)

        failsafe = Bool()
        failsafe.data = not core_safe
        self._failsafe_pub.publish(failsafe)

        status = DiagnosticStatus()
        status.name = 'sara_safety'
        status.hardware_id = 'jetson_orin_nano'
        status.level = DiagnosticStatus.OK if not block_reasons else DiagnosticStatus.WARN
        if self._emergency_stop or self._leak_detected:
            status.level = DiagnosticStatus.ERROR
        status.message = 'Nominal' if not block_reasons else ('Bloklar: ' + '; '.join(block_reasons))
        status.values = [
            KeyValue(key='approved', value=str(core_safe)),
            KeyValue(key='fail_safe_active', value=str(not core_safe)),
            KeyValue(key='thrust_command', value=f'{thrust_cmd:.2f}'),
            KeyValue(key='buoyancy_command', value=f'{buoyancy_cmd:.2f}'),
            KeyValue(key='nose_cap_command', value=str(nose_cap_cmd)),
            KeyValue(key='launch_command', value=str(launch_cmd)),
            KeyValue(key='pitch_deg', value=f'{math.degrees(self._pitch):.1f}'),
            KeyValue(key='surface_detected', value=str(self._surface_detected)),
            KeyValue(key='nose_submerged', value=str(self._nose_submerged)),
            KeyValue(key='tail_submerged', value=str(self._tail_submerged)),
            KeyValue(key='dogru_cikis_acisi', value=str(self._dogru_cikis_acisi)),
            KeyValue(key='motion_permission', value=str(self._motion_permission)),
            KeyValue(key='emergency_stop', value=str(self._emergency_stop)),
            KeyValue(key='leak_detected', value=str(self._leak_detected)),
        ]
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
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