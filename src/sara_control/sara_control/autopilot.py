#!/usr/bin/env python3
"""
autopilot.py
=============
SARA platformu - Otopilot / PID Katmani (Ontasarim Raporu Tablo 14-15)

Kontrol mimarisi: KAPALI CEVRIM PID, eksen basina BAGIMSIZ dongu
(derinlik, heading/yaw, pitch) - rapor - Otopilot Algoritmalari bolumu.

    e(t) = r(t) - y(t)
    u(t) = Kp*e(t) + Ki*Integral(e(t)dt) + Kd*de(t)/dt

Kaskad yapi (rapor: "kisa sureli derinlik/yonelim duzeltmeleri kanatciklar
uzerinden, uzun sureli derinlik koruma ve askida kalma davranisi step motor
kontrollu siringa tabanli degisken sephiye sistemiyle desteklenmektedir"):

    Derinlik hatasi -> [Derinlik-Trim PID] -> pitch trim (kisa sureli, hizli)
    (gudumden gelen hedef_pitch) + pitch_trim -> efektif pitch hedefi
    efektif pitch hedefi -> [Pitch PID] -> kanatcik (elevator) komutu
    heading hatasi (wrap) -> [Heading PID] -> kanatcik (rudder) komutu
    Derinlik hatasi -> [Sephiye PID, dusuk kazanc] -> step motor/siringa komutu (uzun sureli)

Turev terimi olcum TURETME (measurement) uzerinden hesaplanir - navigasyon
katmanindan gelen GERCEK acisal hizlar (pitch_rate, yaw_rate) kullanilir,
konum farkindan turetilmez (gurultu/wrap sorunlarindan kacinmak icin).
Bu ayni zamanda raporun istedigi "filtrelenmis turev yaklasimi" sartini
karsilar (navigasyon katmani zaten bu hizlari hareketli ortalama ile
filtrelemistir).

Satürasyon + Anti-windup: her PID cikisi sinirlandirilir; cikis
sinira dayaninca integral terimi DONDURULUR (basit clamping anti-windup).

ONEMLI: Bu node sadece KOMUT ISTEGI (*_request) uretir. Nihai Pixhawk'a
giden komutlar (*_command), ayri bir Guvenlik Katmani node'unda
dogrulanip/sinirlanip uretilmelidir (rapor: "...guvenlik katmani
denetiminden gecirilerek Pixhawk 6X uzerinden fiziksel ciktilara
donusturulmektedir"). Guvenlik dogrulamasi olmadan Pixhawk'a DOGRUDAN
BAGLANMAMALIDIR.

Girdi:
    /sara/guidance/target_pose             (geometry_msgs/PoseStamped)
    /sara/guidance/forward_motion_request  (std_msgs/Bool)
    /sara/guidance/mission_phase           (std_msgs/String, sadece telemetri)
    /sara/navigation/odom                  (nav_msgs/Odometry)
    /sara/navigation/status                (diagnostic_msgs/DiagnosticStatus)

Cikti:
    /sara/control/thrust_request     (std_msgs/Float64)   -- ESC/itki, [0,1]
    /sara/control/fin_request        (geometry_msgs/Vector3) -- x=pitch(elevator), y=yaw(rudder), [-1,1]
    /sara/control/buoyancy_request   (std_msgs/Float64)   -- step motor/siringa, [-1,1]
    /sara/control/status             (diagnostic_msgs/DiagnosticStatus)
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw_pitch_roll(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw, pitch, roll


class PIDController:
    """
    Klasik PID, olcum-turevli D terimi + clamping anti-windup + saturasyon.

    update() cagrisinda 'measured_rate', hatanin degil OLCULEN buyuklugun
    (turetilen gercek sensor buyuklugu, orn. pitch_rate) ani hizidir; D
    terimi bu hizin negatifi uzerinden hesaplanir (turev-uzerinde-olcum
    yontemi - setpoint sicramalarinda "derivative kick" olusturmaz).
    """

    def __init__(self, kp: float, ki: float, kd: float, out_min: float, out_max: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0.0
        self.last_saturated = False

    def reset(self):
        self._integral = 0.0
        self.last_saturated = False

    def update(self, error: float, measured_rate: float, dt: float) -> float:
        if dt <= 0.0:
            dt = 0.0

        p_term = self.kp * error
        tentative_integral = self._integral + error * dt
        i_term = self.ki * tentative_integral
        d_term = -self.kd * measured_rate

        unclamped = p_term + i_term + d_term
        output = max(self.out_min, min(self.out_max, unclamped))

        # Anti-windup (clamping yontemi): cikis satüre olmadiysa integrali guncelle,
        # satüre oldugunda integrali DONDUR (bir onceki degerde tut).
        if output == unclamped:
            self._integral = tentative_integral
            self.last_saturated = False
        else:
            self.last_saturated = True

        return output


class LowPassFilter:
    def __init__(self, alpha: float):
        self.alpha = alpha
        self._y = None

    def update(self, x: float) -> float:
        self._y = x if self._y is None else self.alpha * x + (1.0 - self.alpha) * self._y
        return self._y


class AutopilotNode(Node):

    def __init__(self):
        super().__init__('autopilot_node')

        # ================= Parametreler =================
        # NOT: Baslangic degerleri yer tutucudur. Rapor geregi: "aracin tahmini
        # kutlesi, gorev derinligi, beklenen hiz araligi, kanatcik etkisi,
        # sephiye sisteminin tepki suresi ve eyleyici sinirlari dikkate
        # alinarak" belirlenip lab/su ici testlerle iyilestirilecektir.

        # --- Derinlik-Trim dongusu (kisa sureli, kanatciga pitch trim uretir) ---
        self.declare_parameter('depth_trim_kp', 0.4)
        self.declare_parameter('depth_trim_ki', 0.02)
        self.declare_parameter('depth_trim_kd', 0.1)
        self.declare_parameter('depth_trim_limit_rad', 0.35)   # ~20 derece azami pitch trimi

        # --- Pitch dongusu (kanatcik/elevator) ---
        self.declare_parameter('pitch_kp', 1.2)
        self.declare_parameter('pitch_ki', 0.05)
        self.declare_parameter('pitch_kd', 0.3)
        self.declare_parameter('fin_pitch_limit', 1.0)          # normalize [-1,1]

        # --- Heading/Yaw dongusu (kanatcik/rudder) ---
        self.declare_parameter('heading_kp', 1.0)
        self.declare_parameter('heading_ki', 0.03)
        self.declare_parameter('heading_kd', 0.25)
        self.declare_parameter('fin_yaw_limit', 1.0)             # normalize [-1,1]

        # --- Sephiye dongusu (step motor/siringa, uzun sureli derinlik koruma) ---
        self.declare_parameter('buoyancy_kp', 0.15)
        self.declare_parameter('buoyancy_ki', 0.01)
        self.declare_parameter('buoyancy_limit', 1.0)             # normalize [-1,1]

        # --- Itki ---
        self.declare_parameter('nominal_thrust', 0.6)              # forward_motion_request=True iken sabit itki [0,1]

        # --- Genel ---
        self.declare_parameter('depth_rate_lpf_alpha', 0.3)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('nav_status_timeout_s', 2.0)

        depth_trim_limit = self.get_parameter('depth_trim_limit_rad').value
        fin_pitch_limit = self.get_parameter('fin_pitch_limit').value
        fin_yaw_limit = self.get_parameter('fin_yaw_limit').value
        buoyancy_limit = self.get_parameter('buoyancy_limit').value

        self.nominal_thrust = self.get_parameter('nominal_thrust').value
        self.nav_status_timeout = self.get_parameter('nav_status_timeout_s').value
        rate = float(self.get_parameter('control_rate_hz').value)

        self._depth_rate_lpf = LowPassFilter(self.get_parameter('depth_rate_lpf_alpha').value)

        # ================= PID nesneleri =================
        self.pid_depth_trim = PIDController(
            self.get_parameter('depth_trim_kp').value,
            self.get_parameter('depth_trim_ki').value,
            self.get_parameter('depth_trim_kd').value,
            -depth_trim_limit, depth_trim_limit,
        )
        self.pid_pitch = PIDController(
            self.get_parameter('pitch_kp').value,
            self.get_parameter('pitch_ki').value,
            self.get_parameter('pitch_kd').value,
            -fin_pitch_limit, fin_pitch_limit,
        )
        self.pid_heading = PIDController(
            self.get_parameter('heading_kp').value,
            self.get_parameter('heading_ki').value,
            self.get_parameter('heading_kd').value,
            -fin_yaw_limit, fin_yaw_limit,
        )
        self.pid_buoyancy = PIDController(
            self.get_parameter('buoyancy_kp').value,
            self.get_parameter('buoyancy_ki').value,
            0.0,
            -buoyancy_limit, buoyancy_limit,
        )

        # ================= Ic durum =================
        # Gudumden gelen hedefler
        self._target_depth = 0.0
        self._target_heading = 0.0
        self._target_pitch = 0.0
        self._forward_motion_request = False
        self._mission_phase = 'BILINMIYOR'

        # Navigasyondan gelen anlik degerler
        self._depth = 0.0
        self._prev_depth = None
        self._heading = 0.0
        self._pitch = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._yaw_rate = 0.0

        self._pixhawk_connected = False
        self._depth_valid = False
        self._last_nav_status_time = None
        self._nav_status_level = DiagnosticStatus.STALE
        self._was_safe = False

        self._last_tick_time = self.get_clock().now()

        # ================= Abonelikler =================
        self.create_subscription(PoseStamped, '/sara/guidance/target_pose', self._on_target_pose, 10)
        self.create_subscription(Bool, '/sara/guidance/forward_motion_request', self._on_forward_motion, 10)
        self.create_subscription(String, '/sara/guidance/mission_phase', self._on_mission_phase, 10)
        self.create_subscription(Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(DiagnosticStatus, '/sara/navigation/status', self._on_nav_status, 10)

        # ================= Yayinlar =================
        self._thrust_pub = self.create_publisher(Float64, '/sara/control/thrust_request', 10)
        self._fin_pub = self.create_publisher(Vector3, '/sara/control/fin_request', 10)
        self._buoyancy_pub = self.create_publisher(Float64, '/sara/control/buoyancy_request', 10)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/control/status', 10)

        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            'autopilot_node baslatildi. Ciktilar SADECE ISTEKTIR '
            '(*_request) - nihai Pixhawk komutlari Guvenlik Katmani '
            'tarafindan uretilmelidir.'
        )

    # ======================================================================
    # Abonelik callback'leri
    # ======================================================================
    def _on_target_pose(self, msg: PoseStamped):
        self._target_depth = msg.pose.position.z
        q = msg.pose.orientation
        yaw, pitch, _roll = quaternion_to_yaw_pitch_roll(q.x, q.y, q.z, q.w)
        self._target_heading = yaw
        self._target_pitch = pitch

    def _on_forward_motion(self, msg: Bool):
        self._forward_motion_request = msg.data

    def _on_mission_phase(self, msg: String):
        self._mission_phase = msg.data

    def _on_odom(self, msg: Odometry):
        self._depth = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        yaw, pitch, _roll = quaternion_to_yaw_pitch_roll(q.x, q.y, q.z, q.w)
        self._heading = yaw
        self._pitch = pitch
        self._roll_rate = msg.twist.twist.angular.x
        self._pitch_rate = msg.twist.twist.angular.y
        self._yaw_rate = msg.twist.twist.angular.z

    def _on_nav_status(self, msg: DiagnosticStatus):
        self._last_nav_status_time = self.get_clock().now()
        self._nav_status_level = msg.level
        values = {kv.key: kv.value for kv in msg.values}
        self._pixhawk_connected = values.get('pixhawk_connected', 'False') == 'True'
        self._depth_valid = values.get('depth_valid', 'False') == 'True'

    # ======================================================================
    # Yardimcilar
    # ======================================================================
    def _nav_ok(self) -> bool:
        if self._last_nav_status_time is None:
            return False
        age = (self.get_clock().now() - self._last_nav_status_time).nanoseconds * 1e-9
        if age > self.nav_status_timeout:
            return False
        return self._nav_status_level != DiagnosticStatus.ERROR

    def _reset_all_pids(self):
        self.pid_depth_trim.reset()
        self.pid_pitch.reset()
        self.pid_heading.reset()
        self.pid_buoyancy.reset()

    # ======================================================================
    # Ana kontrol dongusu
    # ======================================================================
    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick_time).nanoseconds * 1e-9
        self._last_tick_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.0

        # --- Derinlik hizi kestirimi (turev terimi icin, filtrelenmis) ---
        if self._prev_depth is not None and dt > 0.0:
            raw_depth_rate = (self._depth - self._prev_depth) / dt
        else:
            raw_depth_rate = 0.0
        depth_rate = self._depth_rate_lpf.update(raw_depth_rate)
        self._prev_depth = self._depth

        safe = self._nav_ok() and self._pixhawk_connected and self._depth_valid

        if not safe:
            # Yerel on-guvenlik: navigasyon/Pixhawk saglikli degilse tum
            # komut isteklerini sifirla ve PID integrallerini dondur
            # (baglanti kesikken windup birikmesin). NOT: bu, TAM Guvenlik
            # Katmaninin yerini TUTMAZ - o ayrica, tum komutlari
            # dogrulayacak sekilde inSA edilecektir.
            if self._was_safe:
                self.get_logger().warn('Navigasyon/Pixhawk saglikli degil - kontrol ciktilari sifirlaniyor.')
            self._reset_all_pids()
            thrust = 0.0
            fin_pitch = 0.0
            fin_yaw = 0.0
            buoyancy = 0.0
        else:
            depth_error = self._target_depth - self._depth

            # --- Kaskad: Derinlik hatasi -> pitch trim (kisa sureli) ---
            pitch_trim = self.pid_depth_trim.update(depth_error, depth_rate, dt)
            effective_pitch_target = self._target_pitch + pitch_trim

            # --- Pitch PID -> kanatcik (elevator) ---
            pitch_error = effective_pitch_target - self._pitch
            fin_pitch = self.pid_pitch.update(pitch_error, self._pitch_rate, dt)

            # --- Heading PID -> kanatcik (rudder), wrap edilmis hata ---
            heading_error = wrap_pi(self._target_heading - self._heading)
            fin_yaw = self.pid_heading.update(heading_error, self._yaw_rate, dt)

            # --- Sephiye PID -> step motor/siringa (uzun sureli derinlik koruma) ---
            buoyancy = self.pid_buoyancy.update(depth_error, depth_rate, dt)

            # --- Itki: gorev fazina gore (gudumun forward_motion_request'i) ---
            thrust = self.nominal_thrust if self._forward_motion_request else 0.0

        self._was_safe = safe
        self._publish_all(safe, thrust, fin_pitch, fin_yaw, buoyancy)

    # ======================================================================
    # Yayin
    # ======================================================================
    def _publish_all(self, safe, thrust, fin_pitch, fin_yaw, buoyancy):
        t = Float64()
        t.data = float(thrust)
        self._thrust_pub.publish(t)

        fin = Vector3()
        fin.x = float(fin_pitch)
        fin.y = float(fin_yaw)
        fin.z = 0.0
        self._fin_pub.publish(fin)

        b = Float64()
        b.data = float(buoyancy)
        self._buoyancy_pub.publish(b)

        status = DiagnosticStatus()
        status.name = 'sara_autopilot'
        status.hardware_id = 'jetson_orin_nano'
        status.level = DiagnosticStatus.OK if safe else DiagnosticStatus.ERROR
        status.message = 'Nominal' if safe else 'Navigasyon/Pixhawk saglik kontrolu basarisiz - ciktilar sifir'
        status.values = [
            KeyValue(key='mission_phase', value=self._mission_phase),
            KeyValue(key='depth_error_m', value=f'{self._target_depth - self._depth:.3f}'),
            KeyValue(key='heading_error_rad', value=f'{wrap_pi(self._target_heading - self._heading):.3f}'),
            KeyValue(key='pitch_error_rad', value=f'{self._target_pitch - self._pitch:.3f}'),
            KeyValue(key='thrust_request', value=f'{thrust:.2f}'),
            KeyValue(key='fin_pitch_request', value=f'{fin_pitch:.2f}'),
            KeyValue(key='fin_yaw_request', value=f'{fin_yaw:.2f}'),
            KeyValue(key='buoyancy_request', value=f'{buoyancy:.2f}'),
            KeyValue(key='pitch_pid_saturated', value=str(self.pid_pitch.last_saturated)),
            KeyValue(key='heading_pid_saturated', value=str(self.pid_heading.last_saturated)),
            KeyValue(key='buoyancy_pid_saturated', value=str(self.pid_buoyancy.last_saturated)),
        ]
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
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