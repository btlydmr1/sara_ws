#!/usr/bin/env python3
"""
guidance.py (v2)
==================
SARA platformu - Gudum Katmani

Iki AYRI gorev profili destekler (parametre ile secilir: mission_id):

  GOREV 1 - Seyir Gorevi (mission_id=1):
    1. Baslangictan itibaren 2 m derinlige in.
    2. Ilk 10 m duz ilerle.
    3. (10 m tamamlaninca) yarisma suresi baslar - ic telemetri isaretcisi.
    4. Kiyidan en az 50 m uzaklas.
    5. Baslangic/bitis cizgisine geri don.
    6. Enerjiyi kesip su ustunde bekle.

  GOREV 2 - Atis Gorevi (mission_id=2):
    1. 30 m duz git.
    2. Guvenli atis bolgesine ulas.
    3. +30 derece yunuslama aciSiyla yuzeye cik.
    4. Dogru aciyi algila/onayla.
    5. Burun kapagini ac.
    6. Roket atesleme sinyalini gonder (TAMAMEN OTOMATIK, manuel degil).

GUVENLIK KURALLARI (rapor + kullanicidan gelen sartlar):
  - Acil durdurma butonuna basilirsa motorlar HEMEN durur (ayni kontrol
    dongusunde, forward_motion_request=False).
  - Acma butonundan (gorev baslangicindan) sonra 60 sn gecmeden motorlar
    CALISMAZ (AKUSTIK_UYARI_GOREV_BASLATMA fazi).
  - Atesleme MANUEL DEGIL: launch_request sadece otomatik kosul
    degerlendirmesiyle uretilir; hicbir "manuel tetikle" girdisi YOKTUR.
  - Birden fazla bagimsiz fail-safe kontrolu: acil durdurma, navigasyon
    veri gecerliligi/zaman asimi, azami gorev suresi, su kacagi (leak).

Gudum katmani DOGRUDAN motor/servo sinyali URETMEZ; sadece hedef
derinlik/heading/pitch ve gorev fazi kararlarini uretir (Otopilot/PID
katmani bunlari tuketir). Burun kapagi acma ve ates sinyali de birer
"ISTEK/komut talebidir" - launch_request ozelinde nihai firlatma
onayi ayri bir Guvenlik Katmani dugumunde dogrulanmalidir.

Girdi:
    /sara/navigation/odom               (nav_msgs/Odometry)
    /sara/navigation/surface_detected   (std_msgs/Bool)
    /sara/navigation/status             (diagnostic_msgs/DiagnosticStatus)
    /sara/safety/emergency_stop         (std_msgs/Bool, yoksa False varsayilir)
    /sara/safety/leak_detected          (std_msgs/Bool, yoksa False varsayilir)

Cikti:
    /sara/guidance/target_pose            (geometry_msgs/PoseStamped)
    /sara/guidance/mission_phase          (std_msgs/String)
    /sara/guidance/forward_motion_request (std_msgs/Bool)
    /sara/guidance/nose_cap_open_request  (std_msgs/Bool)
    /sara/guidance/launch_request         (std_msgs/Bool)  -- SADECE ISTEK, OTOMATIK
    /sara/guidance/status                 (diagnostic_msgs/DiagnosticStatus)

NOT: Akustik uyari/buzzer sinyali BU node'da degil, ayri mission_start.py
node'unda uretilir (/sara/mission_start/acoustic_warning) - Tablo 11'de
Gudum'den bagimsiz bir katman olarak tanimlandigi icin ayrildi.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


# ---------------------------------------------------------------------------
# Gorev fazlari
# ---------------------------------------------------------------------------
PHASE_REFERANSLAMA = 0
PHASE_AKUSTIK_UYARI = 1
PHASE_DALIS = 2

# Gorev 1 - Seyir
PHASE_G1_DUZ_SEYIR_10M = 10
PHASE_G1_UZAKLASMA_50M = 11
PHASE_G1_GERI_DONUS = 12
PHASE_G1_TAMAMLANDI_YUZEYDE_BEKLE = 13

# Gorev 2 - Atis
PHASE_G2_DUZ_SEYIR_30M = 20
PHASE_G2_GUVENLI_ATIS_BOLGESI = 21
PHASE_G2_TIRMANIS_30_DERECE = 22
PHASE_G2_ACI_DOGRULAMA = 23
PHASE_G2_BURUN_KAPAGI_AC = 24
PHASE_G2_ATES_SINYALI_GONDER = 25
PHASE_G2_TAMAMLANDI = 26

# Ortak fail-safe / terminal
PHASE_GUVENLI_SONLANDIRMA = 99

PHASE_NAMES = {
    PHASE_REFERANSLAMA: 'REFERANSLAMA',
    PHASE_AKUSTIK_UYARI: 'AKUSTIK_UYARI_GOREV_BASLATMA',
    PHASE_DALIS: 'DALIS',
    PHASE_G1_DUZ_SEYIR_10M: 'G1_DUZ_SEYIR_ILK_10M',
    PHASE_G1_UZAKLASMA_50M: 'G1_UZAKLASMA_KIYIDAN_50M',
    PHASE_G1_GERI_DONUS: 'G1_GERI_DONUS_BASLANGIC_CIZGISI',
    PHASE_G1_TAMAMLANDI_YUZEYDE_BEKLE: 'G1_TAMAMLANDI_ENERJI_KESIK_YUZEYDE_BEKLE',
    PHASE_G2_DUZ_SEYIR_30M: 'G2_DUZ_SEYIR_30M',
    PHASE_G2_GUVENLI_ATIS_BOLGESI: 'G2_GUVENLI_ATIS_BOLGESI',
    PHASE_G2_TIRMANIS_30_DERECE: 'G2_TIRMANIS_30_DERECE',
    PHASE_G2_ACI_DOGRULAMA: 'G2_ACI_DOGRULAMA',
    PHASE_G2_BURUN_KAPAGI_AC: 'G2_BURUN_KAPAGI_AC',
    PHASE_G2_ATES_SINYALI_GONDER: 'G2_ATES_SINYALI_GONDER',
    PHASE_G2_TAMAMLANDI: 'G2_TAMAMLANDI',
    PHASE_GUVENLI_SONLANDIRMA: 'GUVENLI_SONLANDIRMA',
}

TERMINAL_PHASES = {PHASE_G1_TAMAMLANDI_YUZEYDE_BEKLE, PHASE_G2_TAMAMLANDI, PHASE_GUVENLI_SONLANDIRMA}


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


def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class GuidanceNode(Node):

    def __init__(self):
        super().__init__('guidance_node')

        # ================= Parametreler =================
        # Hangi gorev kosulacak: 1=Seyir Gorevi, 2=Atis Gorevi
        self.declare_parameter('mission_id', 1)

        # --- Ortak ---
        self.declare_parameter('dive_target_depth', 2.0)          # [m] - Gorev1 madde1 (Gorev2 icin de varsayilan)
        self.declare_parameter('depth_tolerance', 0.15)             # [m]
        self.declare_parameter('heading_tolerance', 0.10)           # [rad] (~5.7 derece)
        self.declare_parameter('pitch_tolerance', 0.10)             # [rad]
        # NOT: motor_inhibit_duration_s / acoustic_warning_lead_s artik BU node'da
        # DEGIL - ayri "mission_start_node" (Gorev Baslatma/Akustik Uyari Katmani,
        # Tablo 11) tarafindan yonetiliyor. Bu node onun uretttigi
        # /sara/mission_start/motion_permission ciktisini dinler.
        self.declare_parameter('kararlilik_suresi_s', 3.0)           # kosul kesintisiz saglanma suresi
        self.declare_parameter('max_mission_duration_s', 600.0)      # fail-safe: azami gorev suresi
        self.declare_parameter('nav_status_timeout_s', 2.0)           # fail-safe: navigasyon veri zaman asimi
        self.declare_parameter('startup_grace_period_s', 5.0)          # DUZELTME: baslangicta diger node'lar
                                                                          # henuz yayina baslamadan "nav gecersiz"
                                                                          # fail-safe'i yanlislikla tetiklenmesin
        self.declare_parameter('control_rate_hz', 10.0)

        # --- Gorev 1 (Seyir) parametreleri ---
        self.declare_parameter('g1_duz_seyir_distance_m', 10.0)         # madde 2: ilk 10 m duz
        self.declare_parameter('g1_uzaklasma_min_distance_m', 50.0)      # madde 4: kiyidan en az 50 m
        self.declare_parameter('g1_geri_donus_tolerance_m', 2.0)          # madde 5: baslangic/bitis cizgisine yakinlik

        # --- Gorev 2 (Atis) parametreleri ---
        self.declare_parameter('g2_duz_seyir_distance_m', 30.0)          # madde 1: 30 m duz git
        # Guvenli atis bolgesi: SAHA/YARISMA KURALINA GORE TANIMLANMALI (TODO - gercek
        # deger geldiginde guncellenecek). Simdilik TEST AMACLI, 30m duz seyir
        # mesafesiyle TUTARLI bir aralik verildi (mesafe 30m'ye ulasinca bolgeye
        # girilebilsin diye) - onceki varsayilan (0-5m) 30m'lik seyirle CELISIYORDU.
        self.declare_parameter('g2_safe_zone_min_m', 25.0)                 # TODO: gercek deger
        self.declare_parameter('g2_safe_zone_max_m', 35.0)                   # TODO: gercek deger
        self.declare_parameter('g2_tirmanis_target_pitch_deg', 30.0)         # madde 3: +30 derece
        self.declare_parameter('g2_firing_depth', 0.0)                        # DUZELTME: gorev tanimi acikca
                                                                                  # "yuzeye cik" diyor - ara bir
                                                                                  # derinlik DEGIL, gercek yuzey (0m).
                                                                                  # Derinlik fiziksel olarak 0'in
                                                                                  # ALTINA inemedigi icin eski 0.3m
                                                                                  # hedefi ASLA yakalanamiyordu.
        self.declare_parameter('g2_nose_cap_open_duration_s', 3.0)            # TODO: gercek servo suresi

        # ================= Iceri aktar =================
        self.mission_id = int(self.get_parameter('mission_id').value)

        self.dive_target_depth = self.get_parameter('dive_target_depth').value
        self.depth_tol = self.get_parameter('depth_tolerance').value
        self.heading_tol = self.get_parameter('heading_tolerance').value
        self.pitch_tol = self.get_parameter('pitch_tolerance').value
        self.kararlilik_suresi = self.get_parameter('kararlilik_suresi_s').value
        self.max_mission_duration = self.get_parameter('max_mission_duration_s').value
        self.nav_status_timeout = self.get_parameter('nav_status_timeout_s').value
        self.startup_grace_period = self.get_parameter('startup_grace_period_s').value

        self.g1_duz_seyir_distance = self.get_parameter('g1_duz_seyir_distance_m').value
        self.g1_uzaklasma_min_distance = self.get_parameter('g1_uzaklasma_min_distance_m').value
        self.g1_geri_donus_tolerance = self.get_parameter('g1_geri_donus_tolerance_m').value

        self.g2_duz_seyir_distance = self.get_parameter('g2_duz_seyir_distance_m').value
        self.g2_safe_zone_min = self.get_parameter('g2_safe_zone_min_m').value
        self.g2_safe_zone_max = self.get_parameter('g2_safe_zone_max_m').value
        self.g2_tirmanis_target_pitch = math.radians(self.get_parameter('g2_tirmanis_target_pitch_deg').value)
        self.g2_firing_depth = self.get_parameter('g2_firing_depth').value
        self.g2_nose_cap_open_duration = self.get_parameter('g2_nose_cap_open_duration_s').value

        rate = float(self.get_parameter('control_rate_hz').value)

        if self.mission_id not in (1, 2):
            self.get_logger().warn(f'Gecersiz mission_id={self.mission_id}, 1 (Seyir Gorevi) varsayilacak.')
            self.mission_id = 1

        # ================= Ic durum =================
        self._phase = PHASE_REFERANSLAMA
        self._mission_start_time = self.get_clock().now()
        self._phase_start_time = self.get_clock().now()
        self._condition_hold_start = None
        self._race_timer_start = None   # Gorev1 madde3: "yarisma suresi baslasin" - ic telemetri isaretcisi

        self._reference_heading = 0.0
        self._reference_depth = 0.0

        self._depth = 0.0
        self._heading = 0.0
        self._pitch = 0.0
        self._approx_x = 0.0
        self._approx_y = 0.0
        self._motion_consistent = False
        self._pixhawk_connected = False
        self._depth_valid = False
        self._surface_detected = False
        self._nose_submerged = True   # burun su icinde mi (True=icinde/su var, False=disarida)
        self._tail_submerged = True   # kuyruk su icinde mi
        self._last_nav_status_time = None
        self._nav_status_level = DiagnosticStatus.STALE

        self._emergency_stop = False
        self._leak_detected = False
        self._motion_permission = False   # mission_start_node'dan gelir (Tablo 11 - Gorev Baslatma/Akustik Uyari)

        self._target_depth = 0.0
        self._target_heading = 0.0
        self._target_pitch = 0.0
        self._forward_motion = False
        self._nose_cap_open_request = False
        self._launch_request = False

        # ================= Abonelikler =================
        self.create_subscription(Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/sara/navigation/surface_detected', self._on_surface, 10)
        # DUZELTME (KTR sayfa 14/37): Atesleme izni "yuzeyde mi degil mi" gibi
        # tek bir genel bayrakla degil, TAM OLARAK "burun su DISINDA, kuyruk
        # su ICINDE" (kismi cikis acisi) kosuluyla verilmelidir. Bu yuzden
        # ham sensorlere DOGRUDAN abone oluyoruz - navigasyonun tek bir
        # boolean'a indirgedigi surface_detected bu ayrimi ifade edemez.
        # DUZELTME: sensor topic'leri (vehicle_sim/gercek donanim) BEST_EFFORT
        # QoS ile yayin yapar. Varsayilan (RELIABLE) abonelik bunlarla
        # UYUMSUZDUR - hicbir mesaj alinamaz, sessizce basarisiz olur
        # (sadece bir WARN log basar, hata firlatmaz - bu yuzden fark
        # edilmesi kolay degildi).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Bool, '/sara/water_detect_1', self._on_water_nose, sensor_qos)  # burun sensoru
        self.create_subscription(Bool, '/sara/water_detect_2', self._on_water_tail, sensor_qos)  # kuyruk sensoru
        self.create_subscription(DiagnosticStatus, '/sara/navigation/status', self._on_nav_status, 10)
        self.create_subscription(Bool, '/sara/safety/emergency_stop', self._on_emergency_stop, 10)
        self.create_subscription(Bool, '/sara/safety/leak_detected', self._on_leak, 10)
        # Gorev Baslatma / Akustik Uyari Katmani (ayri node - mission_start.py, Tablo 11)
        self.create_subscription(Bool, '/sara/mission_start/motion_permission', self._on_motion_permission, 10)

        # ================= Yayinlar =================
        # NOT: acoustic_warning burada YAYINLANMAZ - bu sinyal artik mission_start_node'a
        # ait (/sara/mission_start/acoustic_warning). Ayri topic'ler, ayri sorumluluklar.
        self._target_pub = self.create_publisher(PoseStamped, '/sara/guidance/target_pose', 10)
        self._phase_pub = self.create_publisher(String, '/sara/guidance/mission_phase', 10)
        self._forward_pub = self.create_publisher(Bool, '/sara/guidance/forward_motion_request', 10)
        self._nose_cap_pub = self.create_publisher(Bool, '/sara/guidance/nose_cap_open_request', 10)
        self._launch_pub = self.create_publisher(Bool, '/sara/guidance/launch_request', 10)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/guidance/status', 10)

        self.create_timer(1.0 / rate, self._on_timer)

        mission_name = 'GOREV 1 (Seyir)' if self.mission_id == 1 else 'GOREV 2 (Atis)'
        self.get_logger().info(
            f'guidance_node baslatildi. Secili gorev: {mission_name}. '
            'Ates sinyali TAMAMEN OTOMATIK uretilir; manuel tetikleme girdisi yoktur.'
        )

    # ======================================================================
    # Abonelik callback'leri
    # ======================================================================
    def _on_odom(self, msg: Odometry):
        self._depth = msg.pose.pose.position.z
        self._approx_x = msg.pose.pose.position.x
        self._approx_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw, pitch, _roll = quaternion_to_yaw_pitch_roll(q.x, q.y, q.z, q.w)
        self._heading = yaw
        self._pitch = pitch
        self._motion_consistent = abs(msg.twist.twist.linear.x) > 1e-6

    def _on_surface(self, msg: Bool):
        self._surface_detected = msg.data

    def _on_water_nose(self, msg: Bool):
        self._nose_submerged = msg.data

    def _on_water_tail(self, msg: Bool):
        self._tail_submerged = msg.data

    @property
    def _dogru_cikis_acisi(self) -> bool:
        """KTR sayfa 14/37: atesleme icin gerekli TEK gecerli su durumu -
        burun su DISINDA (False) VE kuyruk su ICINDE (True). Tamamen
        batikken veya tamamen havadayken bu KOSUL SAGLANMAZ (bilerek)."""
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

    def _on_motion_permission(self, msg: Bool):
        self._motion_permission = msg.data

    # ======================================================================
    # Yardimci hesaplar
    # ======================================================================
    def _approx_distance(self) -> float:
        return math.hypot(self._approx_x, self._approx_y)

    def _heading_error_to(self, target_heading: float) -> float:
        return wrap_pi(target_heading - self._heading)

    def _nav_ok(self) -> bool:
        if self._last_nav_status_time is None:
            return False
        age = (self.get_clock().now() - self._last_nav_status_time).nanoseconds * 1e-9
        if age > self.nav_status_timeout:
            return False
        return self._nav_status_level != DiagnosticStatus.ERROR

    def _mission_elapsed(self) -> float:
        return (self.get_clock().now() - self._mission_start_time).nanoseconds * 1e-9

    def _phase_elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_start_time).nanoseconds * 1e-9

    def _race_elapsed(self):
        if self._race_timer_start is None:
            return None
        return (self.get_clock().now() - self._race_timer_start).nanoseconds * 1e-9

    def _goto_phase(self, phase: int):
        if phase != self._phase:
            self.get_logger().info(
                f'Gorev fazi gecisi: {PHASE_NAMES[self._phase]} -> {PHASE_NAMES[phase]}'
            )
            self._phase = phase
            self._phase_start_time = self.get_clock().now()
            self._condition_hold_start = None

    def _conditions_held(self, ok: bool) -> bool:
        now = self.get_clock().now()
        if not ok:
            self._condition_hold_start = None
            return False
        if self._condition_hold_start is None:
            self._condition_hold_start = now
        held = (now - self._condition_hold_start).nanoseconds * 1e-9
        return held >= self.kararlilik_suresi

    # ======================================================================
    # Ana kontrol dongusu - GUVENLIK (fail-safe, coklu ve bagimsiz) + FSM
    # ======================================================================
    def _on_timer(self):
        failsafe_reason = None
        if self._emergency_stop:
            failsafe_reason = 'acil durdurma butonuna basildi'
        elif self._leak_detected:
            failsafe_reason = 'su kacagi (leak) tespit edildi'
        elif self._mission_elapsed() > self.startup_grace_period and not self._nav_ok():
            # DUZELTME: startup_grace_period icinde nav verisi henuz gelmemis
            # olabilir (diger node'lar henuz yayina baslamamis) - bu durumu
            # fail-safe saymiyoruz, sadece GERCEK bir zaman asimini sayiyoruz.
            failsafe_reason = 'navigasyon verisi gecersiz/zaman asimi'
        elif self._mission_elapsed() > self.max_mission_duration:
            failsafe_reason = 'azami gorev suresi asildi'

        if failsafe_reason is not None and self._phase not in TERMINAL_PHASES:
            self.get_logger().error(f'GUVENLI SONLANDIRMA tetiklendi: {failsafe_reason}')
            self._goto_phase(PHASE_GUVENLI_SONLANDIRMA)

        if self._phase == PHASE_REFERANSLAMA:
            self._run_referanslama()
        elif self._phase == PHASE_AKUSTIK_UYARI:
            self._run_akustik_uyari()
        elif self._phase == PHASE_DALIS:
            self._run_dalis()
        elif self._phase == PHASE_G1_DUZ_SEYIR_10M:
            self._run_g1_duz_seyir()
        elif self._phase == PHASE_G1_UZAKLASMA_50M:
            self._run_g1_uzaklasma()
        elif self._phase == PHASE_G1_GERI_DONUS:
            self._run_g1_geri_donus()
        elif self._phase == PHASE_G1_TAMAMLANDI_YUZEYDE_BEKLE:
            self._run_g1_tamamlandi()
        elif self._phase == PHASE_G2_DUZ_SEYIR_30M:
            self._run_g2_duz_seyir()
        elif self._phase == PHASE_G2_GUVENLI_ATIS_BOLGESI:
            self._run_g2_guvenli_atis_bolgesi()
        elif self._phase == PHASE_G2_TIRMANIS_30_DERECE:
            self._run_g2_tirmanis()
        elif self._phase == PHASE_G2_ACI_DOGRULAMA:
            self._run_g2_aci_dogrulama()
        elif self._phase == PHASE_G2_BURUN_KAPAGI_AC:
            self._run_g2_burun_kapagi_ac()
        elif self._phase == PHASE_G2_ATES_SINYALI_GONDER:
            self._run_g2_ates_sinyali()
        elif self._phase == PHASE_G2_TAMAMLANDI:
            self._run_g2_tamamlandi()
        else:
            self._run_guvenli_sonlandirma()

        self._publish_all()

    # ---------------------------------------------------------- Ortak fazlar
    def _run_referanslama(self):
        self._forward_motion = False
        self._target_depth = 0.0
        self._target_heading = self._heading
        self._target_pitch = 0.0

        stable = self._nav_ok() and self._depth_valid and self._pixhawk_connected
        if self._conditions_held(stable):
            self._reference_heading = self._heading
            self._reference_depth = self._depth
            self.get_logger().info(
                f'Referanslar belirlendi: heading={math.degrees(self._reference_heading):.1f} deg, '
                f'derinlik={self._reference_depth:.2f} m'
            )
            self._goto_phase(PHASE_AKUSTIK_UYARI)

    def _run_akustik_uyari(self):
        """60 sn motor inhibit + son 10 sn pinger/buzzer artik BU node'da degil,
        ayri mission_start_node tarafindan yonetiliyor (Tablo 11). Bu faz sadece
        onun urettigi /sara/mission_start/motion_permission=True olmasini bekler."""
        self._forward_motion = False
        self._target_depth = 0.0
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0

        if self._motion_permission:
            self._goto_phase(PHASE_DALIS)

    def _run_dalis(self):
        self._forward_motion = False
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0

        depth_ok = abs(self._depth - self.dive_target_depth) < self.depth_tol
        if self._conditions_held(depth_ok):
            if self.mission_id == 1:
                self._goto_phase(PHASE_G1_DUZ_SEYIR_10M)
            else:
                self._goto_phase(PHASE_G2_DUZ_SEYIR_30M)

    # ---------------------------------------------------------- GOREV 1
    def _run_g1_duz_seyir(self):
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True

        if self._approx_distance() >= self.g1_duz_seyir_distance:
            self._race_timer_start = self.get_clock().now()
            self.get_logger().info('Ilk 10 m tamamlandi - yarisma suresi (ic telemetri) baslatildi.')
            self._goto_phase(PHASE_G1_UZAKLASMA_50M)

    def _run_g1_uzaklasma(self):
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True

        if self._approx_distance() >= self.g1_uzaklasma_min_distance:
            self._goto_phase(PHASE_G1_GERI_DONUS)

    def _run_g1_geri_donus(self):
        return_heading = wrap_pi(self._reference_heading + math.pi)
        self._target_depth = self.dive_target_depth
        self._target_heading = return_heading
        self._target_pitch = 0.0
        self._forward_motion = True

        distance_to_start_ok = self._approx_distance() <= self.g1_geri_donus_tolerance
        if self._conditions_held(distance_to_start_ok):
            self._goto_phase(PHASE_G1_TAMAMLANDI_YUZEYDE_BEKLE)

    def _run_g1_tamamlandi(self):
        self._forward_motion = False
        self._target_depth = 0.0
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._launch_request = False

    # ---------------------------------------------------------- GOREV 2
    def _run_g2_duz_seyir(self):
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True

        if self._approx_distance() >= self.g2_duz_seyir_distance:
            self._goto_phase(PHASE_G2_GUVENLI_ATIS_BOLGESI)

    def _run_g2_guvenli_atis_bolgesi(self):
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = False

        dist = self._approx_distance()
        in_zone = self.g2_safe_zone_min <= dist <= self.g2_safe_zone_max
        conditions_ok = in_zone and self._motion_consistent and not self._emergency_stop

        if self._conditions_held(conditions_ok):
            self._goto_phase(PHASE_G2_TIRMANIS_30_DERECE)

    def _run_g2_tirmanis(self):
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True

        pitch_ok = abs(self._pitch - self.g2_tirmanis_target_pitch) < self.pitch_tol
        # DUZELTME: derinlik fiziksel olarak 0'in altina inemez (yuzey siniri).
        # "Tam esitlik" yerine "yeterince sig/yuzeye ulasti mi" kontrolu dogru olan.
        depth_ok = self._depth <= (self.g2_firing_depth + self.depth_tol)
        if self._conditions_held(pitch_ok and depth_ok):
            self._goto_phase(PHASE_G2_ACI_DOGRULAMA)

    def _run_g2_aci_dogrulama(self):
        """Madde 4: dogru aciyi algila. KTR (sayfa 14/37): 'dogru aci' pitch
        degeriyle DEGIL, fiziksel olarak burun su disinda + kuyruk su icinde
        olmasiyla dogrulanir (kismi cikis - roket atesleme icin gereken tam
        pozisyon). Sadece pitch=30 derece olmasi YETERLI DEGILDIR.

        DUZELTME: itki TAMAMEN KESILMEZ (once False idi). Gercek bir arac,
        yuzeye yakin acili pozisyonunu itkisiz koruyamaz - sephiye tek basina
        yeterli/hizli olmayabilir, arac tekrar batabilir (nose_submerged bir
        daha hic False olamaz, sonsuza kadar bekler). Hafif itki ile pozisyon
        korunur."""
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True

        pitch_ok = abs(self._pitch - self.g2_tirmanis_target_pitch) < self.pitch_tol
        dogru_aci_ok = pitch_ok and self._dogru_cikis_acisi
        if self._conditions_held(dogru_aci_ok):
            self._goto_phase(PHASE_G2_BURUN_KAPAGI_AC)

    def _run_g2_burun_kapagi_ac(self):
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True  # DUZELTME: pozisyonu korumak icin hafif itki
        self._nose_cap_open_request = True

        if self._phase_elapsed() >= self.g2_nose_cap_open_duration:
            self._goto_phase(PHASE_G2_ATES_SINYALI_GONDER)

    def _run_g2_ates_sinyali(self):
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True  # DUZELTME: pozisyonu korumak icin hafif itki
        self._nose_cap_open_request = True

        pitch_ok = abs(self._pitch - self.g2_tirmanis_target_pitch) < self.pitch_tol
        nav_ok = self._nav_ok() and self._pixhawk_connected
        # DUZELTME (KTR sayfa 14/37): atesleme izni "yuzeyde mi" gibi genel
        # bir bayrakla DEGIL, "burun su disinda VE kuyruk su icinde" TAM
        # kosuluyla verilmelidir. Tamamen batikken veya tamamen havadayken
        # atesleme KESINLIKLE ENGELLENIR.
        conditions_ok = (
            pitch_ok and nav_ok
            and self._dogru_cikis_acisi
            and not self._emergency_stop
            and not self._leak_detected
        )

        self._launch_request = self._conditions_held(conditions_ok)

        if self._launch_request and self._phase_elapsed() > (self.kararlilik_suresi * 3.0):
            self._goto_phase(PHASE_G2_TAMAMLANDI)

    def _run_g2_tamamlandi(self):
        self._forward_motion = False
        self._launch_request = False
        self._nose_cap_open_request = True
        self._target_depth = 0.0

    # ---------------------------------------------------------- Fail-safe
    def _run_guvenli_sonlandirma(self):
        self._forward_motion = False
        self._launch_request = False
        self._nose_cap_open_request = False
        self._target_depth = 0.0
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0

    # ======================================================================
    # Yayin
    # ======================================================================
    def _publish_all(self):
        now = self.get_clock().now()

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'sara_odom'
        pose.pose.position.z = float(self._target_depth)
        qx, qy, qz, qw = euler_to_quaternion(0.0, self._target_pitch, self._target_heading)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._target_pub.publish(pose)

        phase_msg = String()
        phase_msg.data = PHASE_NAMES[self._phase]
        self._phase_pub.publish(phase_msg)

        fwd = Bool()
        fwd.data = bool(self._forward_motion)
        self._forward_pub.publish(fwd)

        cap = Bool()
        cap.data = bool(self._nose_cap_open_request)
        self._nose_cap_pub.publish(cap)

        launch = Bool()
        launch.data = bool(self._launch_request)
        self._launch_pub.publish(launch)

        status = DiagnosticStatus()
        status.name = 'sara_guidance'
        status.hardware_id = 'jetson_orin_nano'
        status.level = DiagnosticStatus.ERROR if self._phase == PHASE_GUVENLI_SONLANDIRMA else DiagnosticStatus.OK
        status.message = f'Gorev {self.mission_id} - Faz: {PHASE_NAMES[self._phase]}'

        race_elapsed = self._race_elapsed()
        status.values = [
            KeyValue(key='mission_id', value=str(self.mission_id)),
            KeyValue(key='approx_distance_m', value=f'{self._approx_distance():.2f}'),
            KeyValue(key='heading_error_rad', value=f'{self._heading_error_to(self._target_heading):.3f}'),
            KeyValue(key='pitch_deg', value=f'{math.degrees(self._pitch):.1f}'),
            KeyValue(key='launch_request', value=str(self._launch_request)),
            KeyValue(key='nose_cap_open_request', value=str(self._nose_cap_open_request)),
            KeyValue(key='mission_elapsed_s', value=f'{self._mission_elapsed():.1f}'),
            KeyValue(key='race_elapsed_s', value='-' if race_elapsed is None else f'{race_elapsed:.1f}'),
            KeyValue(key='emergency_stop', value=str(self._emergency_stop)),
            KeyValue(key='leak_detected', value=str(self._leak_detected)),
            KeyValue(key='motion_permission', value=str(self._motion_permission)),
        ]
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
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