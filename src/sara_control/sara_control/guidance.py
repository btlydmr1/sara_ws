#!/usr/bin/env python3
"""
guidance.py (v3 - GRANÜLER FAZ MİMARİSİ)
==========================================
SARA platformu - Gudum Katmani

DUZELTME (v3): Fazlar, KTR Tablo 1/Tablo 2 ve sartnameyle BIREBIR
eslesecek sekilde, her biri kendi hizi/gecis kosuluyla AYRI birer faz
olacak sekilde parcalanmistir (v2'de bircogu "dahili hiz gecisi" ile tek
fazda birlestirilmisti - telemetri/log okumasini zorlastiriyordu).

Iki AYRI gorev profili destekler (parametre ile secilir: mission_id):

  GOREV 1 - Seyir Gorevi (mission_id=1), KTR Tablo 1 ile birebir:
    REFERANSLAMA
      -> AKUSTIK_UYARI_GOREV_BASLATMA   (mission_start_node: 60 sn motor inhibit)
      -> DALIS                           (2 m derinlige in, sabit yerde)
      -> G1_0_10M_KALIBRASYON            (0.895 m/s, ilk 10 m - yarisma suresi
                                           bu fazin sonunda baslar, sartname 6.1.1)
      -> G1_10_40M_ANA_SEYIR             (1.076 m/s, 10-40 m)
      -> G1_40_50M_YAVASLAMA             (0.895 m/s, 40-50 m - madde4: kiyidan
                                           en az 50 m uzaklasma)
      -> G1_U_DONUS                      (0.895 m/s, heading 180 derece donene kadar)
      -> G1_GERI_DONUS_HIZLI             (1.076 m/s, bitis cizgisine ~15 m kalana kadar)
      -> G1_KIYIYA_YAKLASMA              (0.895 m/s, bitis cizgisi toleransina girene kadar)
      -> G1_BITIS_CIZGISI                (madde5: baslangic/bitis cizgisinde konum
                                           dogrulama - kararlilik suresi kadar bekle)
      -> G1_YUZEYE_CIKIS                 (sartname 6.2.1: pozitif sephiyeyle yuzeye cik)
      -> G1_TAMAMLANDI                   (terminal - guvenli, sabit, yuzeyde)

  GOREV 2 - Atis Gorevi (mission_id=2), KTR Tablo 2 ile birebir + sartname
  duzeltmesi (asagida aciklanmistir):
    REFERANSLAMA
      -> AKUSTIK_UYARI_GOREV_BASLATMA
      -> G2_ACILI_DALIS                  (0.400 m/s, KTR'deki "acili kontrollu
                                           dalis" degeri - ileri hareketle
                                           BIRLIKTE 2 m derinlige inilir, DALIS
                                           fazinin aksine yerinde degil ILERLERKEN
                                           dalar - KTR'de arac yuzeyden basliyor)
      -> G2_30M_TAMAMLAMA_SEYIR          (1.076 m/s - DUZELTME asagida)
      -> G2_GUVENLI_ATIS_BOLGESI         (hiz 0, stabil bekleme)
      -> G2_TIRMANIS_35_DERECE           (0.340 m/s, +35 derece hedef pitch ile
                                           yuzeye tirmanis)
      -> G2_ACI_VE_YUZEY_DOGRULAMA       (pitch>=30 derece VE burun disarida/
                                           kuyruk icerde kosulu birlikte dogrulanir)
      -> G2_BURUN_KAPAGI_AC              (3 sn sabit sure)
      -> G2_ATES_SINYALI_GONDER          (TAMAMEN OTOMATIK, manuel tetikleme YOK)
      -> G2_TAMAMLANDI                   (terminal)

  *** KTR TABLO 1 (GOREV 2) HATA DUZELTMESI ***
  KTR raporunda "Acili kontrollu dalis" fazina 10.00 sn / 0.400 m/s
  verilmisti (~4 m). Ancak sartname Madde 1 acikca "kiyiya dik ve duz
  istikamette 30 metre ilerledikten sonra..." diyor - KTR bu 30 m'lik
  YATAY ilerlemeyi hesaba katmamis. Eger tum 30 m 0.400 m/s ile
  alinsaydi tek basina 75 sn surer, bu da Asama-2'nin sartnamedeki
  1 dakikalik (60 sn) sinirini asardi.
  DUZELTME: KTR'nin 0.400 m/s degeri ATILMADI - "G2_ACILI_DALIS" fazi
  olarak GERCEK anlamiyla (yuzeyden 2 m'ye inis manevrasi) korundu.
  Bunun UZERINE, 30 m'nin geri kalanini tamamlayan YENI bir faz
  (G2_30M_TAMAMLAMA_SEYIR) eklendi ve bu faz Gorev 1'deki cruise hizini
  (1.076 m/s) kullanir. Toplam mesafe muhasebesi (approx_distance,
  navigation_node'da surekli birikir) her iki fazda da ayni sayaci
  kullandigi icin, dalis fazinda kac metre alindigi ONEMLI DEGILDIR -
  G2_30M_TAMAMLAMA_SEYIR fazi otomatik olarak "30 m'ye ulasana kadar"
  devam eder. Tipik sure butcesi: dalis ~10 sn (~4 m) + tamamlama
  (30-4)/1.076 ~= 24.2 sn = toplam ~34 sn seyir + guvenli bolge
  bekleme + tirmanis + dogrulama + burun kapagi (3 sn sabit) + ates
  sinyali bekleme - KTR'nin diger fazlariyla toplandiginda 60 sn
  sinirinin GUVENLE ALTINDA kalir.

GUVENLIK KURALLARI (rapor + kullanicidan gelen sartlar):
  - Acil durdurma butonuna basilirsa motorlar HEMEN durur (ayni kontrol
    dongusunde, forward_motion_request=False).
  - Acma butonundan (gorev baslangicindan) sonra 60 sn gecmeden motorlar
    CALISMAZ (AKUSTIK_UYARI_GOREV_BASLATMA fazi - mission_start_node
    tarafindan yonetilir, Tablo 11).
  - Atesleme MANUEL DEGIL: launch_request sadece otomatik kosul
    degerlendirmesiyle uretilir; hicbir "manuel tetikle" girdisi YOKTUR.
  - Birden fazla bagimsiz fail-safe kontrolu: acil durdurma, navigasyon
    veri gecerliligi/zaman asimi, azami gorev suresi, su kacagi (leak),
    atesleme-fazina-ozel bagimsiz zaman asimi.

Gudum katmani DOGRUDAN motor/servo sinyali URETMEZ; sadece hedef
derinlik/heading/pitch/hiz ve gorev fazi kararlarini uretir (Otopilot/PID
katmani bunlari tuketir). Burun kapagi acma ve ates sinyali de birer
"ISTEK/komut talebidir" - launch_request ozelinde nihai firlatma
onayi ayri bir Guvenlik Katmani dugumunde (safety.py) BAGIMSIZ olarak
dogrulanir (orn. launch_min_pitch_deg=25, gudumun 30 derece esiginden
KASITLI olarak dusuk tutulur - iki bagimsiz kontrol ayni anda AYNI
noktada basarisiz olmasin diye).

Girdi:
    /sara/navigation/odom               (nav_msgs/Odometry)
    /sara/navigation/surface_detected   (std_msgs/Bool)
    /sara/navigation/status             (diagnostic_msgs/DiagnosticStatus)
    /sara/water_detect_1                (std_msgs/Bool)  -- burun
    /sara/water_detect_2                (std_msgs/Bool)  -- kuyruk
    /sara/safety/emergency_stop         (std_msgs/Bool, yoksa False varsayilir)
    /sara/safety/leak_detected          (std_msgs/Bool, yoksa False varsayilir)
    /sara/mission_start/motion_permission (std_msgs/Bool)

Cikti:
    /sara/guidance/target_pose            (geometry_msgs/PoseStamped)
    /sara/guidance/mission_phase          (std_msgs/String)
    /sara/guidance/forward_motion_request (std_msgs/Bool)
    /sara/guidance/target_speed           (std_msgs/Float64, m/s)
    /sara/guidance/nose_cap_open_request  (std_msgs/Bool)
    /sara/guidance/launch_request         (std_msgs/Bool)  -- SADECE ISTEK, OTOMATIK
    /sara/guidance/task_complete          (std_msgs/Bool)  -- YENI: terminal faz gostergesi
                                            (sartname 6.2.1 "enerjiyi keserek" gereksinimi icin)
    /sara/guidance/status                 (diagnostic_msgs/DiagnosticStatus)

NOT: Akustik uyari/buzzer sinyali BU node'da degil, ayri mission_start.py
node'unda uretilir (/sara/mission_start/acoustic_warning) - Tablo 11'de
Gudum'den bagimsiz bir katman olarak tanimlandigi icin ayrildi.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


# ---------------------------------------------------------------------------
# Gorev fazlari
# ---------------------------------------------------------------------------
PHASE_REFERANSLAMA = 0
PHASE_AKUSTIK_UYARI_GOREV_BASLATMA = 1
PHASE_DALIS = 2

# Gorev 1 - Seyir (KTR Tablo 1 ile birebir, her satir ayri bir faz)
PHASE_G1_0_10M_KALIBRASYON = 10
PHASE_G1_10_40M_ANA_SEYIR = 11
PHASE_G1_40_50M_YAVASLAMA = 12
PHASE_G1_U_DONUS = 13
PHASE_G1_GERI_DONUS_HIZLI = 14
PHASE_G1_KIYIYA_YAKLASMA = 15
PHASE_G1_BITIS_CIZGISI = 16
PHASE_G1_YUZEYE_CIKIS = 17
PHASE_G1_TAMAMLANDI = 18

# Gorev 2 - Atis (KTR Tablo 2 + 30m sartname duzeltmesi, bkz. modul dokstringi)
PHASE_G2_ACILI_DALIS = 20
PHASE_G2_30M_TAMAMLAMA_SEYIR = 21
PHASE_G2_GUVENLI_ATIS_BOLGESI = 22
PHASE_G2_TIRMANIS_35_DERECE = 23
PHASE_G2_ACI_VE_YUZEY_DOGRULAMA = 24
PHASE_G2_BURUN_KAPAGI_AC = 25
PHASE_G2_ATES_SINYALI_GONDER = 26
PHASE_G2_TAMAMLANDI = 27

# Ortak fail-safe / terminal
PHASE_GUVENLI_SONLANDIRMA = 99

PHASE_NAMES = {
    PHASE_REFERANSLAMA: 'REFERANSLAMA',
    PHASE_AKUSTIK_UYARI_GOREV_BASLATMA: 'AKUSTIK_UYARI_GOREV_BASLATMA',
    PHASE_DALIS: 'DALIS',
    PHASE_G1_0_10M_KALIBRASYON: 'G1_0_10M_KALIBRASYON',
    PHASE_G1_10_40M_ANA_SEYIR: 'G1_10_40M_ANA_SEYIR',
    PHASE_G1_40_50M_YAVASLAMA: 'G1_40_50M_YAVASLAMA',
    PHASE_G1_U_DONUS: 'G1_U_DONUS',
    PHASE_G1_GERI_DONUS_HIZLI: 'G1_GERI_DONUS_HIZLI',
    PHASE_G1_KIYIYA_YAKLASMA: 'G1_KIYIYA_YAKLASMA',
    PHASE_G1_BITIS_CIZGISI: 'G1_BITIS_CIZGISI',
    PHASE_G1_YUZEYE_CIKIS: 'G1_YUZEYE_CIKIS',
    PHASE_G1_TAMAMLANDI: 'G1_TAMAMLANDI',
    PHASE_G2_ACILI_DALIS: 'G2_ACILI_DALIS',
    PHASE_G2_30M_TAMAMLAMA_SEYIR: 'G2_30M_TAMAMLAMA_SEYIR',
    PHASE_G2_GUVENLI_ATIS_BOLGESI: 'G2_GUVENLI_ATIS_BOLGESI',
    PHASE_G2_TIRMANIS_35_DERECE: 'G2_TIRMANIS_35_DERECE',
    PHASE_G2_ACI_VE_YUZEY_DOGRULAMA: 'G2_ACI_VE_YUZEY_DOGRULAMA',
    PHASE_G2_BURUN_KAPAGI_AC: 'G2_BURUN_KAPAGI_AC',
    PHASE_G2_ATES_SINYALI_GONDER: 'G2_ATES_SINYALI_GONDER',
    PHASE_G2_TAMAMLANDI: 'G2_TAMAMLANDI',
    PHASE_GUVENLI_SONLANDIRMA: 'GUVENLI_SONLANDIRMA',
}

TERMINAL_PHASES = {PHASE_G1_TAMAMLANDI, PHASE_G2_TAMAMLANDI, PHASE_GUVENLI_SONLANDIRMA}


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
        self.declare_parameter('heading_tolerance', 0.10)           # [rad] (~5.7 derece) - kanitlanmis deger
        self.declare_parameter('pitch_tolerance', 0.10)             # [rad]
        self.declare_parameter('kararlilik_suresi_s', 3.0)           # kosul kesintisiz saglanma suresi
        self.declare_parameter('max_mission_duration_s', 600.0)      # fail-safe: azami gorev suresi
        self.declare_parameter('g2_ates_sinyali_timeout_s', 30.0)     # atesleme fazina OZEL, BAGIMSIZ zaman asimi
        self.declare_parameter('nav_status_timeout_s', 2.0)           # fail-safe: navigasyon veri zaman asimi
        self.declare_parameter('startup_grace_period_s', 5.0)          # baslangicta diger node'lar henuz
                                                                            # yayina baslamadan "nav gecersiz"
                                                                            # fail-safe'i yanlislikla tetiklenmesin
        self.declare_parameter('control_rate_hz', 10.0)

        # --- Faz hizlari (KTR Tablo 1 / Tablo 2 ile dogrulanan gercek degerler) ---
        self.declare_parameter('max_calibrated_speed_ms', 1.076)   # tam itkide (thrust=1.0) ulasilan hiz
        self.declare_parameter('g1_calib_speed_ms', 0.895)          # G1_0_10M_KALIBRASYON, G1_40_50M_YAVASLAMA,
                                                                          # G1_U_DONUS, G1_KIYIYA_YAKLASMA (KTR'nin
                                                                          # ortak 0.895 m/s degeri)
        self.declare_parameter('g1_cruise_speed_ms', 1.076)          # G1_10_40M_ANA_SEYIR, G1_GERI_DONUS_HIZLI
        self.declare_parameter('g2_dive_speed_ms', 0.400)             # G2_ACILI_DALIS - KTR'nin ORIJINAL degeri,
                                                                          # DEGISTIRILMEDI (bkz. modul dokstringi)
        self.declare_parameter('g2_transit_speed_ms', 1.076)          # G2_30M_TAMAMLAMA_SEYIR - YENI faz,
                                                                          # sartnamenin 30m sartini karsilamak
                                                                          # icin eklendi (KTR hatasi duzeltmesi)
        self.declare_parameter('g2_ascend_speed_ms', 0.340)            # G2_TIRMANIS_35_DERECE

        # --- Gorev 1 (Seyir) mesafe esikleri - KTR Tablo 1 ile birebir ---
        self.declare_parameter('g1_kalibrasyon_distance_m', 10.0)       # 0-10m kalibrasyon sonu /
                                                                              # sartname 6.1.1: yarisma suresi
                                                                              # burada baslar / bitis cizgisi konumu
        self.declare_parameter('g1_ana_seyir_distance_m', 40.0)          # 10-40m ana seyir sonu
        self.declare_parameter('g1_uzaklasma_min_distance_m', 50.0)       # 40-50m yavaslama sonu /
                                                                              # madde4: kiyidan en az 50m
        self.declare_parameter('g1_yaklasma_baslangic_m', 15.0)            # geri donus hizli seyir sonu
                                                                              # (bitis cizgisine ~15m kala yavasla)
        self.declare_parameter('g1_bitis_cizgisi_tolerance_m', 2.0)         # kiyiya yaklasma sonu /
                                                                              # bitis cizgisine varis toleransi
                                                                              # (kanitlanmis deger, PID henuz
                                                                              # ayarlanmadan sikilastirilmadi)

        # --- Gorev 2 (Atis) mesafe/aci esikleri ---
        self.declare_parameter('g2_duz_seyir_distance_m', 30.0)          # madde 1: 30 m duz git (ACILI_DALIS +
                                                                              # 30M_TAMAMLAMA_SEYIR TOPLAMDA bu
                                                                              # mesafeyi kat eder)
        # Guvenli atis bolgesi: 30m'lik mesafe hedefi etrafinda tolerans bandi.
        # Sartname bunun disinda ayrica bir saha/GPS koordinati vermiyor;
        # arac GPS/DVL'siz oldugu icin (dead-reckoning) bu, mesafe bazli
        # bir tolerans bandi olarak modellenmistir - saha testleriyle
        # (dead-reckoning sapma miktarina gore) ayarlanabilir.
        self.declare_parameter('g2_safe_zone_min_m', 25.0)
        self.declare_parameter('g2_safe_zone_max_m', 35.0)
        self.declare_parameter('g2_tirmanis_target_pitch_deg', 35.0)         # madde 3: 30 dereceden FAZLA
                                                                                  # hedeflenmeli, pay icin 35 kullanildi
        self.declare_parameter('g2_min_launch_pitch_deg', 30.0)               # sartname 4.1: ">30 derece" (Madde 4.1)
        self.declare_parameter('g2_firing_depth', 0.0)                        # gorev tanimi acikca "yuzeye cik"
                                                                                  # diyor - gercek yuzey (0m)
        self.declare_parameter('g2_nose_cap_open_duration_s', 3.0)            # TODO: gercek servo suresiyle
                                                                                  # guncellenecek

        # ================= Iceri aktar =================
        self.mission_id = int(self.get_parameter('mission_id').value)

        self.dive_target_depth = self.get_parameter('dive_target_depth').value
        self.depth_tol = self.get_parameter('depth_tolerance').value
        self.heading_tol = self.get_parameter('heading_tolerance').value
        self.pitch_tol = self.get_parameter('pitch_tolerance').value
        self.kararlilik_suresi = self.get_parameter('kararlilik_suresi_s').value
        self.max_mission_duration = self.get_parameter('max_mission_duration_s').value
        self.g2_ates_sinyali_timeout = self.get_parameter('g2_ates_sinyali_timeout_s').value
        self.nav_status_timeout = self.get_parameter('nav_status_timeout_s').value
        self.startup_grace_period = self.get_parameter('startup_grace_period_s').value

        self.max_calibrated_speed = self.get_parameter('max_calibrated_speed_ms').value
        self.g1_calib_speed = self.get_parameter('g1_calib_speed_ms').value
        self.g1_cruise_speed = self.get_parameter('g1_cruise_speed_ms').value
        self.g2_dive_speed = self.get_parameter('g2_dive_speed_ms').value
        self.g2_transit_speed = self.get_parameter('g2_transit_speed_ms').value
        self.g2_ascend_speed = self.get_parameter('g2_ascend_speed_ms').value

        self.g1_kalibrasyon_distance = self.get_parameter('g1_kalibrasyon_distance_m').value
        self.g1_ana_seyir_distance = self.get_parameter('g1_ana_seyir_distance_m').value
        self.g1_uzaklasma_min_distance = self.get_parameter('g1_uzaklasma_min_distance_m').value
        self.g1_yaklasma_baslangic_m = self.get_parameter('g1_yaklasma_baslangic_m').value
        self.g1_bitis_cizgisi_tolerance = self.get_parameter('g1_bitis_cizgisi_tolerance_m').value

        self.g2_duz_seyir_distance = self.get_parameter('g2_duz_seyir_distance_m').value
        self.g2_safe_zone_min = self.get_parameter('g2_safe_zone_min_m').value
        self.g2_safe_zone_max = self.get_parameter('g2_safe_zone_max_m').value
        self.g2_tirmanis_target_pitch = math.radians(self.get_parameter('g2_tirmanis_target_pitch_deg').value)
        self.g2_min_launch_pitch = math.radians(self.get_parameter('g2_min_launch_pitch_deg').value)
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
        self._race_timer_start = None   # Gorev1 madde3/6.1.1: "yarisma suresi baslasin" - ic telemetri isaretcisi

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
        self._target_speed = 0.0
        self._nose_cap_open_request = False
        self._launch_request = False

        # ================= Abonelikler =================
        self.create_subscription(Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/sara/navigation/surface_detected', self._on_surface, 10)
        # DUZELTME (KTR sayfa 14/37): Atesleme izni "yuzeyde mi degil mi" gibi
        # tek bir genel bayrakla degil, TAM OLARAK "burun su DISINDA, kuyruk
        # su ICINDE" (kismi cikis acisi) kosuluyla verilmelidir. Bu yuzden
        # ham sensorlere DOGRUDAN abone oluyoruz.
        # DUZELTME: sensor topic'leri (vehicle_sim/gercek donanim) BEST_EFFORT
        # QoS ile yayin yapar. Varsayilan (RELIABLE) abonelik bunlarla
        # UYUMSUZDUR - hicbir mesaj alinamaz, sessizce basarisiz olur.
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
        self._speed_pub = self.create_publisher(Float64, '/sara/guidance/target_speed', 10)
        self._nose_cap_pub = self.create_publisher(Bool, '/sara/guidance/nose_cap_open_request', 10)
        self._launch_pub = self.create_publisher(Bool, '/sara/guidance/launch_request', 10)
        # YENI (sartname 6.2.1 uyumu): SARA, bitis cizgisinde/gorev sonunda
        # "enerjiyi keserek guvenli bir sekilde... yuzer bir durumda"
        # olmalidir (90 puan). Bu sinyal, terminal bir faza (G1/G2
        # TAMAMLANDI veya GUVENLI_SONLANDIRMA) ulasildiginda True olur;
        # safety_node bunu dinleyip nihai bir "ana guc kesme" komutu
        # uretir (fiziksel MOSFET hattina baglanmasi ayri bir donanim
        # gorevidir, KTR'de tanimlidir).
        self._task_complete_pub = self.create_publisher(Bool, '/sara/guidance/task_complete', 10)
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

    @property
    def _return_heading(self) -> float:
        """G1'de baslangic/bitis cizgisine geri donus icin hedef heading
        (referans heading'in tam tersi, 180 derece dondurulmus)."""
        return wrap_pi(self._reference_heading + math.pi)

    def _distance_to_finish_line(self) -> float:
        """G1'de bitis cizgisi, kiyidan g1_kalibrasyon_distance (10m)
        uzaklikta - sartname 6.1.1: 'kiyidan 10 metre uzaklıktaki
        baslangic/bitis cizgisi'."""
        return abs(self._approx_distance() - self.g1_kalibrasyon_distance)

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
            phase_duration = self._phase_elapsed()      # bu fazda ne kadar kaldi
            mission_elapsed = self._mission_elapsed()     # gorev basindan itibaren toplam sure
            self.get_logger().info(
                f'Gorev fazi gecisi: {PHASE_NAMES[self._phase]} -> {PHASE_NAMES[phase]} '
                f'| bu fazda gecen sure: {phase_duration:.2f} sn '
                f'| toplam gorev suresi: {mission_elapsed:.2f} sn'
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
        elif (
            self._phase == PHASE_G2_ATES_SINYALI_GONDER
            and self._phase_elapsed() > self.g2_ates_sinyali_timeout
            and not self._launch_request
        ):
            # BAGIMSIZ fail-safe: atesleme kosullari bu fazda cok uzun
            # suredir saglanamiyorsa (launch_request hic True olamadiysa),
            # genel 600sn siniri beklemeden guvenli sonlandir.
            failsafe_reason = f'atesleme kosullari {self.g2_ates_sinyali_timeout:.0f} sn icinde saglanamadi'

        if failsafe_reason is not None and self._phase not in TERMINAL_PHASES:
            self.get_logger().error(f'GUVENLI SONLANDIRMA tetiklendi: {failsafe_reason}')
            self._goto_phase(PHASE_GUVENLI_SONLANDIRMA)

        if self._phase == PHASE_REFERANSLAMA:
            self._run_referanslama()
        elif self._phase == PHASE_AKUSTIK_UYARI_GOREV_BASLATMA:
            self._run_akustik_uyari()
        elif self._phase == PHASE_DALIS:
            self._run_dalis()
        elif self._phase == PHASE_G1_0_10M_KALIBRASYON:
            self._run_g1_0_10m_kalibrasyon()
        elif self._phase == PHASE_G1_10_40M_ANA_SEYIR:
            self._run_g1_10_40m_ana_seyir()
        elif self._phase == PHASE_G1_40_50M_YAVASLAMA:
            self._run_g1_40_50m_yavaslama()
        elif self._phase == PHASE_G1_U_DONUS:
            self._run_g1_u_donus()
        elif self._phase == PHASE_G1_GERI_DONUS_HIZLI:
            self._run_g1_geri_donus_hizli()
        elif self._phase == PHASE_G1_KIYIYA_YAKLASMA:
            self._run_g1_kiyiya_yaklasma()
        elif self._phase == PHASE_G1_BITIS_CIZGISI:
            self._run_g1_bitis_cizgisi()
        elif self._phase == PHASE_G1_YUZEYE_CIKIS:
            self._run_g1_yuzeye_cikis()
        elif self._phase == PHASE_G1_TAMAMLANDI:
            self._run_g1_tamamlandi()
        elif self._phase == PHASE_G2_ACILI_DALIS:
            self._run_g2_acili_dalis()
        elif self._phase == PHASE_G2_30M_TAMAMLAMA_SEYIR:
            self._run_g2_30m_tamamlama_seyir()
        elif self._phase == PHASE_G2_GUVENLI_ATIS_BOLGESI:
            self._run_g2_guvenli_atis_bolgesi()
        elif self._phase == PHASE_G2_TIRMANIS_35_DERECE:
            self._run_g2_tirmanis()
        elif self._phase == PHASE_G2_ACI_VE_YUZEY_DOGRULAMA:
            self._run_g2_aci_ve_yuzey_dogrulama()
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
        self._target_speed = 0.0
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
            self._goto_phase(PHASE_AKUSTIK_UYARI_GOREV_BASLATMA)

    def _run_akustik_uyari(self):
        """60 sn motor inhibit + son 10 sn pinger/buzzer mission_start_node
        tarafindan yonetiliyor (Tablo 11). Bu faz sadece onun urettigi
        /sara/mission_start/motion_permission=True olmasini bekler.

        motion_permission geldiginde gorev tipine gore DALLANIR:
          Gorev 1 -> DALIS (yerinde/sabit dalis, sonra 0-10m kalibrasyon)
          Gorev 2 -> G2_ACILI_DALIS (ilerlerken dalis - KTR'de arac
                     yuzeyden basliyor ve "acili" sekilde dalıyor)
        """
        self._forward_motion = False
        self._target_speed = 0.0
        self._target_depth = 0.0
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0

        if self._motion_permission:
            if self.mission_id == 1:
                self._goto_phase(PHASE_DALIS)
            else:
                self._goto_phase(PHASE_G2_ACILI_DALIS)

    def _run_dalis(self):
        """SADECE Gorev 1 icin: baslangic alanindan itibaren yerinde/sabit
        2 m derinlige inis (sartname 6.1.1)."""
        self._forward_motion = False
        self._target_speed = 0.0
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0

        depth_ok = abs(self._depth - self.dive_target_depth) < self.depth_tol
        if self._conditions_held(depth_ok):
            self._goto_phase(PHASE_G1_0_10M_KALIBRASYON)

    # ---------------------------------------------------------- GOREV 1 (KTR Tablo 1)
    def _run_g1_0_10m_kalibrasyon(self):
        """KTR: '0-10 m kalibrasyon' fazi - dusuk/kalibrasyon hizi (0.895 m/s).
        Bu fazin sonunda sartname 6.1.1 geregi yarisma suresi (ic telemetri
        isaretcisi) baslatilir."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g1_calib_speed

        if self._approx_distance() >= self.g1_kalibrasyon_distance:
            self._race_timer_start = self.get_clock().now()
            self.get_logger().info('Ilk 10 m tamamlandi - yarisma suresi (ic telemetri) baslatildi.')
            self._goto_phase(PHASE_G1_10_40M_ANA_SEYIR)

    def _run_g1_10_40m_ana_seyir(self):
        """KTR: '10-40 m ana seyir' fazi - cruise hizi (1.076 m/s)."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g1_cruise_speed

        if self._approx_distance() >= self.g1_ana_seyir_distance:
            self._goto_phase(PHASE_G1_40_50M_YAVASLAMA)

    def _run_g1_40_50m_yavaslama(self):
        """KTR: '40-50 m donus oncesi yavaslama' fazi - kalibrasyon hizi (0.895 m/s).
        Madde4: kiyidan en az 50 m uzaklasma sarti bu fazin sonunda saglanir."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g1_calib_speed

        if self._approx_distance() >= self.g1_uzaklasma_min_distance:
            self._goto_phase(PHASE_G1_U_DONUS)

    def _run_g1_u_donus(self):
        """KTR: 'U donus kontrollu manevra' fazi - kalibrasyon hizinda (0.895
        m/s) heading 180 derece dondurulur (~2.5 m donus yaricapi)."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._return_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g1_calib_speed

        heading_aligned = abs(self._heading_error_to(self._return_heading)) < self.heading_tol
        if self._conditions_held(heading_aligned):
            self._goto_phase(PHASE_G1_GERI_DONUS_HIZLI)

    def _run_g1_geri_donus_hizli(self):
        """KTR: 'Geri donus hizli seyir' fazi - cruise hizinda (1.076 m/s),
        bitis cizgisine g1_yaklasma_baslangic_m (15 m) kalana kadar."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._return_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g1_cruise_speed

        if self._distance_to_finish_line() <= self.g1_yaklasma_baslangic_m:
            self._goto_phase(PHASE_G1_KIYIYA_YAKLASMA)

    def _run_g1_kiyiya_yaklasma(self):
        """KTR: 'Kiyiya yaklasma' fazi - kalibrasyon hizinda (0.895 m/s),
        bitis cizgisi toleransina girene kadar."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._return_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g1_calib_speed

        distance_ok = self._distance_to_finish_line() <= self.g1_bitis_cizgisi_tolerance
        if self._conditions_held(distance_ok):
            self._goto_phase(PHASE_G1_BITIS_CIZGISI)

    def _run_g1_bitis_cizgisi(self):
        """Madde5: baslangic/bitis cizgisine geri donus TAMAMLANDI - konum
        kararlilik suresi kadar dogrulanir (ileri hareket durur, heading
        korunur), sonra yuzeye cikisa gecilir."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._return_heading
        self._target_pitch = 0.0
        self._forward_motion = False
        self._target_speed = 0.0

        if self._conditions_held(True):
            self._goto_phase(PHASE_G1_YUZEYE_CIKIS)

    def _run_g1_yuzeye_cikis(self):
        """Sartname 6.2.1: SARA, bitis cizgisinde ENERJIYI KESEREK guvenli
        bir sekilde, POZITIF SEPHIYE ile su ustunde kurtarilabilir bir
        vaziyette yuzmelidir (90 puan). Bu faz, sephiye sistemi araciligiyla
        yuzeye kontrollu cikisi temsil eder."""
        self._target_depth = 0.0
        self._target_heading = self._return_heading
        self._target_pitch = 0.0
        self._forward_motion = False
        self._target_speed = 0.0

        depth_ok = self._depth <= self.depth_tol
        if self._conditions_held(depth_ok):
            self._goto_phase(PHASE_G1_TAMAMLANDI)

    def _run_g1_tamamlandi(self):
        """Terminal - arac yuzeyde, sabit, guvenli durumda bekler.
        NOT: sartnamedeki 'enerjiyi keserek gorevi sonlandirma' fiziksel
        ana guc kesme islemi, GUDUM katmaninin kapsami DISINDADIR - bu,
        ayri bir donanim/guvenlik katmani (orn. MOSFET tabanli ana guc
        kesme, KTR'de tanimli) tarafindan, arac yuzeye ulastiginda
        (surface_detected=True) tetiklenmelidir."""
        self._forward_motion = False
        self._target_speed = 0.0
        self._target_depth = 0.0
        self._target_heading = self._return_heading
        self._target_pitch = 0.0
        self._launch_request = False
        self._nose_cap_open_request = False

    # ---------------------------------------------------------- GOREV 2 (KTR Tablo 2 + duzeltme)
    def _run_g2_acili_dalis(self):
        """KTR: 'Acili kontrollu dalis' fazi - ORIJINAL KTR hizi (0.400 m/s)
        korunmustur. Arac YUZEYDEN baslar (KTR Tablo 2) ve ILERLERKEN 2 m
        derinlige iner (DALIS fazinin aksine yerinde degil, hareket
        halinde dalar - bu yuzden 'acili'). target_pitch=0 birakilarak
        gercek dalis acisi otopilotun derinlik-trim kaskadindan (kisa
        sureli pitch trim) emergent olarak olusur."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g2_dive_speed

        depth_ok = abs(self._depth - self.dive_target_depth) < self.depth_tol
        if self._conditions_held(depth_ok):
            self._goto_phase(PHASE_G2_30M_TAMAMLAMA_SEYIR)

    def _run_g2_30m_tamamlama_seyir(self):
        """YENI FAZ - KTR Tablo 1 hata duzeltmesi (bkz. modul dokstringi):
        sartname madde1'in gerektirdigi TOPLAM 30 m yatay ilerlemenin,
        G2_ACILI_DALIS fazinda alinamayan kalanini cruise hizinda
        (1.076 m/s) tamamlar. approx_distance() surekli biriken bir
        sayac oldugu icin dalis fazinda kac metre alindigi onemli
        degildir - bu faz otomatik olarak '30 m toplam'a ulasana kadar
        devam eder."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = True
        self._target_speed = self.g2_transit_speed

        if self._approx_distance() >= self.g2_duz_seyir_distance:
            self._goto_phase(PHASE_G2_GUVENLI_ATIS_BOLGESI)

    def _run_g2_guvenli_atis_bolgesi(self):
        """KTR: 'Ateşleme öncesi stabil bekleme' fazi - hiz 0.

        DUZELTME (saha testinde bulundu - 388 sn kilitlenme): Bu faz
        aracin DURMASINI komut eder (forward_motion=False, hedef hiz=0).
        Eskiden kosul icinde self._motion_consistent (navigation'un
        odom.twist.x'inden tureyen, "arac SU AN ILERI HAREKET EDIYOR mu"
        anlamina gelen bir bayrak) de aranıyordu. Ancak itki sifirlanip
        arac gercekten durunca twist.x de sifira dustugu icin
        motion_consistent HICBIR ZAMAN True olamiyordu - kendi kendini
        engelleyen bir kosuldu (arac hem DURSUN hem de HAREKET EDIYOR
        OLSUN isteniyordu). Yerine, aracin GERCEKTEN hedef derinlikte
        SABIT kaldigini dogrulayan depth_ok kontrolu eklendi - bu, "stabil
        bekleme" niyetine (KTR) çok daha uygun bir dogrulamadir."""
        self._target_depth = self.dive_target_depth
        self._target_heading = self._reference_heading
        self._target_pitch = 0.0
        self._forward_motion = False
        self._target_speed = 0.0

        dist = self._approx_distance()
        in_zone = self.g2_safe_zone_min <= dist <= self.g2_safe_zone_max
        depth_ok = abs(self._depth - self.dive_target_depth) < self.depth_tol
        conditions_ok = in_zone and depth_ok and not self._emergency_stop

        if self._conditions_held(conditions_ok):
            self._goto_phase(PHASE_G2_TIRMANIS_35_DERECE)

    def _run_g2_tirmanis(self):
        """KTR: 'Yuzeye cikis' fazi - kontrollu cikis hizi (0.340 m/s),
        hedef pitch +35 derece (sartname: >30 derece, pay icin 35 kullanildi)."""
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True
        self._target_speed = self.g2_ascend_speed

        # Sartname 4.1: ">30 derece" sarti - TEK YONLU esik, simetrik
        # tolerans DEGIL (simetrik tolerans 30 derecenin ALTINI da kabul ederdi).
        pitch_ok = self._pitch >= self.g2_min_launch_pitch
        # Derinlik fiziksel olarak 0'in altina inemez (yuzey siniri) -
        # "tam esitlik" yerine "yeterince sig/yuzeye ulasti mi" kontrolu.
        depth_ok = self._depth <= (self.g2_firing_depth + self.depth_tol)
        if self._conditions_held(pitch_ok and depth_ok):
            self._goto_phase(PHASE_G2_ACI_VE_YUZEY_DOGRULAMA)

    def _run_g2_aci_ve_yuzey_dogrulama(self):
        """Madde4: dogru aciyi algila. KTR (sayfa 14/37): 'dogru aci'
        sadece pitch degeriyle DEGIL, fiziksel olarak burun su disinda +
        kuyruk su icinde olmasiyla (kismi cikis) BIRLIKTE dogrulanir.
        Itki TAMAMEN KESILMEZ - yuzeye yakin acili pozisyon itkisiz
        korunamayabilir, hafif itki ile pozisyon korunur."""
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True
        self._target_speed = self.g2_ascend_speed

        pitch_ok = self._pitch >= self.g2_min_launch_pitch  # >30 derece sarti, tek yonlu
        dogru_aci_ok = pitch_ok and self._dogru_cikis_acisi
        if self._conditions_held(dogru_aci_ok):
            self._goto_phase(PHASE_G2_BURUN_KAPAGI_AC)

    def _run_g2_burun_kapagi_ac(self):
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True  # pozisyonu korumak icin hafif itki
        self._target_speed = self.g2_ascend_speed
        self._nose_cap_open_request = True

        if self._phase_elapsed() >= self.g2_nose_cap_open_duration:
            self._goto_phase(PHASE_G2_ATES_SINYALI_GONDER)

    def _run_g2_ates_sinyali(self):
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch
        self._forward_motion = True  # pozisyonu korumak icin hafif itki
        self._target_speed = self.g2_ascend_speed
        self._nose_cap_open_request = True

        pitch_ok = self._pitch >= self.g2_min_launch_pitch  # >30 derece sarti (Sartname 4.1), tek yonlu
        nav_ok = self._nav_ok() and self._pixhawk_connected
        # KTR sayfa 14/37: atesleme izni "yuzeyde mi" gibi genel bir
        # bayrakla DEGIL, "burun su disinda VE kuyruk su icinde" TAM
        # kosuluyla verilmelidir.
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
        """Terminal - ates sinyali gonderildi, arac guvenli/sabit pozisyonda
        bekler."""
        self._forward_motion = False
        self._target_speed = 0.0
        self._launch_request = False
        self._nose_cap_open_request = True
        self._target_depth = self.g2_firing_depth
        self._target_pitch = self.g2_tirmanis_target_pitch

    # ---------------------------------------------------------- Fail-safe
    def _run_guvenli_sonlandirma(self):
        self._forward_motion = False
        self._target_speed = 0.0
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

        speed_msg = Float64()
        speed_msg.data = float(self._target_speed) if self._forward_motion else 0.0
        self._speed_pub.publish(speed_msg)

        cap = Bool()
        cap.data = bool(self._nose_cap_open_request)
        self._nose_cap_pub.publish(cap)

        launch = Bool()
        launch.data = bool(self._launch_request)
        self._launch_pub.publish(launch)

        task_complete = Bool()
        task_complete.data = self._phase in TERMINAL_PHASES
        self._task_complete_pub.publish(task_complete)

        status = DiagnosticStatus()
        status.name = 'sara_guidance'
        status.hardware_id = 'jetson_orin_nano'
        status.level = DiagnosticStatus.ERROR if self._phase == PHASE_GUVENLI_SONLANDIRMA else DiagnosticStatus.OK
        status.message = f'Gorev {self.mission_id} - Faz: {PHASE_NAMES[self._phase]}'

        race_elapsed = self._race_elapsed()
        status.values = [
            KeyValue(key='mission_id', value=str(self.mission_id)),
            KeyValue(key='approx_distance_m', value=f'{self._approx_distance():.2f}'),
            KeyValue(key='distance_to_finish_line_m', value=f'{self._distance_to_finish_line():.2f}'),
            KeyValue(key='heading_error_rad', value=f'{self._heading_error_to(self._target_heading):.3f}'),
            KeyValue(key='pitch_deg', value=f'{math.degrees(self._pitch):.1f}'),
            KeyValue(key='target_speed_ms', value=f'{self._target_speed:.3f}'),
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