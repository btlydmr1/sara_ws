#!/usr/bin/env python3
"""
actuator_driver.py
====================
SARA platformu - PCA9685 Eyleyici Surucusu (GERCEK DONANIM)

Bagli donanim (kullanicidan alinan gercek kablolama):
    PCA9685 (I2C PWM surucu karti)
        VCC -> Jetson pin 17 (3.3V)
        GND -> ortak GND
        SDA -> Jetson pin 3
        SCL -> Jetson pin 5
        V+  -> harici servo BEC (guc kaynagi, Jetson'dan DEGIL)
        CH0 -> ESC (itki motoru)
        CH1 -> Pitch servo (kanatcik/elevator)
        CH2 -> Yaw servo (kanatcik/rudder)
        CH3 -> Burun kapagi servosu

*** ONEMLI - PIXHAWK BU KATMANDA KULLANILMAZ ***
Bu donanim topolojisinde Pixhawk 6X SADECE IMU/telemetri kaynagidir
(UART/MAVLink -> mavros -> /mavros/imu/data). Eyleyiciler Pixhawk'tan
DEGIL, dogrudan Jetson'dan (bu node araciligiyla) suruluyor. Bu yuzden
'pixhawk_bridge.py' (RC override) BU DONANIMLA KULLANILMAMALIDIR -
onun yerine bu node kullanilir.

*** EKSIK/BEKLEYEN DONANIM (bu node bunlari SURMEZ) ***
- Sephiye (step motor) icin ayri bir surucu/pin ataması NETLESMEDI.
  /sara/control/buoyancy_command dinlenir ama HICBIR FIZIKSEL CIKISA
  baglanmaz (TODO).
- Roket atesleme (/sara/control/launch_command) icin FIZIKSEL BAGLANTI
  TANIMLANMADI. Bu node bu topic'i kasitli olarak DINLEMEZ - atesleme
  devresi ayri, ozel olarak tasarlanip dogrulanmadan hicbir yazilim
  bu sinyali bir role/squib'e baglamamalidir.
- YENI: Ana guc kesme (/sara/safety/main_power_cutoff_command, sartname
  6.2.1 - gorev sonu "enerjiyi keserek" sarti) icin de FIZIKSEL BAGLANTI
  (MOSFET gate pini) NETLESMEDI. Bu node bu topic'i de kasitli olarak
  DINLEMEZ. KTR'de tanimlanan 22.2V/10A MOSFET devresi saha/donanim
  ekibi tarafindan netlestirildiginde, bu topic'e abone olan ve ilgili
  GPIO/PWM pinini suren kucuk, ayri bir surucu (orn. main_power_driver.py)
  eklenmelidir - launch_command icin uygulanan "ayri, dogrulanmis devre"
  prensibiyle AYNI sekilde.
- YENI (denetimde bulundu): Sartname Madde 4.1 GEREGI ZORUNLU olan
  Buzzer/Pinger icin de FIZIKSEL PIN ATAMASI NETLESMEDI.
  mission_start_node, /sara/mission_start/acoustic_warning sinyalini
  DOGRU ZAMANLAMAYLA (hareketten 10 sn once True, hareket basladiginda
  False) uretiyor ve telemetry_node bunu SADECE CSV'ye kaydediyor - ANCAK
  hicbir dosya bu sinyali GERCEK bir buzzer/pinger cikisina baglamiyordu.
  Bu node artik BU TOPIC'E ABONE OLUR (asagida) ve durumu status
  mesajinda gorunur kilar (buzzer_wired=False), boylece eksiklik SESSIZCE
  gozden kacmaz. Gercek GPIO/PWM pini netlestiginde, _on_acoustic_warning
  callback'ine tek satirlik bir GPIO.output(...) veya PCA9685 kanal yazma
  eklemek yeterli olacaktir.

Girdi (SADECE safety_node'un nihai komutlari):
    /sara/control/thrust_command      (std_msgs/Float64, [0,1])
    /sara/control/fin_command         (geometry_msgs/Vector3, x=pitch y=yaw, [-1,1])
    /sara/control/nose_cap_command    (std_msgs/Bool)
    /sara/mission_start/acoustic_warning (std_msgs/Bool)  -- YENI: buzzer/pinger
                                          sinyali (sartname 4.1). Fiziksel pin
                                          NETLESENE KADAR sadece durum olarak
                                          izlenir (buzzer_wired=False status).

Cikti:
    /sara/actuator/status (diagnostic_msgs/DiagnosticStatus)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


def pulse_us_to_duty_cycle(pulse_us: float, freq_hz: float) -> int:
    """Mikrosaniye cinsinden PWM darbe genisligini PCA9685 16-bit duty_cycle degerine cevirir."""
    period_us = 1_000_000.0 / freq_hz
    ratio = max(0.0, min(1.0, pulse_us / period_us))
    return int(round(ratio * 65535))


class ActuatorDriverNode(Node):

    def __init__(self):
        super().__init__('actuator_driver')

        # ================= Kanal atamalari (KULLANICIDAN ALINAN GERCEK KABLOLAMA) =================
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('pwm_frequency_hz', 50.0)   # standart servo/ESC frekansi

        self.declare_parameter('thrust_channel', 0)
        self.declare_parameter('pitch_channel', 1)
        self.declare_parameter('yaw_channel', 2)
        self.declare_parameter('nose_cap_channel', 3)

        # Darbe genisligi kalibrasyonu [us] - TODO: gercek servo/ESC ile dogrulanip ayarlanmali
        self.declare_parameter('pulse_min_us', 1000.0)
        self.declare_parameter('pulse_mid_us', 1500.0)
        self.declare_parameter('pulse_max_us', 2000.0)
        self.declare_parameter('nose_cap_closed_us', 1000.0)
        self.declare_parameter('nose_cap_open_us', 2000.0)

        # ================= KAVITASYON GUVENLIGI (KTR ile tutarlilik) =================
        # KTR raporu (Bolum: Maksimum Sapma Acisi) CFD analizleriyle 25
        # derecede kavitasyon riski tespit edip "yazilimsal olarak kanatcik
        # sapma acisi maksimum 20 derece ile sinirlandirilmistir" diye
        # ACIKCA iddia ediyor. DUZELTME: bu limit önceden HICBIR YERDE
        # uygulanmiyordu - PID cikislari [-1,1] normalize araligi dogrudan
        # pulse_min_us..pulse_max_us'a haritalaniyordu, gercek derece
        # cinsinden bir sinir YOKTU. Asagidaki iki parametre, normalize
        # komutu pulse'a cevirmeden ONCE gercek derece cinsinden kirpar:
        #   fin_full_mechanical_range_deg: [-1,1] -> pulse_min..pulse_max
        #     araliginin karsiladigi TOPLAM mekanik servo sapmasi (yaklasik
        #     +-75 derece, servo_controller.py'deki MAX_ANGLE ile tutarli
        #     varsayilan - GERCEK SERVO/LINKAGE ILE SAHADA DOGRULANMALI).
        #   fin_max_deflection_deg: KTR'nin izin verdigi azami sapma (20).
        self.declare_parameter('fin_full_mechanical_range_deg', 75.0)
        self.declare_parameter('fin_max_deflection_deg', 20.0)

        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)

        self.thrust_ch = int(self.get_parameter('thrust_channel').value)
        self.pitch_ch = int(self.get_parameter('pitch_channel').value)
        self.yaw_ch = int(self.get_parameter('yaw_channel').value)
        self.nose_cap_ch = int(self.get_parameter('nose_cap_channel').value)

        self.pulse_min = self.get_parameter('pulse_min_us').value
        self.pulse_mid = self.get_parameter('pulse_mid_us').value
        self.pulse_max = self.get_parameter('pulse_max_us').value
        self.nose_closed_us = self.get_parameter('nose_cap_closed_us').value
        self.nose_open_us = self.get_parameter('nose_cap_open_us').value

        # KAVITASYON GUVENLIGI: normalize [-1,1] komutunun kirpilacagi oran.
        # orn. full_range=75 deg, max_deflection=20 deg -> fraction=0.267,
        # yani PID cikisi ne kadar buyuk olursa olsun servoya en fazla
        # +-0.267 (yani gercekte +-20 derece) olarak yazilir.
        full_range_deg = self.get_parameter('fin_full_mechanical_range_deg').value
        max_deflection_deg = self.get_parameter('fin_max_deflection_deg').value
        self.fin_clamp_fraction = (
            max(0.0, min(1.0, max_deflection_deg / full_range_deg)) if full_range_deg > 0.0 else 1.0
        )

        self.command_timeout = self.get_parameter('command_timeout_s').value
        self.freq = self.get_parameter('pwm_frequency_hz').value
        rate = float(self.get_parameter('control_rate_hz').value)

        # ================= PCA9685 baglantisi =================
        self._pca = None
        if HARDWARE_AVAILABLE:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self._pca = PCA9685(i2c, address=int(self.get_parameter('i2c_address').value))
                self._pca.frequency = self.freq
                self.get_logger().info('PCA9685 baglantisi kuruldu.')
            except Exception as e:
                self.get_logger().error(f'PCA9685 baglantisi KURULAMADI: {e}')
                self._pca = None
        else:
            self.get_logger().error(
                'adafruit_pca9685/board/busio bulunamadi! '
                '"pip install adafruit-circuitpython-pca9685" ile kurun. '
                'Bu node donanima ULASAMIYOR (guvenli - hicbir sey yazilmiyor).'
            )

        # ================= Ic durum =================
        self._thrust_command = 0.0
        self._fin_command = Vector3()
        self._nose_cap_command = False
        self._acoustic_warning = False   # YENI - buzzer/pinger sinyali (henuz fiziksel cikisa baglanmadi)

        self._last_thrust_time = None
        self._last_fin_time = None
        self._last_nose_cap_time = None

        # ================= Abonelikler - SADECE nihai/onayli komutlar =================
        self.create_subscription(Float64, '/sara/control/thrust_command', self._on_thrust, 10)
        self.create_subscription(Vector3, '/sara/control/fin_command', self._on_fin, 10)
        self.create_subscription(Bool, '/sara/control/nose_cap_command', self._on_nose_cap, 10)
        # YENI: buzzer/pinger sinyali - fiziksel pin netlesene kadar sadece
        # dinlenir/loglanir, hicbir GPIO/PWM cikisina yazilmaz (bkz. dosya
        # basligindaki "EKSIK/BEKLEYEN DONANIM" notu).
        self.create_subscription(Bool, '/sara/mission_start/acoustic_warning', self._on_acoustic_warning, 10)

        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/actuator/status', 10)

        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().warn(
            'actuator_driver baslatildi. Kanallar: thrust=CH%d, pitch=CH%d, yaw=CH%d, '
            'nose_cap=CH%d. SEPHIYE, ATESLEME VE BUZZER/PINGER BU NODE TARAFINDAN '
            'FIZIKSEL OLARAK SURULMUYOR (donanim '
            'netlesmedi).' % (self.thrust_ch, self.pitch_ch, self.yaw_ch, self.nose_cap_ch)
        )

    # ======================================================================
    def _on_thrust(self, msg: Float64):
        self._thrust_command = msg.data
        self._last_thrust_time = self.get_clock().now()

    def _on_fin(self, msg: Vector3):
        self._fin_command = msg
        self._last_fin_time = self.get_clock().now()

    def _on_nose_cap(self, msg: Bool):
        self._nose_cap_command = msg.data

    def _on_acoustic_warning(self, msg: Bool):
        # TODO (donanim netlesince): burada gercek GPIO.output(BUZZER_PIN, ...)
        # veya ilgili PCA9685 kanalina yazma eklenmelidir. Su an sadece
        # durum takip edilir ve status mesajinda gorunur kilinir - boylece
        # sartname 4.1'in zorunlu buzzer/pinger gereksinimi SESSIZCE
        # atlanmis olmaz, acikca "wired=False" olarak raporlanir.
        self._acoustic_warning = msg.data
        self._last_nose_cap_time = self.get_clock().now()

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < self.command_timeout

    # ======================================================================
    def _write_channel(self, channel: int, pulse_us: float):
        if self._pca is None:
            return
        duty = pulse_us_to_duty_cycle(pulse_us, self.freq)
        try:
            self._pca.channels[channel].duty_cycle = duty
        except Exception as e:
            self.get_logger().error(f'PCA9685 kanal {channel} yazma hatasi: {e}')

    def _on_timer(self):
        # --- Itki: 0..1 -> pulse_min..pulse_max ---
        thrust = self._thrust_command if self._fresh(self._last_thrust_time) else 0.0
        thrust = max(0.0, min(1.0, thrust))
        thrust_pulse = self.pulse_min + thrust * (self.pulse_max - self.pulse_min)
        self._write_channel(self.thrust_ch, thrust_pulse)

        # --- Kanatciklar: -1..1 -> pulse_min..pulse_max (0=orta) ---
        if self._fresh(self._last_fin_time):
            pitch = max(-1.0, min(1.0, self._fin_command.x))
            yaw = max(-1.0, min(1.0, self._fin_command.y))
        else:
            pitch = 0.0
            yaw = 0.0
        # KAVITASYON GUVENLIGI (KTR: azami 20 derece sapma) - PID cikisi ne
        # kadar buyuk olursa olsun, fiziksel servoya bu oranin OTESINDE bir
        # komut asla gitmez.
        pitch = max(-self.fin_clamp_fraction, min(self.fin_clamp_fraction, pitch))
        yaw = max(-self.fin_clamp_fraction, min(self.fin_clamp_fraction, yaw))
        pitch_pulse = self.pulse_mid + pitch * (self.pulse_max - self.pulse_mid)
        yaw_pulse = self.pulse_mid + yaw * (self.pulse_max - self.pulse_mid)
        self._write_channel(self.pitch_ch, pitch_pulse)
        self._write_channel(self.yaw_ch, yaw_pulse)

        # --- Burun kapagi: Bool -> iki konum ---
        nose_open = self._nose_cap_command if self._fresh(self._last_nose_cap_time) else False
        nose_pulse = self.nose_open_us if nose_open else self.nose_closed_us
        self._write_channel(self.nose_cap_ch, nose_pulse)

        self._publish_status(thrust, pitch, yaw, nose_open)

    def _publish_status(self, thrust, pitch, yaw, nose_open):
        status = DiagnosticStatus()
        status.name = 'sara_actuator_driver'
        status.hardware_id = 'pca9685'
        status.level = DiagnosticStatus.OK if self._pca is not None else DiagnosticStatus.ERROR
        status.message = 'Nominal' if self._pca is not None else 'PCA9685 baglantisi yok'
        status.values = [
            KeyValue(key='thrust', value=f'{thrust:.2f}'),
            KeyValue(key='pitch', value=f'{pitch:.2f}'),
            KeyValue(key='yaw', value=f'{yaw:.2f}'),
            KeyValue(key='fin_clamp_fraction', value=f'{self.fin_clamp_fraction:.3f}'),
            KeyValue(key='nose_cap_open', value=str(nose_open)),
            KeyValue(key='acoustic_warning_received', value=str(self._acoustic_warning)),
            KeyValue(key='buzzer_wired', value='False'),
            KeyValue(key='buoyancy_wired', value='False'),
            KeyValue(key='launch_wired', value='False'),
        ]
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriverNode()
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