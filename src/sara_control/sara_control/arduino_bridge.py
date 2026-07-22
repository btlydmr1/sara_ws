#!/usr/bin/env python3
"""
arduino_bridge.py
===================
SARA platformu - Jetson <-> Arduino Uno (PCA9685/ESC/Servo/SEN0257)
Seri Haberlesme Koprusu

BU DOSYA NEDEN VAR (ekipten gelen "pixhawk_uno_servo_controller.py" ile
farki): Ekibin hazirladigi orijinal dosya, /mavros/imu/data'yi DOGRUDAN
okuyup KENDI basit kontrol kararini uretiyor ve su sensorlerinden KENDI
burun-acma kararini veriyordu - bu, SARA'nin guvenlik zincirini
(guidance -> autopilot -> safety) tamamen BAYPAS ediyordu (60 sn motor
inhibit, acil durdurma, gorev fazi, kavitasyon limiti hicbiri gecerli
olmuyordu).

BILINCLI OLARAK AYNEN KORUNAN KISIMLAR (bunlar zaten calisan, test
edilmis sensor kodlaridir, DEGISTIRILMEDI):
    - KararliDijitalGiris sinifi (su sensoru debounce mantigi)
    - GPIO pin 15/16 okuma sekli, aktif seviyeler
    - Seri baglanti/yeniden baglanma/heartbeat gonderme mantigi
    - Arduino seri komut protokolu (PITCH=/YAW=/NOSE=/MOTOR_*/HB) - HIC
      DEGISTIRILMEDI, Arduino firmware'i (.ino) DOKUNULMADAN kullanilir

DEGISEN TEK SEY - kontrol kararinin KAYNAGI:
    ESKI: /mavros/imu/data -> bu dosyanin kendi P-kontrolcusu -> Arduino
    YENI: safety.py (SARA'nin TEK yetkili karar mercii) -> bu dosya
          (sadece ILETIR, KARAR VERMEZ) -> Arduino

    Dinlenen (safety.py'den, zaten TUM guvenlik kontrollerinden gecmis):
        /sara/control/thrust_command    (Float64 [0,1])
        /sara/control/fin_command       (Vector3, x=pitch y=yaw, [-1,1])
        /sara/control/nose_cap_command  (Bool)
        /sara/control/buoyancy_command  (Float64) - siringa/yuzey servosu (CH5).
            DOGRULANDI (ekipten gelen el yazisi notla): bu servo, gorev
            bitisinde yuzeye cikisi saglayan TEK YONLU surekli-donus
            servosudur. Arduino firmware'i SADECE SURFACE_START/STOP
            destekler, "geri don/dal" komutu YOK - bu yuzden pozitif
            buoyancy_command "yuzeye cik" niyeti olarak yorumlanir,
            tam cift-yonlu derinlik kontrolu bu donanimla YAPILAMAZ
            (bkz. _send_buoyancy_command).

    Su sensoru okuma (GPIO 15/16) burada KALIYOR (calisan kod) ama
    ARTIK KENDI BASINA nose-acma KARARI VERMIYOR - sadece ham veriyi
    /sara/water_detect_1 ve /sara/water_detect_2 (std_msgs/Bool) olarak
    YAYINLIYOR. guidance.py/safety.py bu veriyi KENDI coklu-kosullu
    ateşleme/kapak mantigiyla (orn. "burun disarida + kuyruk icinde")
    degerlendiriyor - nose_cap_command kararini NIHAI olarak safety.py
    veriyor, biz sadece Arduino'ya iletiyoruz.

*** ESC BILINEN SINIRLAMA (Arduino firmware'i DEGISTIRILMEDI) ***
Arduino sadece MOTOR_START (tam guce ramp) / MOTOR_STOP destekliyor -
oransal hiz (orn. "%75 guc") komutu YOK. /sara/control/thrust_command
suzeklu [0,1] araligindaki degeri THRUST_THRESHOLD uzerinden IKILI
(calisiyor/duruyor) sinyale indirgiyoruz. KTR'nin farkli faz hizlari
(0.400/0.895/1.076/0.340 m/s) su anki donanimla FIZIKSEL OLARAK
AYIRT EDILEMIYOR. Ileride firmware'e "SPEED=0.75" gibi bir komut
eklenirse, bu dosyada sadece _send_motor_commands() fonksiyonu
guncellenmesi yeterli olur.

Basinc: Arduino'nun telemetri satirindaki 'pressure_kpa' alanindan
okunuyor (SEN0257, Arduino'nun kendi A0 pinine bagli - eski
pressure_sensor_driver.py'nin ADS1115/Jetson-I2C yolu ARTIK
KULLANILMIYOR, bu donanim topolojisinde gecersiz). Arduino'nun kendi
sifir kalibrasyonu (Z komutu, startup'ta otomatik calisir) GAUGE
(gore) basinc uretir; navigation_node KENDI P0 (yuzey referans)
kalibrasyonunu bagimsiz yaptigi icin, standart atmosfer basinci
(101325 Pa) eklenerek "mutlak basinc" formatina donusturulur - boylece
navigation.py hic degistirilmeden calismaya devam eder.

*** LAUNCH DOSYASI UYARISI ***
Bu koprü artik su VE basinc sensorlerini de Arduino uzerinden okudugu
icin, water_sensor_driver.py / pressure_sensor_driver.py /
sensor_get_data.py / actuator_driver.py ARTIK BU KOPRÜYLE BIRLIKTE
CALISTIRILMAMALIDIR - ayni GPIO pinlerini (15/16) ve/veya
/sara/pressure, /sara/control/*_command topic'lerini CAKISTIRIR.
sara_system.launch.py hardware modunda artik SADECE bu koprü
calistirilir.

KULLANIM:
    ros2 run sara_control arduino_bridge --port /dev/ttyUSB0 --baud 115200
"""

import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float64, Empty
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import FluidPressure
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

import serial
import Jetson.GPIO as GPIO


# ================= AYNEN KORUNDU: calisan su sensoru kodu =================
SENSOR_1_PIN = 15  # burun
SENSOR_2_PIN = 16  # kuyruk
SU_VAR_SEVIYESI_1 = GPIO.LOW
SU_VAR_SEVIYESI_2 = GPIO.LOW
SU_SENSORU_DEBOUNCE_SEC = 0.03


class KararliDijitalGiris:
    """AYNEN KORUNDU (ekipten gelen, calisan/test edilmis debounce kodu)."""

    def __init__(self, pin, aktif_seviye):
        self.pin = pin
        self.aktif_seviye = aktif_seviye
        ilk = GPIO.input(pin) == aktif_seviye
        self.ham = ilk
        self.aday = ilk
        self.kararli = ilk
        self.aday_baslangici = time.monotonic()

    def guncelle(self, simdi):
        self.ham = GPIO.input(self.pin) == self.aktif_seviye
        if self.ham != self.aday:
            self.aday = self.ham
            self.aday_baslangici = simdi
            return False
        if (
            self.aday != self.kararli
            and simdi - self.aday_baslangici >= SU_SENSORU_DEBOUNCE_SEC
        ):
            self.kararli = self.aday
            return True
        return False


def clamp(value, low, high):
    return max(low, min(high, value))


class ArduinoBridgeNode(Node):
    CENTER_DEG = 90.0
    # Arduino/servo mekanik siniri (KANAT_SERVO_MIN/MAX_DERECE = 45..135,
    # Arduino firmware'inden - DEGISTIRILMEDI)
    FIN_MECHANICAL_LIMIT_DEG = 45.0
    # KTR kavitasyon guvenligi (25 derecede risk tespit edildi, 20 derece
    # sinir konuldu) - Arduino DOGRUDAN derece bekledigi icin fraksiyon
    # hesabina GEREK YOK, sinir dogrudan derece cinsinden uygulanir.
    FIN_CAVITATION_LIMIT_DEG = 20.0
    MIN_SERVO_UPDATE_DELTA_DEG = 0.3  # gereksiz seri trafigini azaltmak icin

    COMMAND_RATE_HZ = 20.0
    HEARTBEAT_RATE_HZ = 5.0          # Arduino HEARTBEAT_ZAMAN_ASIMI_MS=1000, boylukla rahat pay
    WATER_POLL_RATE_HZ = 50.0
    RECONNECT_RATE_HZ = 1.0

    THRUST_THRESHOLD = 0.05          # bu esigin altinda MOTOR_STOP sayilir
    COMMAND_TIMEOUT_SEC = 0.5        # safety_node'dan komut kesilirse guvenli varsayilan (0/False)
    ARM_SETTLE_SEC = 3.3             # Arduino ARM_BEKLEME_MS=3000ms + pay

    SURFACE_ATMOSPHERIC_PA = 101325.0  # navigation.py'nin kendi P0 kalibrasyonu icin taban deger

    def __init__(self, port, baud):
        super().__init__('arduino_bridge_node')

        self.serial_lock = threading.Lock()
        self.port = port
        self.baud = baud
        self.connection = None
        self._open_serial_locked()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SENSOR_1_PIN, GPIO.IN)
        GPIO.setup(SENSOR_2_PIN, GPIO.IN)
        self.water_1 = KararliDijitalGiris(SENSOR_1_PIN, SU_VAR_SEVIYESI_1)
        self.water_2 = KararliDijitalGiris(SENSOR_2_PIN, SU_VAR_SEVIYESI_2)

        # ================= Guvenlik zincirinden gelen NIHAI komutlar =================
        self._thrust_command = 0.0
        self._fin_pitch_command = 0.0   # -1..1 normalize, safety.py'den (kavitasyon-oncesi)
        self._fin_yaw_command = 0.0
        self._nose_cap_command = False
        self._buoyancy_command = 0.0    # YENI - siringa/yuzey servosu (CH5)
        self._last_thrust_time = None
        self._last_fin_time = None
        self._last_nose_cap_time = None
        self._last_buoyancy_time = None  # YENI

        # ================= Arduino'ya gonderilen son durum (gereksiz trafik onlemek icin) =================
        self._armed = False
        self._arm_sent_time = None
        self._motor_running = False
        self._last_pitch_sent_deg = None
        self._last_yaw_sent_deg = None
        self._last_nose_sent = None
        self._surface_servo_running = False  # YENI - CH5 durumu

        self._last_telemetry = {}

        self.create_subscription(Float64, '/sara/control/thrust_command', self._on_thrust, 10)
        self.create_subscription(Vector3, '/sara/control/fin_command', self._on_fin, 10)
        self.create_subscription(Bool, '/sara/control/nose_cap_command', self._on_nose_cap, 10)
        # YENI: siringa/yuzey servosu (CH5, Arduino'da YUZEY_SERVO_KANALI).
        # Fotografli notta dogrulandi: "1 siringa -> gorev bitisinde yuzeye
        # cikis icin (servo surekli donecek)". Bu servo TEK YONLU calisir
        # (Arduino firmware'inde sadece SURFACE_START/SURFACE_STOP var,
        # "geri don/dal" komutu YOK) - bu yuzden buoyancy_command'in
        # SADECE "yuzeye cik" (pozitif) niyetini algiliyoruz, tam
        # cift-yonlu derinlik kontrolu bu donanimla desteklenmiyor
        # (bkz. _send_buoyancy_command yorumu).
        self.create_subscription(Float64, '/sara/control/buoyancy_command', self._on_buoyancy, 10)
        # YENI: navigation.py'nin ZATEN VAR OLAN /sara/navigation/recalibrate
        # topic'ine (kendi P0/yuzey basinci kalibrasyonu icin) BU KOPRÜ DE
        # abone olur - boylece TEK bir "recalibrate" komutu hem yazilim
        # tarafindaki (navigation.py) hem donanim tarafindaki (Arduino'nun
        # kendi 'Z' sifir kalibrasyonu) referans noktasini BIRLIKTE
        # sifirlar. Mevcut mekanizmaya EKLENDI, yeni bir topic ICAT
        # EDILMEDI - amac takimin tek bir komutla ikisini de tetikleyebilmesi.
        self.create_subscription(Empty, '/sara/navigation/recalibrate', self._on_recalibrate, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._water1_pub = self.create_publisher(Bool, '/sara/water_detect_1', sensor_qos)
        self._water2_pub = self.create_publisher(Bool, '/sara/water_detect_2', sensor_qos)
        self._pressure_pub = self.create_publisher(FluidPressure, '/sara/pressure', sensor_qos)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/actuator/status', 10)

        self.create_timer(1.0 / self.COMMAND_RATE_HZ, self._send_commands)
        self.create_timer(1.0 / self.HEARTBEAT_RATE_HZ, self._send_heartbeat)
        self.create_timer(1.0 / self.COMMAND_RATE_HZ, self._read_serial)
        self.create_timer(1.0 / self.WATER_POLL_RATE_HZ, self._update_water_sensors)
        self.create_timer(1.0 / self.RECONNECT_RATE_HZ, self._reconnect_serial)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'arduino_bridge_node baslatildi: {port} @ {baud}. '
            'safety_node komutlarini (thrust/fin/nose_cap) Arduino\'ya iletir, '
            'su/basinc sensorlerini ROS\'a yayinlar. KENDI karar VERMEZ.'
        )

    # ======================================================================
    # Seri baglanti (AYNEN KORUNAN mantik, sadece isim/log Turkce netlestirildi)
    # ======================================================================
    def _open_serial_locked(self):
        try:
            self.connection = serial.Serial(self.port, self.baud, timeout=0.0)
            time.sleep(2.0)  # Arduino USB acilinca yeniden baslar
            self.connection.reset_input_buffer()
            self._armed = False
            self._motor_running = False
            self._last_pitch_sent_deg = None
            self._last_yaw_sent_deg = None
            self._last_nose_sent = None
            self._surface_servo_running = False
        except (serial.SerialException, OSError) as hata:
            self.get_logger().error(f'Arduino seri baglantisi acilamadi: {hata}')
            self.connection = None

    def _serial_write(self, command: str) -> bool:
        with self.serial_lock:
            if self.connection is None or not self.connection.is_open:
                return False
            try:
                self.connection.write((command + '\n').encode('ascii'))
                self.connection.flush()
                return True
            except serial.SerialException as hata:
                self.get_logger().error(f'Arduino seri yazma hatasi: {hata}')
                self._disconnect_serial_locked()
                return False

    def _disconnect_serial_locked(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except serial.SerialException:
                pass
        self.connection = None

    def _reconnect_serial(self):
        if self.connection is not None and self.connection.is_open:
            return
        try:
            yeni = serial.Serial(self.port, self.baud, timeout=0.0)
            time.sleep(2.0)
            yeni.reset_input_buffer()
            with self.serial_lock:
                self.connection = yeni
            self._armed = False
            self._motor_running = False
            self._last_pitch_sent_deg = None
            self._last_yaw_sent_deg = None
            self._last_nose_sent = None
            self._surface_servo_running = False
            self.get_logger().info(f'Arduino seri baglantisi yeniden kuruldu: {self.port}')
        except (serial.SerialException, OSError) as hata:
            self.get_logger().warning(f'Arduino yeniden baglanamadi: {hata}')

    def _read_serial(self):
        with self.serial_lock:
            if self.connection is None or not self.connection.is_open:
                return
            try:
                while self.connection.in_waiting:
                    line = self.connection.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                    if line.startswith('ERR,'):
                        self.get_logger().warning(f'Arduino: {line}')
                        continue
                    if line.startswith('OK,') or line.startswith('INFO,') or line.startswith('READY'):
                        self.get_logger().info(f'Arduino: {line}')
                        continue
                    self._parse_and_publish_telemetry(line)
            except (serial.SerialException, OSError) as hata:
                self.get_logger().error(f'Arduino seri okuma hatasi: {hata}')
                self._disconnect_serial_locked()

    def _parse_and_publish_telemetry(self, line: str):
        fields = {}
        for part in line.split(','):
            if '=' not in part:
                return
            key, _, value = part.partition('=')
            try:
                fields[key.strip()] = float(value.strip())
            except ValueError:
                return
        if 'pressure_kpa' not in fields:
            return
        self._last_telemetry = fields

        # Arduino'nun kendi sifir-kalibreli (gauge) basincina standart
        # atmosfer basinci eklenerek "mutlak basinc" formatina getirilir -
        # navigation.py boylece hic degismeden kendi P0 kalibrasyonunu yapar.
        pressure_pa = self.SURFACE_ATMOSPHERIC_PA + fields['pressure_kpa'] * 1000.0
        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sara_pressure_sensor_arduino'
        msg.fluid_pressure = pressure_pa
        msg.variance = 0.0
        self._pressure_pub.publish(msg)

    # ======================================================================
    # Guvenlik zincirinden gelen komutlar
    # ======================================================================
    def _on_thrust(self, msg: Float64):
        self._thrust_command = msg.data
        self._last_thrust_time = self.get_clock().now()

    def _on_fin(self, msg: Vector3):
        self._fin_pitch_command = msg.x
        self._fin_yaw_command = msg.y
        self._last_fin_time = self.get_clock().now()

    def _on_nose_cap(self, msg: Bool):
        self._nose_cap_command = msg.data
        self._last_nose_cap_time = self.get_clock().now()

    def _on_buoyancy(self, msg: Float64):
        self._buoyancy_command = msg.data
        self._last_buoyancy_time = self.get_clock().now()

    def _on_recalibrate(self, _msg: Empty):
        # GUVENLIK: Arduino'nun 'Z' komutu ~3 saniye BLOKE EDER (delay(3000)
        # + 300 ornek ortalama) - bu sirada heartbeat isleyemez. Motor
        # calisiyorken tetiklenirse Arduino'nun HEARTBEAT_LOST guvenlik
        # kilidine (motoruHemenNotreAl(MOTOR_FAULT)) yol acabilir. Bu yuzden
        # sadece motor calismiyorken Arduino'ya iletilir; aksi halde
        # reddedilir ve nedeni loglanir (navigation.py KENDI P0 kalibrasyonunu
        # yine de yapar - sadece Arduino'nun donanimsal sifir noktasi
        # guncellenmemis olur).
        if self._motor_running:
            self.get_logger().warning(
                'Yeniden kalibrasyon istegi alindi ama motor CALISIYOR - '
                "Arduino'nun 'Z' komutu (3 sn bloke eder) GUVENLIK ICIN GONDERILMEDI. "
                'Motor durunca tekrar deneyin.'
            )
            return
        self._serial_write('Z')
        self.get_logger().info("Yeniden kalibrasyon istegi -> Arduino'ya 'Z' (basinc sifir kalibrasyonu) gonderildi.")

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < self.COMMAND_TIMEOUT_SEC

    # ======================================================================
    # Arduino'ya komut gonderme - SADECE ILETIR, KARAR VERMEZ
    # ======================================================================
    def _send_commands(self):
        self._send_motor_command()
        self._send_fin_commands()
        self._send_nose_cap_command()
        self._send_buoyancy_command()

    def _send_buoyancy_command(self):
        """Siringa/yuzey servosu (CH5) - fotografli notta dogrulandi:
        "gorev bitisinde yuzeye cikis icin, servo surekli donecek".

        *** BILINEN SINIRLAMA (Arduino firmware'i DEGISTIRILMEDI) ***
        Bu servo TEK YONLU calisir - Arduino sadece SURFACE_START (sabit
        yonde surekli don) / SURFACE_STOP destekliyor, "geri don/dal"
        komutu YOK. Bu yuzden autopilot.py'nin urettigi surekli
        buoyancy_command degeri (potansiyel olarak dalis icin negatif,
        cikis icin pozitif) burada TAM cift-yonlu derinlik kontrolu
        olarak KULLANILAMIYOR - sadece "yuzeye cikmak istiyoruz mu"
        (pozitif deger) niyeti algilanip SURFACE_START/STOP'a
        cevriliyor. Dalis, aracin agirlik/trim dengesiyle VE pitch/fin
        kontroluyle saglaniyor (guidance.py'nin DALIS/G2_ACILI_DALIS
        fazlarinda oldugu gibi), bu servo SADECE surfacing'e yardimci
        oluyor - THRUST_THRESHOLD ile ayni mantik, farkli fiziksel
        mekanizma."""
        buoyancy = self._buoyancy_command if self._fresh(self._last_buoyancy_time) else 0.0
        want_surface = buoyancy > self.THRUST_THRESHOLD

        if want_surface and not self._surface_servo_running:
            self._serial_write('SURFACE_START')
            self._surface_servo_running = True
        elif not want_surface and self._surface_servo_running:
            self._serial_write('SURFACE_STOP')
            self._surface_servo_running = False

    def _send_motor_command(self):
        """*** ESC BILINEN SINIRLAMA *** - bkz. modul dokstringi. Arduino
        sadece START(tam guc)/STOP destekler, oransal hiz YOK. thrust_command
        [0,1] degeri THRUST_THRESHOLD uzerinden ikili sinyale indirgenir."""
        now = time.monotonic()
        thrust = self._thrust_command if self._fresh(self._last_thrust_time) else 0.0
        thrust_active = thrust > self.THRUST_THRESHOLD

        if not self._armed:
            if thrust_active:
                self._serial_write('MOTOR_ARM')
                self._armed = True
                self._arm_sent_time = now
                self.get_logger().info('Ilk thrust_command alindi - MOTOR_ARM gonderildi (3 sn bekleniyor).')
            return

        arm_settled = (self._arm_sent_time is not None) and (now - self._arm_sent_time >= self.ARM_SETTLE_SEC)
        if thrust_active and arm_settled and not self._motor_running:
            self._serial_write('MOTOR_START')
            self._motor_running = True
        elif not thrust_active and self._motor_running:
            self._serial_write('MOTOR_STOP')
            self._motor_running = False

    def _send_fin_commands(self):
        fin_fresh = self._fresh(self._last_fin_time)
        pitch_norm = clamp(self._fin_pitch_command, -1.0, 1.0) if fin_fresh else 0.0
        yaw_norm = clamp(self._fin_yaw_command, -1.0, 1.0) if fin_fresh else 0.0

        # KAVITASYON GUVENLIGI: normalize komut once mekanik sinira (45 derece)
        # olceklenir, SONRA kavitasyon sinirina (20 derece) kirpilir.
        pitch_deg_offset = clamp(
            pitch_norm * self.FIN_MECHANICAL_LIMIT_DEG,
            -self.FIN_CAVITATION_LIMIT_DEG, self.FIN_CAVITATION_LIMIT_DEG,
        )
        yaw_deg_offset = clamp(
            yaw_norm * self.FIN_MECHANICAL_LIMIT_DEG,
            -self.FIN_CAVITATION_LIMIT_DEG, self.FIN_CAVITATION_LIMIT_DEG,
        )
        pitch_deg = self.CENTER_DEG + pitch_deg_offset
        yaw_deg = self.CENTER_DEG + yaw_deg_offset

        if self._last_pitch_sent_deg is None or abs(pitch_deg - self._last_pitch_sent_deg) > self.MIN_SERVO_UPDATE_DELTA_DEG:
            self._serial_write(f'PITCH={pitch_deg:.1f}')
            self._last_pitch_sent_deg = pitch_deg
        if self._last_yaw_sent_deg is None or abs(yaw_deg - self._last_yaw_sent_deg) > self.MIN_SERVO_UPDATE_DELTA_DEG:
            self._serial_write(f'YAW={yaw_deg:.1f}')
            self._last_yaw_sent_deg = yaw_deg

    def _send_nose_cap_command(self):
        nose_fresh = self._fresh(self._last_nose_cap_time)
        nose_open = self._nose_cap_command if nose_fresh else False
        if self._last_nose_sent != nose_open:
            self._serial_write('NOSE=90.0' if nose_open else 'NOSE=0.0')
            self._last_nose_sent = nose_open
            self.get_logger().info(f"nose_cap_command -> {'ACIK' if nose_open else 'KAPALI'} (safety.py onayli)")

    def _send_heartbeat(self):
        self._serial_write('HB')

    # ======================================================================
    # Su sensoru okuma (AYNEN KORUNAN kod) - SADECE YAYINLAR, KARAR VERMEZ
    # ======================================================================
    def _update_water_sensors(self):
        now = time.monotonic()
        self.water_1.guncelle(now)
        self.water_2.guncelle(now)

        w1 = Bool()
        w1.data = bool(self.water_1.kararli)  # True = burunda su VAR (batik)
        self._water1_pub.publish(w1)

        w2 = Bool()
        w2.data = bool(self.water_2.kararli)  # True = kuyrukta su VAR (batik)
        self._water2_pub.publish(w2)

    # ======================================================================
    # Tanilama
    # ======================================================================
    def _publish_status(self):
        status = DiagnosticStatus()
        status.name = 'sara_arduino_bridge'
        status.hardware_id = 'arduino_uno'
        connected = self.connection is not None and self.connection.is_open
        status.level = DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR
        status.message = 'Nominal' if connected else 'Arduino seri baglantisi yok'
        status.values = [
            KeyValue(key='connected', value=str(connected)),
            KeyValue(key='armed', value=str(self._armed)),
            KeyValue(key='motor_running', value=str(self._motor_running)),
            KeyValue(key='pitch_deg_sent', value=str(self._last_pitch_sent_deg)),
            KeyValue(key='yaw_deg_sent', value=str(self._last_yaw_sent_deg)),
            KeyValue(key='nose_cap_sent', value=str(self._last_nose_sent)),
            KeyValue(key='surface_servo_running', value=str(self._surface_servo_running)),  # YENI
            KeyValue(key='water_1_kararli', value=str(self.water_1.kararli)),
            KeyValue(key='water_2_kararli', value=str(self.water_2.kararli)),
            KeyValue(key='esc_us_arduino', value=str(self._last_telemetry.get('esc_us', '-'))),
            KeyValue(key='motor_state_arduino', value=str(self._last_telemetry.get('motor_state', '-'))),
            KeyValue(key='proportional_thrust_supported', value='False'),
            KeyValue(key='bidirectional_buoyancy_supported', value='False'),  # YENI
        ]
        self._status_pub.publish(status)

    def close(self):
        try:
            self._serial_write('NOSE=0.0')
            self._serial_write('SURFACE_STOP')  # YENI - siringa servosunu da guvenli durdur
            self._serial_write('SERVO_CENTER')
            self._serial_write('MOTOR_STOP')
            time.sleep(0.15)
            self._serial_write('MOTOR_DISARM')
            time.sleep(0.1)
        finally:
            with self.serial_lock:
                self._disconnect_serial_locked()
            GPIO.cleanup()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    arguments, ros_arguments = parser.parse_known_args()

    rclpy.init(args=ros_arguments)
    node = None
    try:
        node = ArduinoBridgeNode(arguments.port, arguments.baud)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()