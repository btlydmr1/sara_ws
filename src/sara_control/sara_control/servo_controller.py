#!/usr/bin/env python3
"""
servo_controller.py
=====================
*** UYARI - SADECE IZOLE I2C/SERVO BENCH TESTI ICINDIR ***
*** UCUS/YARISMA KODUNDA KESINLIKLE KULLANILMAMALIDIR      ***

DUZELTME (kapsamli denetim): Bu dosya iki bagimsiz sorun icin duzenlendi:

  1) DOSYA ICI KOPYALAMA HATASI: Yuklenen orijinal dosyada AYNI modulun
     (docstring, importlar, sabitler, ServoController sinifi, main())
     ICERIGI YANLISLIKLA IKI KEZ art arda yapistirilmisti (muhtemelen bir
     duzenleme/kopyalama kazasi). Python bunu teknik olarak calistirabilir
     (ikinci tanimlar birincileri sessizce golgeler) ama bu, bakim ve
     denetim acisindan ciddi bir risktir - hangi surumun gercekten
     calistigi belirsizlesir. Bu dosya TEK, TEMIZ bir kopya olarak
     yeniden yazilmistir (orijindeki DAHA GENEL/esnek "v2" mantigi -
     AXES_SERVO1/2 liste tabanli eksen toplama - esas alinmistir).

  2) GUVENLIK ZINCIRI BAYPASI: Bu dosya /mavros/imu/data'yi DOGRUDAN
     okuyup PCA9685 servolarini kendi ic mantigiyla suruyordu -
     safety_node'un urettigi /sara/control/fin_command'i HIC DINLEMEDEN.
     Bu, projedeki TEK YETKILI karar mercii ilkesini (bkz. safety.py
     dokstringi) ihlal eder ve su guvenlik katmanlarinin HICBIRINE tabi
     DEGILDIR:
       - Acil durdurma (/sara/safety/emergency_stop)
       - 60 sn motor inhibit / hareket izni (/sara/mission_start/motion_permission)
       - 20 derece kavitasyon guvenlik limiti (actuator_driver.py'de
         uygulanan fin_max_deflection_deg klempi)
       - Gorev fazi / gudum mantigi (guidance.py)
     Ayrica actuator_driver.py (PCA9685 uzerinden AYNI fiziksel
     kanatciklari/servolari, ama GUVENLIK KATMANINDAN gelen komutlarla
     suren "resmi" surucu) ile CAKISIR - ikisi ayni anda calistirilirsa
     hem I2C yazma catismasi hem de ongorulemez/guvensiz davranis olusur.

     DUZELTME: Bu dosya artik KOD ICINDE ACIKCA KILITLENMISTIR - asagidaki
     `--ros-args -p i_understand_this_bypasses_safety:=true` parametresi
     ACIKCA verilmeden node hicbir GPIO/I2C/PCA9685 baglantisi ACMADAN
     hemen kapanir. Bu, "yanlislikla / farkinda olmadan" calistirilmasini
     engellemek icin bilincli bir guvenlik kilidi olup, launch dosyasina
     (sara_system.launch.py) DA BILEREK DAHIL EDILMEMISTIR.

     Bu dosyayi SADECE su durumda, ELLE calistirin: tezgah uzerinde,
     su altina INMEDEN, sadece servo/PCA9685/I2C kablolamasini gozle
     dogrulamak icin izole bir bench test yaparken.

KULLANIM (bilerek, sadece bench test icin):
    ros2 run sara_control servo_controller --ros-args \
        -p i_understand_this_bypasses_safety:=true

Donanim (calisan koddan):
  - set_pulse_width_range(500, 2500)  -> DS3225 dogru pulse
  - Buton: Blinka digitalio (board.D24)

Kontrol (SADECE bench test amacli, ROS2 guvenlik topic'lerinden bagimsiz):
  - Servo 1,2: Pixhawk yonelimine gore (orantili, ters yon, +-MAX_ANGLE)
  - Servo 3: butonla 90 <-> 0 derece (Pixhawk'tan bagimsiz)
"""

import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Imu

try:
    import board
    import digitalio
    from adafruit_servokit import ServoKit
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


# ----- KONTROL AYARLARI -----
SERVO_1 = 0
SERVO_2 = 1
SERVO_3 = 2

# Her servo hangi eksen(ler)e tepki versin? Liste -> o eksenlerin TOPLAMI.
# Secenekler: 'roll', 'pitch', 'yaw'
AXES_SERVO1 = ['pitch']          # Servo 1: ileri-geri
AXES_SERVO2 = ['roll', 'yaw']    # Servo 2: yana egme (roll) + yatayda donme (yaw)

# Ters donuyorsa isareti cevir (+1 / -1)
SIGN_SERVO1 = -1.0
SIGN_SERVO2 = -1.0

GAIN = 1.0              # eksen acisi(derece) -> servo acisi orani
MAX_ANGLE = 75.0         # 90+-75 = 15..165 (BENCH TEST icin mekanik sinir -
                          # UCUS kodunda gercek kavitasyon limiti (20 derece)
                          # actuator_driver.py'de AYRI olarak uygulanir)
CENTER = 90

SERVO3_CLOSED = 0
SERVO3_OPEN = 90

SERVO_RATE = 50.0
BUTTON_RATE = 100.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_diff_deg(a, b):
    d = a - b
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # ================= GUVENLIK KILIDI (bkz. dosya basligi) =================
        self.declare_parameter('i_understand_this_bypasses_safety', False)
        confirmed = bool(self.get_parameter('i_understand_this_bypasses_safety').value)
        if not confirmed:
            self.get_logger().fatal(
                'servo_controller.py, TUM guvenlik zincirini (acil durdurma, 60sn '
                'motor inhibit, 20 derece kavitasyon limiti, gudum) baypas eden bir '
                'BENCH TEST aracidir ve UCUS/YARISMA KODUNDA KULLANILMAMALIDIR. '
                'Bilerek ve SADECE izole bir tezgah testi icin calistirmak istiyorsaniz: '
                "'ros2 run sara_control servo_controller --ros-args "
                "-p i_understand_this_bypasses_safety:=true'"
            )
            raise SystemExit(1)

        self.get_logger().warn(
            'servo_controller BENCH TEST modunda baslatiliyor - guvenlik zincirinin '
            'DISINDA calisiyor. Suya INMEYIN.'
        )

        if not HARDWARE_AVAILABLE:
            self.get_logger().fatal(
                'board / digitalio / adafruit_servokit bulunamadi. Bu node yalnizca '
                'gercek Jetson donanim uzerinde calisir.'
            )
            raise SystemExit(1)

        # ----- DONANIM -----
        self.kit = ServoKit(channels=16)
        self.kit.servo[SERVO_1].set_pulse_width_range(500, 2500)
        self.kit.servo[SERVO_2].set_pulse_width_range(500, 2500)
        self.kit.servo[SERVO_3].set_pulse_width_range(500, 2500)

        self.button_pin = board.D24
        self.button = digitalio.DigitalInOut(self.button_pin)
        self.button.direction = digitalio.Direction.INPUT
        self.button.pull = digitalio.Pull.UP

        self.i2c_lock = threading.Lock()

        with self.i2c_lock:
            self.kit.servo[SERVO_1].angle = CENTER
            self.kit.servo[SERVO_2].angle = CENTER
            self.kit.servo[SERVO_3].angle = SERVO3_CLOSED

        self.servo3_state = False
        self.last_button = True

        self.ref = None
        self.s1 = 0.0
        self.s2 = 0.0
        self.have_data = False

        self.button_group = MutuallyExclusiveCallbackGroup()
        self.servo_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_cb, qos_profile_sensor_data,
            callback_group=self.servo_group)
        self.timer_servo = self.create_timer(
            1.0 / SERVO_RATE, self.drive_servos, callback_group=self.servo_group)
        self.timer_button = self.create_timer(
            1.0 / BUTTON_RATE, self.check_button, callback_group=self.button_group)

        self.get_logger().info(
            f"servo_controller (BENCH TEST) basladi: Servo1<-{'+'.join(AXES_SERVO1)}, "
            f"Servo2<-{'+'.join(AXES_SERVO2)}, Servo3=buton. max +-{MAX_ANGLE} derece")

    def imu_cb(self, msg):
        q = msg.orientation

        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                           1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        sinp = clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
        pitch = math.asin(sinp)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        if self.ref is None:
            self.ref = (roll_deg, pitch_deg, yaw_deg)
            self.get_logger().info(
                f"Referans: roll={roll_deg:.0f} pitch={pitch_deg:.0f} yaw={yaw_deg:.0f}")
            return

        d_roll = angle_diff_deg(roll_deg, self.ref[0])
        d_pitch = pitch_deg - self.ref[1]
        d_yaw = angle_diff_deg(yaw_deg, self.ref[2])
        angles = {'roll': d_roll, 'pitch': d_pitch, 'yaw': d_yaw}

        # Servo acisi = secili eksenlerin TOPLAMI (orn. Servo2 = roll + yaw)
        toplam1 = sum(angles[e] for e in AXES_SERVO1)
        toplam2 = sum(angles[e] for e in AXES_SERVO2)
        self.s1 = clamp(SIGN_SERVO1 * GAIN * toplam1, -MAX_ANGLE, MAX_ANGLE)
        self.s2 = clamp(SIGN_SERVO2 * GAIN * toplam2, -MAX_ANGLE, MAX_ANGLE)
        self.have_data = True

        self.get_logger().info(
            f"roll={d_roll:6.1f}  pitch={d_pitch:6.1f}  yaw={d_yaw:6.1f}  "
            f"|  S1={self.s1:6.1f}  S2={self.s2:6.1f}",
            throttle_duration_sec=0.3)

    def drive_servos(self):
        if not self.have_data:
            return
        try:
            with self.i2c_lock:
                self.kit.servo[SERVO_1].angle = clamp(CENTER + self.s1, 0, 180)
                self.kit.servo[SERVO_2].angle = clamp(CENTER + self.s2, 0, 180)
        except Exception as e:
            self.get_logger().warn(f"Servo yazma hatasi: {e}")

    def check_button(self):
        current = self.button.value
        if current != self.last_button:
            self.get_logger().info(
                "BUTON: BASILDI" if current is False else "BUTON: birakildi")
        if self.last_button is True and current is False:
            self.servo3_state = not self.servo3_state
            try:
                with self.i2c_lock:
                    self.kit.servo[SERVO_3].angle = SERVO3_OPEN if self.servo3_state else SERVO3_CLOSED
                self.get_logger().info(
                    "3. servo ACIK (90)" if self.servo3_state else "3. servo KAPALI (0)")
            except Exception as e:
                self.get_logger().warn(f"Servo3 yazma hatasi: {e}")
        self.last_button = current

    def safe_shutdown(self):
        try:
            with self.i2c_lock:
                self.kit.servo[SERVO_1].angle = CENTER
                self.kit.servo[SERVO_2].angle = CENTER
                self.kit.servo[SERVO_3].angle = SERVO3_CLOSED
            self.button.deinit()
        except Exception:
            pass


def main():
    rclpy.init()
    try:
        node = ServoController()
    except SystemExit:
        rclpy.shutdown()
        sys.exit(1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.safe_shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()