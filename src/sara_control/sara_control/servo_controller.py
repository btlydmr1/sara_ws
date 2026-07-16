#!/usr/bin/env python3
"""
SARA servo kontrolcüsü - Jetson + PCA9685 (Blinka)

v6: Her servo için BİRDEN FAZLA eksen toplanabilir.
  - Servo 1 <- pitch
  - Servo 2 <- roll + yaw  (yana eğme VE yatayda dönme, ikisi de aynı tepki)
  - Servo 3 <- buton (Pixhawk'tan bağımsız)

Buton: Blinka digitalio (board.D24)
Donanım: set_pulse_width_range(500, 2500) -> DS3225
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor#!/usr/bin/env python3
"""
SARA servo kontrolcüsü - Jetson + PCA9685 (Blinka)

v4 değişikliği: Buton AYRI THREAD'de, kesintisiz okunur (servo işi bloklamaz).
  - Buton okuma 100 Hz, kendi callback group'unda -> basış kaçmaz
  - MultiThreadedExecutor ile servo ve buton paralel çalışır
  - I2C kilidi: servo yazmaları (1,2 ve 3) çakışmaz

Donanım (çalışan koddan):
  - set_pulse_width_range(500, 2500)  -> DS3225 doğru pulse
  - Buton: Blinka digitalio (board.D4 = Jetson fiziksel pin 7)

Kontrol:
  - Servo 1,2: Pixhawk yönelimine göre (orantılı, ters yön, ±MAX_ANGLE)
  - Servo 3: butonla 90° <-> 0° (Pixhawk'tan bağımsız)
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Imu

import board
import digitalio
from adafruit_servokit import ServoKit


# ----- DONANIM -----
kit = ServoKit(channels=16)
SERVO_1 = 0
SERVO_2 = 1
SERVO_3 = 2

kit.servo[SERVO_1].set_pulse_width_range(500, 2500)
kit.servo[SERVO_2].set_pulse_width_range(500, 2500)
kit.servo[SERVO_3].set_pulse_width_range(500, 2500)

# Buton pini: Jetson fiziksel pin 7 = board.D4 (çalışan kodla aynı)
# Eğer butonu BAŞKA pine bağladıysan, board.D4'ü ona göre değiştir.
BUTTON_PIN = board.D24
button = digitalio.DigitalInOut(BUTTON_PIN)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP

# ----- KONTROL AYARLARI -----
AXIS_SERVO1 = 'pitch'
AXIS_SERVO2 = 'roll'     # az çalışıyorsa 'roll' yap
SIGN_SERVO1 = -1.0
SIGN_SERVO2 = -1.0
GAIN = 1.0
MAX_ANGLE = 75.0        # 90±75 = 15..165
CENTER = 90

SERVO3_CLOSED = 0
SERVO3_OPEN = 90

SERVO_RATE = 50.0
BUTTON_RATE = 100.0     # buton çok sık okunsun (10ms) -> basış kaçmaz


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

        # I2C kilidi: servo yazmaları aynı anda olmasın (iki thread'den)
        self.i2c_lock = threading.Lock()

        # Başlangıç konumları
        with self.i2c_lock:
            kit.servo[SERVO_1].angle = CENTER
            kit.servo[SERVO_2].angle = CENTER
            kit.servo[SERVO_3].angle = SERVO3_CLOSED

        self.servo3_state = False
        self.last_button = True

        self.ref = None
        self.s1 = 0.0
        self.s2 = 0.0
        self.have_data = False

        # Buton kendi callback group'unda -> servo işi onu bloklamaz
        self.button_group = MutuallyExclusiveCallbackGroup()
        self.servo_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_cb, qos_profile_sensor_data,
            callback_group=self.servo_group)

        self.timer_servo = self.create_timer(
            1.0 / SERVO_RATE, self.drive_servos,
            callback_group=self.servo_group)

        self.timer_button = self.create_timer(
            1.0 / BUTTON_RATE, self.check_button,
            callback_group=self.button_group)

        self.get_logger().info(
            f"servo_controller başladı: Servo1<-{AXIS_SERVO1}, Servo2<-{AXIS_SERVO2}, "
            f"Servo3=buton. buton {BUTTON_RATE:.0f}Hz ayrı thread, max ±{MAX_ANGLE}°")

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
                f"Referans: roll={roll_deg:.0f}° pitch={pitch_deg:.0f}° yaw={yaw_deg:.0f}°")
            return

        d_roll = angle_diff_deg(roll_deg, self.ref[0])
        d_pitch = pitch_deg - self.ref[1]
        d_yaw = angle_diff_deg(yaw_deg, self.ref[2])
        angles = {'roll': d_roll, 'pitch': d_pitch, 'yaw': d_yaw}

        self.s1 = clamp(SIGN_SERVO1 * GAIN * angles[AXIS_SERVO1], -MAX_ANGLE, MAX_ANGLE)
        self.s2 = clamp(SIGN_SERVO2 * GAIN * angles[AXIS_SERVO2], -MAX_ANGLE, MAX_ANGLE)
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
                kit.servo[SERVO_1].angle = clamp(CENTER + self.s1, 0, 180)
                kit.servo[SERVO_2].angle = clamp(CENTER + self.s2, 0, 180)
        except Exception as e:
            self.get_logger().warn(f"Servo yazma hatası: {e}")

    def check_button(self):
        # button.value -> GPIO okuması (hızlı, bloklamaz)
        current = button.value
        # TEŞHİS: buton durumu her değiştiğinde terminale yaz
        if current != self.last_button:
            self.get_logger().info(
                "BUTON: BASILDI (LOW)" if current is False else "BUTON: bırakıldı (HIGH)")
        if self.last_button is True and current is False:
            self.servo3_state = not self.servo3_state
            try:
                with self.i2c_lock:
                    if self.servo3_state:
                        kit.servo[SERVO_3].angle = SERVO3_OPEN
                    else:
                        kit.servo[SERVO_3].angle = SERVO3_CLOSED
                self.get_logger().info(
                    "3. servo AÇIK (90°)" if self.servo3_state else "3. servo KAPALI (0°)")
            except Exception as e:
                self.get_logger().warn(f"Servo3 yazma hatası: {e}")
        self.last_button = current


def main():
    rclpy.init()
    node = ServoController()
    # MultiThreadedExecutor: buton ve servo paralel çalışsın
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    try:
        with node.i2c_lock:
            kit.servo[SERVO_1].angle = CENTER
            kit.servo[SERVO_2].angle = CENTER
            kit.servo[SERVO_3].angle = SERVO3_CLOSED
        button.deinit()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Imu

import board
import digitalio
from adafruit_servokit import ServoKit


# ----- DONANIM -----
kit = ServoKit(channels=16)
SERVO_1 = 0
SERVO_2 = 1
SERVO_3 = 2

kit.servo[SERVO_1].set_pulse_width_range(500, 2500)
kit.servo[SERVO_2].set_pulse_width_range(500, 2500)
kit.servo[SERVO_3].set_pulse_width_range(500, 2500)

# Buton pini (çalışan değer: board.D24)
BUTTON_PIN = board.D24
button = digitalio.DigitalInOut(BUTTON_PIN)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP

# ----- KONTROL AYARLARI -----
# Her servo hangi eksen(ler)e tepki versin? Liste -> o eksenlerin TOPLAMI.
# Seçenekler: 'roll', 'pitch', 'yaw'
AXES_SERVO1 = ['pitch']          # Servo 1: ileri-geri
AXES_SERVO2 = ['roll', 'yaw']    # Servo 2: yana eğme (roll) + yatayda dönme (yaw)

# Ters dönüyorsa işareti çevir (+1 / -1)
SIGN_SERVO1 = -1.0
SIGN_SERVO2 = -1.0

GAIN = 1.0              # eksen açısı(derece) -> servo açısı oranı
MAX_ANGLE = 75.0        # 90±75 = 15..165
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

        self.i2c_lock = threading.Lock()

        with self.i2c_lock:
            kit.servo[SERVO_1].angle = CENTER
            kit.servo[SERVO_2].angle = CENTER
            kit.servo[SERVO_3].angle = SERVO3_CLOSED

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
            f"servo_controller başladı: Servo1<-{'+'.join(AXES_SERVO1)}, "
            f"Servo2<-{'+'.join(AXES_SERVO2)}, Servo3=buton. max ±{MAX_ANGLE}°")

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
                f"Referans: roll={roll_deg:.0f}° pitch={pitch_deg:.0f}° yaw={yaw_deg:.0f}°")
            return

        d_roll = angle_diff_deg(roll_deg, self.ref[0])
        d_pitch = pitch_deg - self.ref[1]
        d_yaw = angle_diff_deg(yaw_deg, self.ref[2])
        angles = {'roll': d_roll, 'pitch': d_pitch, 'yaw': d_yaw}

        # Servo açısı = seçili eksenlerin TOPLAMI (örn. Servo2 = roll + yaw)
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
                kit.servo[SERVO_1].angle = clamp(CENTER + self.s1, 0, 180)
                kit.servo[SERVO_2].angle = clamp(CENTER + self.s2, 0, 180)
        except Exception as e:
            self.get_logger().warn(f"Servo yazma hatası: {e}")

    def check_button(self):
        current = button.value
        if current != self.last_button:
            self.get_logger().info(
                "BUTON: BASILDI" if current is False else "BUTON: bırakıldı")
        if self.last_button is True and current is False:
            self.servo3_state = not self.servo3_state
            try:
                with self.i2c_lock:
                    kit.servo[SERVO_3].angle = SERVO3_OPEN if self.servo3_state else SERVO3_CLOSED
                self.get_logger().info(
                    "3. servo AÇIK (90°)" if self.servo3_state else "3. servo KAPALI (0°)")
            except Exception as e:
                self.get_logger().warn(f"Servo3 yazma hatası: {e}")
        self.last_button = current


def main():
    rclpy.init()
    node = ServoController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    try:
        with node.i2c_lock:
            kit.servo[SERVO_1].angle = CENTER
            kit.servo[SERVO_2].angle = CENTER
            kit.servo[SERVO_3].angle = SERVO3_CLOSED
        button.deinit()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()