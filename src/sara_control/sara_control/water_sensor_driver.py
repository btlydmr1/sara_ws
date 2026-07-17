#!/usr/bin/env python3
"""
water_sensor_driver.py
========================
SARA platformu - SEN0368 Su Sensoru Surucusu (GERCEK DONANIM)

Bagli donanim (kullanicidan alinan gercek kablolama):
    SEN0368-1 (BURUN sensoru):
        Besleme -> regule 5V, GND -> ortak GND
        IO2 -> seviye donusturucu -> Jetson pin 15
    SEN0368-2 (KUYRUK sensoru):
        Besleme -> regule 5V, GND -> ortak GND
        IO2 -> seviye donusturucu -> Jetson pin 16

Bu node, guidance.py/safety.py'nin bekledigi /sara/water_detect_1 (BURUN)
ve /sara/water_detect_2 (KUYRUK) topic'lerine, vehicle_sim.py'nin test
modunda kullandigi TAM AYNI sozlesmeyle (True=su algilandi/batik,
False=su yok/disarida) yayin yapar.

*** POLARITE DOGRULAMASI GEREKLI ***
SEN0368'in "su algilandi" durumda IO2 ciktisinin HIGH mi LOW mu oldugu
kullanilan seviye donusturucu tipine bagli olarak DEGISEBILIR. Ilk
kurulumda 'active_high' parametrelerini sensoru fiilen suya batirip
cikararak DOGRULAYIN, varsayilan degerler tahminidir.

Cikti:
    /sara/water_detect_1 (std_msgs/Bool) - BURUN
    /sara/water_detect_2 (std_msgs/Bool) - KUYRUK
    /sara/water_sensor/status (diagnostic_msgs/DiagnosticStatus)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

try:
    import Jetson.GPIO as GPIO
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False


class WaterSensorDriverNode(Node):

    def __init__(self):
        super().__init__('water_sensor_driver')

        self.declare_parameter('nose_pin', 15)     # BURUN - fiziksel (BOARD) pin numarasi
        self.declare_parameter('tail_pin', 16)     # KUYRUK - fiziksel (BOARD) pin numarasi
        self.declare_parameter('nose_active_high', True)  # DOGRULANMALI: HIGH=su var mi?
        self.declare_parameter('tail_active_high', True)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.nose_pin = int(self.get_parameter('nose_pin').value)
        self.tail_pin = int(self.get_parameter('tail_pin').value)
        self.nose_active_high = self.get_parameter('nose_active_high').value
        self.tail_active_high = self.get_parameter('tail_active_high').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._gpio_ok = False
        if HARDWARE_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BOARD)
                GPIO.setup(self.nose_pin, GPIO.IN)
                GPIO.setup(self.tail_pin, GPIO.IN)
                self._gpio_ok = True
                self.get_logger().info(f'GPIO hazir: burun=pin{self.nose_pin}, kuyruk=pin{self.tail_pin}')
            except Exception as e:
                self.get_logger().error(f'GPIO kurulumu BASARISIZ: {e}')
        else:
            self.get_logger().error(
                'Jetson.GPIO bulunamadi! "pip install Jetson.GPIO" ile kurun. '
                'Bu node donanima ULASAMIYOR.'
            )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._nose_pub = self.create_publisher(Bool, '/sara/water_detect_1', sensor_qos)
        self._tail_pub = self.create_publisher(Bool, '/sara/water_detect_2', sensor_qos)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/water_sensor/status', 10)

        self.create_timer(1.0 / rate, self._on_timer)

    def _on_timer(self):
        if not self._gpio_ok:
            status = DiagnosticStatus()
            status.name = 'sara_water_sensor'
            status.level = DiagnosticStatus.ERROR
            status.message = 'GPIO hazir degil'
            self._status_pub.publish(status)
            return

        nose_raw = GPIO.input(self.nose_pin)
        tail_raw = GPIO.input(self.tail_pin)

        nose_submerged = bool(nose_raw) if self.nose_active_high else not bool(nose_raw)
        tail_submerged = bool(tail_raw) if self.tail_active_high else not bool(tail_raw)

        n = Bool()
        n.data = nose_submerged
        self._nose_pub.publish(n)

        t = Bool()
        t.data = tail_submerged
        self._tail_pub.publish(t)

        status = DiagnosticStatus()
        status.name = 'sara_water_sensor'
        status.level = DiagnosticStatus.OK
        status.message = 'Nominal'
        status.values = [
            KeyValue(key='nose_submerged', value=str(nose_submerged)),
            KeyValue(key='tail_submerged', value=str(tail_submerged)),
            KeyValue(key='nose_raw', value=str(nose_raw)),
            KeyValue(key='tail_raw', value=str(tail_raw)),
        ]
        self._status_pub.publish(status)

    def destroy_node(self):
        if HARDWARE_AVAILABLE and self._gpio_ok:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaterSensorDriverNode()
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