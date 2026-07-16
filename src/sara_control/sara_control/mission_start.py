#!/usr/bin/env python3
"""
mission_start.py
==================
SARA platformu - Gorev Baslatma / Akustik Uyari Katmani
(Ontasarim Raporu Tablo 11'de GUDUM'DEN BAGIMSIZ ayri bir yazilim katmani
olarak tanimlanmistir: "Ana enerjilendirme sonrasinda 60 saniye motor
inhibit suresini ve son 10 saniyelik pinger/buzzer uyarisini yonetir.")

Bu node bilerek KUCUK ve TEK SORUMLULUKLU tutulmustur: sadece zamanlayici +
guvenlik durumu izler, hicbir gorev fazi / hedef derinlik-heading-pitch
mantigi ICERMEZ (o Gudum katmaninda kalir - guidance.py).

Akis:
    IDLE (start_command bekleniyor)
        --start_command=True-->
    COUNTING_DOWN (60 sn motor inhibit, son 10 sn'de acoustic_warning=True)
        --60 sn dolunca-->
    PERMITTED (motion_permission=True, kalici)

    Herhangi bir anda emergency_stop=True olursa: motion_permission=False
    olur ve IDLE'a donulur (yeniden baslatma icin start_command gerekir) -
    bu, "acil durdurma -> motor hemen durur" guvenlik kuralinin bu katmandaki
    karsiligidir.

Girdi:
    /sara/mission_start/start_command   (std_msgs/Bool)  - "gorev baslatma izni"
                                          (fiziksel anahtar/lanyard -> True)
    /sara/safety/emergency_stop         (std_msgs/Bool)  - yoksa False varsayilir

Cikti:
    /sara/mission_start/motion_permission  (std_msgs/Bool)  - "Hareket baslatma izni"
    /sara/mission_start/acoustic_warning   (std_msgs/Bool)  - pinger/buzzer donanim sinyali
    /sara/mission_start/status             (diagnostic_msgs/DiagnosticStatus)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


STATE_IDLE = 'IDLE_BEKLEMEDE'
STATE_COUNTING_DOWN = 'SAYIM_60SN_MOTOR_INHIBIT'
STATE_PERMITTED = 'HAREKET_IZNI_VERILDI'


class MissionStartNode(Node):

    def __init__(self):
        super().__init__('mission_start_node')

        self.declare_parameter('motor_inhibit_duration_s', 60.0)   # GUVENLIK KURALI - sabit
        self.declare_parameter('acoustic_warning_lead_s', 10.0)     # rapor - Tablo 11
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('auto_start', False)                  # True ise start_command beklemeden ana enerjilendirmede otomatik baslar

        self.motor_inhibit_duration = self.get_parameter('motor_inhibit_duration_s').value
        self.acoustic_warning_lead = self.get_parameter('acoustic_warning_lead_s').value
        self.auto_start = self.get_parameter('auto_start').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._state = STATE_IDLE
        self._countdown_start = None
        self._start_command = bool(self.auto_start)
        self._emergency_stop = False

        self._motion_permission = False
        self._acoustic_warning = False

        self.create_subscription(Bool, '/sara/mission_start/start_command', self._on_start_command, 10)
        self.create_subscription(Bool, '/sara/safety/emergency_stop', self._on_emergency_stop, 10)

        self._permission_pub = self.create_publisher(Bool, '/sara/mission_start/motion_permission', 10)
        self._warning_pub = self.create_publisher(Bool, '/sara/mission_start/acoustic_warning', 10)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/mission_start/status', 10)

        self.create_timer(1.0 / rate, self._on_timer)

        if self.auto_start:
            self._countdown_start = self.get_clock().now()
            self._state = STATE_COUNTING_DOWN

        self.get_logger().info(
            'mission_start_node baslatildi. '
            f'auto_start={self.auto_start}. Motor inhibit suresi = {self.motor_inhibit_duration:.0f} sn, '
            f'son {self.acoustic_warning_lead:.0f} sn pinger/buzzer uyarisi verilecek.'
        )

    # ======================================================================
    def _on_start_command(self, msg: Bool):
        self._start_command = msg.data

    def _on_emergency_stop(self, msg: Bool):
        self._emergency_stop = msg.data

    # ======================================================================
    def _on_timer(self):
        # GUVENLIK KURALI: acil durdurma -> motor hemen durur, IDLE'a don
        if self._emergency_stop:
            if self._state != STATE_IDLE:
                self.get_logger().warn('Acil durdurma - hareket izni geri alindi, IDLE durumuna donuluyor.')
            self._state = STATE_IDLE
            self._countdown_start = None
            self._motion_permission = False
            self._acoustic_warning = False
            self._publish_status()
            return

        if self._state == STATE_IDLE:
            self._motion_permission = False
            self._acoustic_warning = False
            if self._start_command:
                self.get_logger().info(
                    f'Gorev baslatma izni alindi - {self.motor_inhibit_duration:.0f} sn motor inhibit sayaci basladi.'
                )
                self._state = STATE_COUNTING_DOWN
                self._countdown_start = self.get_clock().now()

        elif self._state == STATE_COUNTING_DOWN:
            elapsed = (self.get_clock().now() - self._countdown_start).nanoseconds * 1e-9
            remaining = self.motor_inhibit_duration - elapsed
            self._motion_permission = False
            self._acoustic_warning = remaining <= self.acoustic_warning_lead

            if elapsed >= self.motor_inhibit_duration:
                self._state = STATE_PERMITTED
                self._acoustic_warning = False
                self._motion_permission = True
                self.get_logger().info('60 sn motor inhibit tamamlandi - hareket baslatma izni VERILDI.')

        else:  # STATE_PERMITTED
            self._motion_permission = True
            self._acoustic_warning = False

        self._publish_status()

    def _publish_status(self):
        perm = Bool()
        perm.data = bool(self._motion_permission)
        self._permission_pub.publish(perm)

        warn = Bool()
        warn.data = bool(self._acoustic_warning)
        self._warning_pub.publish(warn)

        remaining_s = 0.0
        if self._state == STATE_COUNTING_DOWN and self._countdown_start is not None:
            elapsed = (self.get_clock().now() - self._countdown_start).nanoseconds * 1e-9
            remaining_s = max(0.0, self.motor_inhibit_duration - elapsed)

        status = DiagnosticStatus()
        status.name = 'sara_mission_start'
        status.hardware_id = 'jetson_orin_nano'
        status.level = DiagnosticStatus.WARN if self._emergency_stop else DiagnosticStatus.OK
        status.message = f'Durum: {self._state}'
        status.values = [
            KeyValue(key='motion_permission', value=str(self._motion_permission)),
            KeyValue(key='acoustic_warning', value=str(self._acoustic_warning)),
            KeyValue(key='remaining_inhibit_s', value=f'{remaining_s:.1f}'),
            KeyValue(key='emergency_stop', value=str(self._emergency_stop)),
        ]
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = MissionStartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()