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
        --60 sn dolunca VE alt sistemler saglikliysa-->
    PERMITTED (motion_permission=True, kalici)

    YENI - Self-Check Gate: 60 sn dolmus olsa bile, navigation/guidance/
    autopilot/safety node'larindan biri ERROR durumdaysa veya hic veri
    gelmiyorsa (crash/baglanti kaybi), hareket izni VERILMEZ - sebep
    status mesajinda acikca gorunur (orn. "navigasyon hazir degil").
    Bu, sahada bir alt sistem arizasinin sessizce/belirsiz sekilde
    gozden kacmasini onler. (DUZELTME: guidance_node de bu kontrole
    eklendi - eskiden sadece navigation/autopilot/safety kontrol
    ediliyordu, gudum katmani baslangicta cokmus olsa dahi 60sn sonunda
    hareket izni verilebiliyordu.)

    Herhangi bir anda emergency_stop=True olursa: motion_permission=False
    olur ve IDLE'a donulur (yeniden baslatma icin start_command gerekir) -
    bu, "acil durdurma -> motor hemen durur" guvenlik kuralinin bu katmandaki
    karsiligidir.

Girdi:
    /sara/mission_start/start_command   (std_msgs/Bool)  - "gorev baslatma izni"
                                          (fiziksel anahtar/lanyard -> True)
    /sara/safety/emergency_stop         (std_msgs/Bool)  - yoksa False varsayilir
    /sara/navigation/status              (diagnostic_msgs/DiagnosticStatus) - YENI: self-check
    /sara/guidance/status                 (diagnostic_msgs/DiagnosticStatus) - YENI: self-check
    /sara/control/status                 (diagnostic_msgs/DiagnosticStatus) - YENI: self-check (autopilot)
    /sara/safety/status                  (diagnostic_msgs/DiagnosticStatus) - YENI: self-check

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
        self.declare_parameter('require_subsystems_healthy', True)     # YENI: self-check gate ac/kapa (test icin kapatilabilir)
        self.declare_parameter('subsystem_status_timeout_s', 2.0)       # YENI: alt sistem status'unun "taze" sayilma suresi

        self.motor_inhibit_duration = self.get_parameter('motor_inhibit_duration_s').value
        self.acoustic_warning_lead = self.get_parameter('acoustic_warning_lead_s').value
        self.auto_start = self.get_parameter('auto_start').value
        self.require_subsystems_healthy = self.get_parameter('require_subsystems_healthy').value
        self.subsystem_status_timeout = self.get_parameter('subsystem_status_timeout_s').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._state = STATE_IDLE
        self._countdown_start = None
        self._start_command = bool(self.auto_start)
        self._prev_start_command = self._start_command
        self._emergency_stop = False

        self._motion_permission = False
        self._acoustic_warning = False

        # YENI - Self-Check Gate: alt sistemlerin son bilinen durumu
        self._nav_status_time = None
        self._nav_status_level = DiagnosticStatus.STALE
        self._guidance_status_time = None   # YENI
        self._guidance_status_level = DiagnosticStatus.STALE  # YENI
        self._autopilot_status_time = None
        self._autopilot_status_level = DiagnosticStatus.STALE
        self._safety_status_time = None
        self._safety_status_level = DiagnosticStatus.STALE

        self.create_subscription(Bool, '/sara/mission_start/start_command', self._on_start_command, 10)
        self.create_subscription(Bool, '/sara/safety/emergency_stop', self._on_emergency_stop, 10)
        self.create_subscription(DiagnosticStatus, '/sara/navigation/status', self._on_nav_status, 10)
        self.create_subscription(DiagnosticStatus, '/sara/guidance/status', self._on_guidance_status, 10)  # YENI
        self.create_subscription(DiagnosticStatus, '/sara/control/status', self._on_autopilot_status, 10)
        self.create_subscription(DiagnosticStatus, '/sara/safety/status', self._on_safety_status, 10)

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

    def _on_nav_status(self, msg: DiagnosticStatus):
        self._nav_status_time = self.get_clock().now()
        self._nav_status_level = msg.level

    def _on_guidance_status(self, msg: DiagnosticStatus):
        self._guidance_status_time = self.get_clock().now()
        self._guidance_status_level = msg.level

    def _on_autopilot_status(self, msg: DiagnosticStatus):
        self._autopilot_status_time = self.get_clock().now()
        self._autopilot_status_level = msg.level

    def _on_safety_status(self, msg: DiagnosticStatus):
        self._safety_status_time = self.get_clock().now()
        self._safety_status_level = msg.level

    def _subsystem_fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < self.subsystem_status_timeout

    def _subsystems_healthy(self):
        """YENI - Self-Check Gate: navigation/autopilot/safety node'larindan
        veri geliyor mu VE hicbiri ERROR bildirmiyor mu? Donmezse (True,[]),
        donerse (False, [sebep listesi]) - status mesajinda gosterilir."""
        if not self.require_subsystems_healthy:
            return True, []

        problems = []
        if not self._subsystem_fresh(self._nav_status_time):
            problems.append('navigasyon_veri_yok')
        elif self._nav_status_level == DiagnosticStatus.ERROR:
            problems.append('navigasyon_ERROR')

        if not self._subsystem_fresh(self._guidance_status_time):
            problems.append('gudum_veri_yok')
        elif self._guidance_status_level == DiagnosticStatus.ERROR:
            problems.append('gudum_ERROR')

        if not self._subsystem_fresh(self._autopilot_status_time):
            problems.append('otopilot_veri_yok')
        elif self._autopilot_status_level == DiagnosticStatus.ERROR:
            problems.append('otopilot_ERROR')

        if not self._subsystem_fresh(self._safety_status_time):
            problems.append('guvenlik_veri_yok')
        elif self._safety_status_level == DiagnosticStatus.ERROR:
            problems.append('guvenlik_ERROR')

        return (len(problems) == 0), problems

    # ======================================================================
    def _on_timer(self):
        # DUZELTME (Teknik Sartname Madde 4.2): "Enerjilendirme butonu aktif
        # edildikten sonra TEKRAR BASILMASI durumunda TUM SISTEMIN
        # enerjilendirilmesi KESILMELIDIR." Bu, Acil Durdurma'dan FARKLI bir
        # "Normal Kapali" durumu (Tablo 3: Acil Durdurma=0, Enerjilendirme=0
        # -> Normal Kapali). start_command'in True'dan False'a GECISINI
        # (ikinci basis) yakalayip sistemi IDLE'a donduruyoruz.
        if self._prev_start_command and not self._start_command and self._state != STATE_IDLE:
            self.get_logger().warn(
                'Enerjilendirme butonuna tekrar basildi - sistem NORMAL KAPALI durumuna donuyor '
                '(Acil Durdurma DEGIL).'
            )
            self._state = STATE_IDLE
            self._countdown_start = None
            self._motion_permission = False
            self._acoustic_warning = False
            self._prev_start_command = self._start_command
            self._publish_status()
            return
        self._prev_start_command = self._start_command

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
                # YENI - Self-Check Gate: sure dolmus olsa bile, alt
                # sistemler saglikli degilse izin VERILMEZ.
                healthy, problems = self._subsystems_healthy()
                if healthy:
                    self._state = STATE_PERMITTED
                    self._acoustic_warning = False
                    self._motion_permission = True
                    self.get_logger().info('60 sn motor inhibit tamamlandi - hareket baslatma izni VERILDI.')
                else:
                    self._acoustic_warning = False
                    self._motion_permission = False
                    self.get_logger().warn(
                        f'60 sn tamamlandi AMA alt sistemler hazir degil, izin VERILMEDI: '
                        f'{", ".join(problems)}',
                        throttle_duration_sec=5.0,
                    )

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
        healthy, problems = self._subsystems_healthy()
        if self._emergency_stop:
            status.level = DiagnosticStatus.WARN
        elif not healthy:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = f'Durum: {self._state}'
        if not healthy:
            status.message += f' - Alt sistem sorunu: {", ".join(problems)}'
        status.values = [
            KeyValue(key='motion_permission', value=str(self._motion_permission)),
            KeyValue(key='acoustic_warning', value=str(self._acoustic_warning)),
            KeyValue(key='remaining_inhibit_s', value=f'{remaining_s:.1f}'),
            KeyValue(key='emergency_stop', value=str(self._emergency_stop)),
            KeyValue(key='subsystems_healthy', value=str(healthy)),
            KeyValue(key='subsystem_problems', value=','.join(problems) if problems else '-'),
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()