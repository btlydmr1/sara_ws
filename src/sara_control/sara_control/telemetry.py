#!/usr/bin/env python3
"""
telemetry.py
=============
SARA platformu - Telemetri / Loglama Katmani (Tablo 11'in kendi satiri:
"Sensor, durum kestirimi, guvenlik ve komut verilerini test amaciyla
kaydeder.")

Calisma prensibi: "ucus kayit cihazi" (flight recorder) mantigi. Sistemdeki
BUTUN onemli topic'lere abone olunur, her birinin SON deger goruntusu ic
durumda tutulur; sabit bir frekansta (varsayilan 5Hz) bu goruntunun TAMAMI
TEK BIR CSV SATIRINA yazilir. Boylece test sonrasi tum gorev kaydini
Excel/pandas ile acip zaman ekseninde inceleyebilirsiniz (derinlik-zaman,
pitch-zaman, faz gecisleri, komutlar vb.).

NOT: Bu, ham ROS2 mesaj kaydi (rosbag2) YERINE GECMEZ - rosbag2 ile tam
sadakatte (kayip mesaj olmadan) kayit almak isterseniz ayrica
`ros2 bag record -a` kullanabilirsiniz. Bu node, TEST SONRASI HIZLI ANALIZ
icin okunabilir, tek dosyalik ozet bir CSV uretir.

Dinlenen topic'ler (TUM katmanlar):
    Navigasyon: /sara/navigation/odom, surface_detected, status
    Gudum: /sara/guidance/target_pose, mission_phase, forward_motion_request,
           nose_cap_open_request, launch_request, status
    Gorev Baslatma: /sara/mission_start/motion_permission, acoustic_warning, status
    Otopilot: /sara/control/thrust_request, fin_request, buoyancy_request, status
    Guvenlik: /sara/control/thrust_command, fin_command, buoyancy_command,
              nose_cap_command, launch_command, /sara/safety/approved,
              fail_safe_active, status
    Koprü: /sara/bridge/status
    Guvenlik girdileri: /sara/safety/emergency_stop, leak_detected

Cikti: ~/sara_logs/sara_telemetry_<YYYYMMDD_HHMMSS>.csv (varsayilan; parametre ile degistirilebilir)
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
from diagnostic_msgs.msg import DiagnosticStatus


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


def level_to_str(level) -> str:
    mapping = {
        DiagnosticStatus.OK: 'OK',
        DiagnosticStatus.WARN: 'WARN',
        DiagnosticStatus.ERROR: 'ERROR',
        DiagnosticStatus.STALE: 'STALE',
    }
    return mapping.get(level, str(level))


CSV_COLUMNS = [
    'timestamp_s', 'mission_phase',
    'depth_m', 'heading_deg', 'pitch_deg', 'roll_deg',
    'approx_x_m', 'approx_y_m', 'approx_distance_m',
    'surface_detected', 'nav_status_level',
    'target_depth_m', 'target_heading_deg', 'target_pitch_deg',
    'forward_motion_request', 'target_speed_ms', 'nose_cap_open_request', 'launch_request',
    'task_complete',                                                       # YENI
    'guidance_status_level',
    'motion_permission', 'acoustic_warning', 'mission_start_status_level',
    'thrust_request', 'fin_pitch_request', 'fin_yaw_request',
    'buoyancy_request', 'autopilot_status_level',
    'thrust_command', 'fin_pitch_command', 'fin_yaw_command',
    'buoyancy_command', 'nose_cap_command', 'launch_command',
    'safety_approved', 'fail_safe_active', 'main_power_cutoff_command',     # YENI
    'safety_status_level',
    'emergency_stop', 'leak_detected',
]


class TelemetryNode(Node):

    def __init__(self):
        super().__init__('telemetry_node')

        self.declare_parameter('log_directory', '~/sara_logs')
        self.declare_parameter('log_rate_hz', 5.0)
        self.declare_parameter('log_to_console', False)

        log_dir = os.path.expanduser(self.get_parameter('log_directory').value)
        rate = float(self.get_parameter('log_rate_hz').value)
        self.log_to_console = self.get_parameter('log_to_console').value

        os.makedirs(log_dir, exist_ok=True)
        # DUZELTME: saniye hassasiyeti yerine mikrosaniye - ayni saniye
        # icinde iki kez baslatilirsa (hizli tekrar test) dosya adi
        # CAKISMASINI onler (cakisirsa bir node digerinin dosyasini 'w'
        # modunda acip icerigini bozabilir).
        filename = f"sara_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.csv"
        self.log_path = os.path.join(log_dir, filename)

        self._csv_file = open(self.log_path, 'w', newline='')
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS)
        self._csv_writer.writeheader()
        self._csv_file.flush()
        self._row_count = 0

        # ================= Ic durum (her topic'in SON goruntusu) =================
        self._mission_phase = ''
        self._depth = 0.0
        self._heading = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._approx_x = 0.0
        self._approx_y = 0.0
        self._surface_detected = False
        self._nav_status_level = DiagnosticStatus.STALE

        self._target_depth = 0.0
        self._target_heading = 0.0
        self._target_pitch = 0.0
        self._forward_motion_request = False
        self._target_speed = 0.0  # YENI
        self._nose_cap_open_request = False
        self._launch_request = False
        self._guidance_status_level = DiagnosticStatus.STALE

        self._motion_permission = False
        self._acoustic_warning = False
        self._mission_start_status_level = DiagnosticStatus.STALE

        self._thrust_request = 0.0
        self._fin_pitch_request = 0.0
        self._fin_yaw_request = 0.0
        self._buoyancy_request = 0.0
        self._autopilot_status_level = DiagnosticStatus.STALE

        self._thrust_command = 0.0
        self._fin_pitch_command = 0.0
        self._fin_yaw_command = 0.0
        self._buoyancy_command = 0.0
        self._nose_cap_command = False
        self._launch_command = False
        self._safety_approved = False
        self._fail_safe_active = False
        self._safety_status_level = DiagnosticStatus.STALE

        self._emergency_stop = False
        self._leak_detected = False
        self._task_complete = False              # YENI
        self._main_power_cutoff_command = False  # YENI

        # ================= Abonelikler =================
        self.create_subscription(Odometry, '/sara/navigation/odom', self._on_odom, 10)
        self.create_subscription(Bool, '/sara/navigation/surface_detected', self._on_surface, 10)
        self.create_subscription(DiagnosticStatus, '/sara/navigation/status',
                                  lambda m: self._set('_nav_status_level', m.level), 10)

        self.create_subscription(PoseStamped, '/sara/guidance/target_pose', self._on_target_pose, 10)
        self.create_subscription(String, '/sara/guidance/mission_phase',
                                  lambda m: self._set('_mission_phase', m.data), 10)
        self.create_subscription(Bool, '/sara/guidance/forward_motion_request',
                                  lambda m: self._set('_forward_motion_request', m.data), 10)
        self.create_subscription(Float64, '/sara/guidance/target_speed',
                                  lambda m: self._set('_target_speed', m.data), 10)  # YENI
        self.create_subscription(Bool, '/sara/guidance/nose_cap_open_request',
                                  lambda m: self._set('_nose_cap_open_request', m.data), 10)
        self.create_subscription(Bool, '/sara/guidance/launch_request',
                                  lambda m: self._set('_launch_request', m.data), 10)
        self.create_subscription(Bool, '/sara/guidance/task_complete',
                                  lambda m: self._set('_task_complete', m.data), 10)  # YENI
        self.create_subscription(DiagnosticStatus, '/sara/guidance/status',
                                  lambda m: self._set('_guidance_status_level', m.level), 10)

        self.create_subscription(Bool, '/sara/mission_start/motion_permission',
                                  lambda m: self._set('_motion_permission', m.data), 10)
        self.create_subscription(Bool, '/sara/mission_start/acoustic_warning',
                                  lambda m: self._set('_acoustic_warning', m.data), 10)
        self.create_subscription(DiagnosticStatus, '/sara/mission_start/status',
                                  lambda m: self._set('_mission_start_status_level', m.level), 10)

        self.create_subscription(Float64, '/sara/control/thrust_request',
                                  lambda m: self._set('_thrust_request', m.data), 10)
        self.create_subscription(Vector3, '/sara/control/fin_request', self._on_fin_request, 10)
        self.create_subscription(Float64, '/sara/control/buoyancy_request',
                                  lambda m: self._set('_buoyancy_request', m.data), 10)
        self.create_subscription(DiagnosticStatus, '/sara/control/status',
                                  lambda m: self._set('_autopilot_status_level', m.level), 10)

        self.create_subscription(Float64, '/sara/control/thrust_command',
                                  lambda m: self._set('_thrust_command', m.data), 10)
        self.create_subscription(Vector3, '/sara/control/fin_command', self._on_fin_command, 10)
        self.create_subscription(Float64, '/sara/control/buoyancy_command',
                                  lambda m: self._set('_buoyancy_command', m.data), 10)
        self.create_subscription(Bool, '/sara/control/nose_cap_command',
                                  lambda m: self._set('_nose_cap_command', m.data), 10)
        self.create_subscription(Bool, '/sara/control/launch_command',
                                  lambda m: self._set('_launch_command', m.data), 10)
        self.create_subscription(Bool, '/sara/safety/approved',
                                  lambda m: self._set('_safety_approved', m.data), 10)
        self.create_subscription(Bool, '/sara/safety/fail_safe_active',
                                  lambda m: self._set('_fail_safe_active', m.data), 10)
        self.create_subscription(Bool, '/sara/safety/main_power_cutoff_command',
                                  lambda m: self._set('_main_power_cutoff_command', m.data), 10)  # YENI
        self.create_subscription(DiagnosticStatus, '/sara/safety/status',
                                  lambda m: self._set('_safety_status_level', m.level), 10)

        self.create_subscription(Bool, '/sara/safety/emergency_stop',
                                  lambda m: self._set('_emergency_stop', m.data), 10)
        self.create_subscription(Bool, '/sara/safety/leak_detected',
                                  lambda m: self._set('_leak_detected', m.data), 10)

        # DUZELTME (denetimde bulundu): /sara/bridge/status aboneligi
        # KALDIRILDI - bu topic'i publish eden hicbir node YOK (eski,
        # artik kullanilmayan pixhawk_bridge.py'den kalma bir yetim
        # abonelikti, bkz. sara_system.launch.py'nin kendi notu: "DUZELTME:
        # 'pixhawk_bridge' ARTIK KULLANILMIYOR"). CSV'de surekli STALE
        # gorunmesi disinda zararsizdi ama artik gereksiz kod/sutun.

        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(f'telemetry_node baslatildi. Kayit dosyasi: {self.log_path}')

    def _set(self, attr_name, value):
        setattr(self, attr_name, value)

    def _on_odom(self, msg: Odometry):
        self._depth = msg.pose.pose.position.z
        self._approx_x = msg.pose.pose.position.x
        self._approx_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw, pitch, roll = quaternion_to_yaw_pitch_roll(q.x, q.y, q.z, q.w)
        self._heading = yaw
        self._pitch = pitch
        self._roll = roll

    def _on_surface(self, msg: Bool):
        self._surface_detected = msg.data

    def _on_target_pose(self, msg: PoseStamped):
        self._target_depth = msg.pose.position.z
        q = msg.pose.orientation
        yaw, pitch, _roll = quaternion_to_yaw_pitch_roll(q.x, q.y, q.z, q.w)
        self._target_heading = yaw
        self._target_pitch = pitch

    def _on_fin_request(self, msg: Vector3):
        self._fin_pitch_request = msg.x
        self._fin_yaw_request = msg.y

    def _on_fin_command(self, msg: Vector3):
        self._fin_pitch_command = msg.x
        self._fin_yaw_command = msg.y

    # ======================================================================
    def _on_timer(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        row = {
            'timestamp_s': f'{now_s:.3f}',
            'mission_phase': self._mission_phase,
            'depth_m': f'{self._depth:.3f}',
            'heading_deg': f'{math.degrees(self._heading):.1f}',
            'pitch_deg': f'{math.degrees(self._pitch):.1f}',
            'roll_deg': f'{math.degrees(self._roll):.1f}',
            'approx_x_m': f'{self._approx_x:.2f}',
            'approx_y_m': f'{self._approx_y:.2f}',
            'approx_distance_m': f'{math.hypot(self._approx_x, self._approx_y):.2f}',
            'surface_detected': self._surface_detected,
            'nav_status_level': level_to_str(self._nav_status_level),
            'target_depth_m': f'{self._target_depth:.3f}',
            'target_heading_deg': f'{math.degrees(self._target_heading):.1f}',
            'target_pitch_deg': f'{math.degrees(self._target_pitch):.1f}',
            'forward_motion_request': self._forward_motion_request,
            'target_speed_ms': f'{self._target_speed:.3f}',
            'nose_cap_open_request': self._nose_cap_open_request,
            'launch_request': self._launch_request,
            'task_complete': self._task_complete,  # YENI
            'guidance_status_level': level_to_str(self._guidance_status_level),
            'motion_permission': self._motion_permission,
            'acoustic_warning': self._acoustic_warning,
            'mission_start_status_level': level_to_str(self._mission_start_status_level),
            'thrust_request': f'{self._thrust_request:.2f}',
            'fin_pitch_request': f'{self._fin_pitch_request:.2f}',
            'fin_yaw_request': f'{self._fin_yaw_request:.2f}',
            'buoyancy_request': f'{self._buoyancy_request:.2f}',
            'autopilot_status_level': level_to_str(self._autopilot_status_level),
            'thrust_command': f'{self._thrust_command:.2f}',
            'fin_pitch_command': f'{self._fin_pitch_command:.2f}',
            'fin_yaw_command': f'{self._fin_yaw_command:.2f}',
            'buoyancy_command': f'{self._buoyancy_command:.2f}',
            'nose_cap_command': self._nose_cap_command,
            'launch_command': self._launch_command,
            'safety_approved': self._safety_approved,
            'fail_safe_active': self._fail_safe_active,
            'main_power_cutoff_command': self._main_power_cutoff_command,  # YENI
            'safety_status_level': level_to_str(self._safety_status_level),
            'emergency_stop': self._emergency_stop,
            'leak_detected': self._leak_detected,
        }
        self._csv_writer.writerow(row)
        self._row_count += 1
        # DUZELTME: eskiden 50 satirda bir (5Hz'de ~10 sn) flush
        # yapiliyordu - beklenmedik kapanma (VM crash, guc kesintisi vb.)
        # durumunda bu pencere kaybolabiliyordu (sahada boyle bir CSV
        # bulundu: basliktan sonra tamamen NUL byte). 5Hz'de flush+fsync
        # maliyeti ihmal edilebilir (CSV satiri kucuk), guvenlik/test
        # verisi icin her satirda diske yazmak tercih edildi.
        self._csv_file.flush()
        os.fsync(self._csv_file.fileno())

        if self.log_to_console:
            self.get_logger().info(
                f"[{row['mission_phase']}] derinlik={row['depth_m']}m "
                f"heading={row['heading_deg']}deg pitch={row['pitch_deg']}deg"
            )

    def close(self):
        try:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(f'Telemetri kaydi kapatildi: {self.log_path} ({self._row_count} satir)')
        except Exception as e:
            self.get_logger().error(f'Telemetri dosyasi kapatilirken hata: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()