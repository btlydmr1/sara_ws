#!/usr/bin/env python3
"""
sara_system.launch.py
=======================
SARA platformu - TUM sistemi TEK KOMUTLA baslatan launch dosyasi.

KULLANIM:

  Test modu (vehicle_sim ile, donanim GEREKMEZ):
    ros2 launch sara_control sara_system.launch.py mode:=test mission_id:=1
    ros2 launch sara_control sara_system.launch.py mode:=test mission_id:=2

  Gercek donanim modu (Pixhawk/MAVROS BAGLI OLMALI, mavros ayrica
  baslatilmis olmalidir - bu launch dosyasi mavros'u BASLATMAZ):
    ros2 launch sara_control sara_system.launch.py mode:=hardware mission_id:=1

ARGUMANLAR:
    mission_id     : '1' (Seyir Gorevi) veya '2' (Atis Gorevi). Varsayilan: '1'
    mode           : 'test' (vehicle_sim) veya 'hardware' (gercek sensor/eyleyici
                     suruculeri). Varsayilan: 'test'
    enable_telemetry : CSV telemetri kaydini ac/kapat. Varsayilan: 'true'
    log_directory  : telemetri CSV'lerinin kaydedilecegi klasor.
                     Varsayilan: '~/sara_logs'

DUZELTME: 'pixhawk_bridge' ARTIK KULLANILMIYOR - bu donanim topolojisinde
Pixhawk 6X sadece IMU/telemetri saglar, eyleyicileri SURMEZ (eyleyiciler
dogrudan Jetson'dan PCA9685 uzerinden surulur). 'hardware' modu artik
actuator_driver + water_sensor_driver + pressure_sensor_driver baslatir.

NOT: sensor_get_data.py (daha yeni, tek-node birlesik sensor katmani)
BİLEREK BURADA BASLATILMIYOR - henuz donanima baglanmadi/test edilmedi.
Donanim netlesip sensor_get_data.py'ye gecmeye hazir olundugunda, bu
dosyadaki water_sensor_driver_node + pressure_sensor_driver_node ikilisi
TEK BIR sensor_get_data_node ile degistirilecek (ikisi AYNI ANDA
calistirilmamali - cakisir).

Her node, once kendi config/*.yaml dosyasini okur, SONRA launch
argumanlari (orn. mission_id) bu degerlerin UZERINE yazar.

NOT: 'hardware' modunda, bu launch dosyasindan ONCE ayrica su calistirilmis
olmalidir (bu launch dosyasi mavros baglantisini KURMAZ):
    ros2 launch mavros px4.launch (veya ilgili apm/px4 launch dosyaniz)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sara_control')
    config_dir = os.path.join(pkg_share, 'config')

    mission_id_arg = DeclareLaunchArgument(
        'mission_id', default_value='1',
        description="Kosulacak gorev: '1' (Seyir) veya '2' (Atis)"
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='test',
        description="'test' (vehicle_sim) veya 'hardware' (gercek sensor/eyleyici suruculeri)"
    )
    enable_telemetry_arg = DeclareLaunchArgument(
        'enable_telemetry', default_value='true',
        description='CSV telemetri kaydini ac/kapat'
    )
    log_directory_arg = DeclareLaunchArgument(
        'log_directory', default_value='~/sara_logs',
        description='Telemetri CSV kayit klasoru'
    )

    mission_id = LaunchConfiguration('mission_id')
    mode = LaunchConfiguration('mode')
    enable_telemetry = LaunchConfiguration('enable_telemetry')
    log_directory = LaunchConfiguration('log_directory')

    is_test_mode = PythonExpression(["'", mode, "' == 'test'"])
    is_hardware_mode = PythonExpression(["'", mode, "' == 'hardware'"])

    # ================= TEST MODU - sahte sensor/fizik simulatoru =================
    vehicle_sim_node = Node(
        package='sara_control',
        executable='vehicle_sim',
        name='vehicle_sim',
        output='screen',
        parameters=[os.path.join(config_dir, 'vehicle_sim_params.yaml')],
        condition=IfCondition(is_test_mode),
    )

    # ================= GERCEK DONANIM MODU =================
    # DUZELTME: pixhawk_bridge yerine gercek kablolamaya uygun 3 surucu.
    # sensor_get_data.py BİLEREK dahil edilmedi (henuz baglanmadi).
    actuator_driver_node = Node(
        package='sara_control',
        executable='actuator_driver',
        name='actuator_driver',
        output='screen',
        condition=IfCondition(is_hardware_mode),
    )
    water_sensor_driver_node = Node(
        package='sara_control',
        executable='water_sensor_driver',
        name='water_sensor_driver',
        output='screen',
        condition=IfCondition(is_hardware_mode),
    )
    pressure_sensor_driver_node = Node(
        package='sara_control',
        executable='pressure_sensor_driver',
        name='pressure_sensor_driver',
        output='screen',
        condition=IfCondition(is_hardware_mode),
    )

    # ================= HER IKI MODDA DA CALISAN CEKIRDEK NODE'LAR =================
    # Her node: once kendi yaml'ini okur, SONRA (varsa) launch argumani UZERINE yazar.
    navigation_node = Node(
        package='sara_control',
        executable='navigation',
        name='navigation_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'navigation_params.yaml')],
    )

    mission_start_node = Node(
        package='sara_control',
        executable='mission_start',
        name='mission_start_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'mission_start_params.yaml')],
    )

    guidance_node = Node(
        package='sara_control',
        executable='guidance',
        name='guidance_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'guidance_params.yaml'),
            {'mission_id': mission_id},  # yaml'daki mission_id'nin UZERINE yazar
        ],
    )

    autopilot_node = Node(
        package='sara_control',
        executable='autopilot',
        name='autopilot_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'autopilot_params.yaml')],
    )

    safety_node = Node(
        package='sara_control',
        executable='safety',
        name='safety_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'safety_params.yaml')],
    )

    telemetry_node = Node(
        package='sara_control',
        executable='telemetry',
        name='telemetry_node',
        output='screen',
        parameters=[{'log_directory': log_directory}],
        condition=IfCondition(enable_telemetry),
    )

    return LaunchDescription([
        mission_id_arg,
        mode_arg,
        enable_telemetry_arg,
        log_directory_arg,

        vehicle_sim_node,
        actuator_driver_node,
        water_sensor_driver_node,
        pressure_sensor_driver_node,

        navigation_node,
        mission_start_node,
        guidance_node,
        autopilot_node,
        safety_node,
        telemetry_node,
    ])