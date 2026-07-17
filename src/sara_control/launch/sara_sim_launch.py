#!/usr/bin/env python3
"""
sara_system.launch.py
=======================
SARA platformu - TUM sistemi TEK KOMUTLA baslatan launch dosyasi.

Su ana kadar 7 node'u ayri ayri terminallerde elle baslatiyorduk - bu
dosya hepsini tek surecte, dogru sirayla, dogru parametrelerle baslatir.

KULLANIM:

  Test modu (vehicle_sim ile, donanim GEREKMEZ):
    ros2 launch sara_control sara_system.launch.py mode:=test mission_id:=1
    ros2 launch sara_control sara_system.launch.py mode:=test mission_id:=2

  Gercek donanim modu (Pixhawk/MAVROS ve I2C/GPIO donanimi BAGLI OLMALI,
  mavros ayrica baslatilmis olmalidir - bu launch dosyasi mavros'u BASLATMAZ):
    ros2 launch sara_control sara_system.launch.py mode:=hardware mission_id:=1

ARGUMANLAR:
    mission_id     : '1' (Seyir Gorevi) veya '2' (Atis Gorevi). Varsayilan: '1'
    mode           : 'test' (vehicle_sim) veya 'hardware' (gercek sensor/
                     eyleyici surucu node'lari). Varsayilan: 'test'
    enable_telemetry : CSV telemetri kaydini ac/kapat. Varsayilan: 'true'
    log_directory  : telemetri CSV'lerinin kaydedilecegi klasor.
                     Varsayilan: '~/sara_logs'

NOT: 'hardware' modunda, bu launch dosyasindan ONCE ayrica su calistirilmis
olmalidir (bu launch dosyasi mavros baglantisini KURMAZ - Pixhawk 6X bu
donanim topolojisinde SADECE IMU/telemetri kaynagidir, eyleyicileri
SURMEZ; eyleyiciler dogrudan Jetson'dan PCA9685 uzerinden surulur):
    ros2 launch mavros px4.launch (veya ilgili apm/px4 launch dosyaniz)

'hardware' modunda baslatilan gercek donanim sürücüleri (PIXHAWK_BRIDGE
ARTIK KULLANILMIYOR - bu donanim topolojisinde Pixhawk eyleyici surmuyor):
    actuator_driver        - PCA9685: ESC(CH0), Pitch(CH1), Yaw(CH2), Burun kapagi(CH3)
    water_sensor_driver     - SEN0368 x2: Jetson GPIO pin15(burun)/pin16(kuyruk)
    pressure_sensor_driver  - ADS1115+SEN0257: Jetson I2C pin27(SDA)/pin28(SCL)

EKSIK (henuz surulmuyor, donanim netlesince eklenecek):
    - Sephiye (step motor) - pin/surucu atamasi belirsiz
    - Roket atesleme sinyali - fiziksel baglanti tanimsiz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mission_id_arg = DeclareLaunchArgument(
        'mission_id', default_value='1',
        description="Kosulacak gorev: '1' (Seyir) veya '2' (Atis)"
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='test',
        description="'test' (vehicle_sim ile) veya 'hardware' (gercek sensor/eyleyici suruculeri ile)"
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
        condition=IfCondition(is_test_mode),
    )

    # ================= GERCEK DONANIM MODU - Jetson'a dogrudan bagli suruculer =================
    # NOT: pixhawk_bridge ARTIK KULLANILMIYOR - bu donanim topolojisinde
    # Pixhawk 6X sadece IMU/telemetri saglar (mavros uzerinden), eyleyicileri
    # SURMEZ. Tum eyleyiciler PCA9685 uzerinden dogrudan Jetson'dan surulur.
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
    navigation_node = Node(
        package='sara_control',
        executable='navigation',
        name='navigation_node',
        output='screen',
    )

    mission_start_node = Node(
        package='sara_control',
        executable='mission_start',
        name='mission_start_node',
        output='screen',
    )

    guidance_node = Node(
        package='sara_control',
        executable='guidance',
        name='guidance_node',
        output='screen',
        parameters=[{'mission_id': mission_id}],
    )

    autopilot_node = Node(
        package='sara_control',
        executable='autopilot',
        name='autopilot_node',
        output='screen',
    )

    safety_node = Node(
        package='sara_control',
        executable='safety',
        name='safety_node',
        output='screen',
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