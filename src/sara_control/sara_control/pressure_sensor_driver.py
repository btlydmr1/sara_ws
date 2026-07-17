#!/usr/bin/env python3
"""
pressure_sensor_driver.py
===========================
SARA platformu - ADS1115 + SEN0257 Basinc Sensoru Surucusu (GERCEK DONANIM)

Bagli donanim (kullanicidan alinan gercek kablolama):
    ADS1115 (I2C ADC):
        VDD->pin1, GND->pin6, SDA->pin27, SCL->pin28, ADDR->GND (adres 0x48)
        A0 <- SEN0257 gerilim bolucu ciktisi
    SEN0257 (analog basinc sensoru):
        Kirmizi->5V, Siyah->GND
        Sari (analog cikis) -> 10k -> ADS A0 -> 20k -> GND  (gerilim bolucu)

*** GERILIM BOLUCU GERI HESABI ***
ADS1115, A0'da gerilim bolucunun SONRASINI olcer. Sensorun GERCEK cikis
gerilimini geri hesaplamak icin bolucu orani duzeltilir:
    V_sensor = V_adc * (R1+R2)/R2 = V_adc * (10k+20k)/20k = V_adc * 1.5

*** KALIBRASYON GEREKLI ***
SEN0257'nin gerilim->basinc egrisi (voltage_min/max, pressure_min/max_pa
parametreleri) DFRobot'un yaygin SEN0257 varyanti icin TAHMINI degerlerle
(0.5-4.5V -> 0-1.2MPa) ayarlanmistir. SATIN ALDIGINIZ SENSORUN GERCEK
DATASHEET DEGERLERINI DOGRULAYIP PARAMETRELERI GUNCELLEYIN.

Bu node, navigation.py'nin bekledigi TAM AYNI /sara/pressure
(sensor_msgs/FluidPressure) topic'ine, Pascal biriminde MUTLAK basinc
olarak yayin yapar (navigation.py zaten yuzey referansini P0 olarak
kendi kalibre ediyor - burada ekstra bir sey yapmaya gerek yok).

Cikti:
    /sara/pressure (sensor_msgs/FluidPressure)
    /sara/pressure_sensor/status (diagnostic_msgs/DiagnosticStatus)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import FluidPressure
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

try:
    import smbus2
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

_REG_CONVERSION = 0x00
_REG_CONFIG = 0x01

# Config: OS=1(baslat) MUX=100(AIN0-GND) PGA=001(+-4.096V) MODE=1(single-shot)
#         DR=100(128SPS) COMP_QUE=11(comparator kapali) -> 0xC383
_CONFIG_AIN0_4V096_SINGLE = 0xC383
_FULL_SCALE_VOLTAGE = 4.096  # PGA=001 icin tam skala [V]


class PressureSensorDriverNode(Node):

    def __init__(self):
        super().__init__('pressure_sensor_driver')

        self.declare_parameter('i2c_bus', 0)          # TODO: 'i2cdetect -y <bus>' ile dogrulayin (pin27/28)
        self.declare_parameter('i2c_address', 0x48)     # ADDR->GND -> varsayilan adres

        self.declare_parameter('divider_r1_ohm', 10000.0)
        self.declare_parameter('divider_r2_ohm', 20000.0)

        self.declare_parameter('sensor_voltage_min', 0.5)
        self.declare_parameter('sensor_voltage_max', 4.5)
        self.declare_parameter('sensor_pressure_min_pa', 0.0)
        self.declare_parameter('sensor_pressure_max_pa', 1200000.0)

        self.declare_parameter('publish_rate_hz', 20.0)

        self.r1 = self.get_parameter('divider_r1_ohm').value
        self.r2 = self.get_parameter('divider_r2_ohm').value
        self.v_min = self.get_parameter('sensor_voltage_min').value
        self.v_max = self.get_parameter('sensor_voltage_max').value
        self.p_min = self.get_parameter('sensor_pressure_min_pa').value
        self.p_max = self.get_parameter('sensor_pressure_max_pa').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._bus = None
        self._addr = int(self.get_parameter('i2c_address').value)
        if HARDWARE_AVAILABLE:
            try:
                self._bus = smbus2.SMBus(int(self.get_parameter('i2c_bus').value))
                self.get_logger().info(f'ADS1115 I2C baglantisi acildi (adres=0x{self._addr:02X}).')
            except Exception as e:
                self.get_logger().error(f'I2C bus acilamadi: {e}')
                self._bus = None
        else:
            self.get_logger().error(
                'smbus2 bulunamadi! "pip install smbus2" ile kurun. Bu node donanima ULASAMIYOR.'
            )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pressure_pub = self.create_publisher(FluidPressure, '/sara/pressure', sensor_qos)
        self._status_pub = self.create_publisher(DiagnosticStatus, '/sara/pressure_sensor/status', 10)

        self.create_timer(1.0 / rate, self._on_timer)

    def _read_ads1115_voltage(self) -> float:
        """AIN0 kanalindan tek seferlik olcum yapar, ADS1115'in olctugu gerilimi dondurur."""
        high = (_CONFIG_AIN0_4V096_SINGLE >> 8) & 0xFF
        low = _CONFIG_AIN0_4V096_SINGLE & 0xFF
        self._bus.write_i2c_block_data(self._addr, _REG_CONFIG, [high, low])

        time.sleep(0.009)

        data = self._bus.read_i2c_block_data(self._addr, _REG_CONVERSION, 2)
        raw = (data[0] << 8) | data[1]
        if raw > 32767:
            raw -= 65536
        voltage = raw * (_FULL_SCALE_VOLTAGE / 32768.0)
        return voltage

    def _on_timer(self):
        if self._bus is None:
            status = DiagnosticStatus()
            status.name = 'sara_pressure_sensor'
            status.level = DiagnosticStatus.ERROR
            status.message = 'I2C baglantisi yok'
            self._status_pub.publish(status)
            return

        try:
            v_adc = self._read_ads1115_voltage()
        except Exception as e:
            self.get_logger().error(f'ADS1115 okuma hatasi: {e}')
            status = DiagnosticStatus()
            status.name = 'sara_pressure_sensor'
            status.level = DiagnosticStatus.ERROR
            status.message = f'Okuma hatasi: {e}'
            self._status_pub.publish(status)
            return

        v_sensor = v_adc * (self.r1 + self.r2) / self.r2

        ratio = (v_sensor - self.v_min) / (self.v_max - self.v_min) if self.v_max != self.v_min else 0.0
        ratio = max(0.0, min(1.0, ratio))
        pressure_pa = self.p_min + ratio * (self.p_max - self.p_min)

        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sara_pressure_sensor'
        msg.fluid_pressure = pressure_pa
        msg.variance = 0.0
        self._pressure_pub.publish(msg)

        status = DiagnosticStatus()
        status.name = 'sara_pressure_sensor'
        status.level = DiagnosticStatus.OK
        status.message = 'Nominal'
        status.values = [
            KeyValue(key='v_adc', value=f'{v_adc:.3f}'),
            KeyValue(key='v_sensor', value=f'{v_sensor:.3f}'),
            KeyValue(key='pressure_pa', value=f'{pressure_pa:.1f}'),
        ]
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = PressureSensorDriverNode()
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