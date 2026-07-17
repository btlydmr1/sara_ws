#!/usr/bin/env python3
"""
sensor_data.py
==============
SARA platformu - Sensör Veri Alma Katmanı

Bu düğüm gerçek donanım verilerini ROS2 ortamına aktarır:

Girdiler
--------
- SEN0257 basınç sensörü -> ADS1115 A0
- SEN0368 sensör 1      -> Jetson fiziksel pin 15
- SEN0368 sensör 2      -> Jetson fiziksel pin 16
- Pixhawk IMU           -> /mavros/imu/data

Çıktılar
--------
- /sara/pressure          (sensor_msgs/FluidPressure, Pascal)
- /sara/water_detect_1    (std_msgs/Bool)
- /sara/water_detect_2    (std_msgs/Bool)
- /sara/imu/data          (sensor_msgs/Imu)
- /sara/sensors/status    (diagnostic_msgs/DiagnosticStatus)

Mimari sınır
------------
Bu düğüm:
- Derinlik, roll, pitch, heading veya yaklaşık konum hesaplamaz.
- Motor, servo, ESC veya başka bir eyleyiciyi sürmez.
- Yalnızca sensör verisini okur, temel doğrulama/filtreleme yapar ve yayınlar.

Basınçtan derinlik hesabı navigation_node içinde yapılır:
    h = (P - P0) / (rho * g)
"""

from __future__ import annotations

import math
import statistics
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Bool


# Donanım kütüphanelerini birbirinden bağımsız yükle.
# Böylece basınç kütüphanesi eksikse IMU köprüsü yine çalışabilir.
try:
    from adafruit_extended_bus import ExtendedI2C
    from adafruit_ads1x15 import ADS1115, AnalogIn, ads1x15

    PRESSURE_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - yalnız gerçek donanımda oluşur
    ExtendedI2C = None
    ADS1115 = None
    AnalogIn = None
    ads1x15 = None
    PRESSURE_IMPORT_ERROR = exc

try:
    import Jetson.GPIO as GPIO

    GPIO_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - yalnız gerçek donanımda oluşur
    GPIO = None
    GPIO_IMPORT_ERROR = exc


class ConsecutiveBooleanFilter:
    """
    Boolean girişin değiştiğini kabul etmek için aynı yeni değerin
    belirli sayıda art arda görülmesini bekler.

    Böylece SEN0368 üzerindeki kısa süreli seviye sıçramaları doğrudan
    yüzey kararına yansımaz.
    """

    def __init__(self, required_count: int):
        self.required_count = max(1, int(required_count))
        self.stable_value: Optional[bool] = None
        self._candidate: Optional[bool] = None
        self._candidate_count = 0

    def update(self, raw_value: bool) -> Optional[bool]:
        raw_value = bool(raw_value)

        if self.stable_value is not None and raw_value == self.stable_value:
            self._candidate = None
            self._candidate_count = 0
            return self.stable_value

        if raw_value == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = raw_value
            self._candidate_count = 1

        if self._candidate_count >= self.required_count:
            self.stable_value = raw_value
            self._candidate = None
            self._candidate_count = 0

        return self.stable_value


class SensorDataNode(Node):

    def __init__(self):
        super().__init__('sensor_data_node')

        # ==============================================================
        # Genel etkinleştirme parametreleri
        # ==============================================================
        self.declare_parameter('pressure_enabled', True)
        self.declare_parameter('water_sensors_enabled', True)
        self.declare_parameter('imu_bridge_enabled', True)

        self.pressure_enabled = bool(
            self.get_parameter('pressure_enabled').value
        )
        self.water_enabled = bool(
            self.get_parameter('water_sensors_enabled').value
        )
        self.imu_bridge_enabled = bool(
            self.get_parameter('imu_bridge_enabled').value
        )

        # ==============================================================
        # Basınç sensörü / ADS1115 parametreleri
        # ==============================================================
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('ads1115_address', 0x48)
        self.declare_parameter('ads1115_gain', 1)
        self.declare_parameter('ads1115_data_rate', 860)

        self.declare_parameter('pressure_publish_rate_hz', 20.0)
        self.declare_parameter('pressure_sample_count', 16)
        self.declare_parameter('pressure_filter_alpha', 0.50)
        self.declare_parameter('pressure_retry_sec', 2.0)
        self.declare_parameter('pressure_timeout_sec', 2.0)

        # Gerilim bölücü:
        # SEN0257 çıkışı -> R1 -> ADS1115 A0 -> R2 -> GND
        self.declare_parameter('divider_r1_ohm', 10000.0)
        self.declare_parameter('divider_r2_ohm', 20000.0)

        # SEN0257 nominal karakteristiği:
        # 0.5 V -> 0 Pa, 4.5 V -> 1.600.000 Pa
        self.declare_parameter('sensor_zero_voltage_v', 0.5)
        self.declare_parameter('sensor_slope_pa_per_v', 400000.0)

        # Açık devre, kısa devre veya yanlış bağlantı tespiti için sınırlar.
        self.declare_parameter('sensor_voltage_min_valid_v', 0.10)
        self.declare_parameter('sensor_voltage_max_valid_v', 4.90)
        self.declare_parameter('pressure_frame_id', 'pressure_sensor_link')

        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.ads_address = int(
            self.get_parameter('ads1115_address').value
        )
        self.ads_gain = int(self.get_parameter('ads1115_gain').value)
        self.ads_data_rate = int(
            self.get_parameter('ads1115_data_rate').value
        )

        self.pressure_rate_hz = max(
            1.0,
            float(self.get_parameter('pressure_publish_rate_hz').value)
        )
        self.pressure_sample_count = max(
            1,
            int(self.get_parameter('pressure_sample_count').value)
        )
        self.pressure_alpha = float(
            self.get_parameter('pressure_filter_alpha').value
        )
        self.pressure_alpha = max(0.0, min(1.0, self.pressure_alpha))

        self.pressure_retry_sec = max(
            0.5,
            float(self.get_parameter('pressure_retry_sec').value)
        )
        self.pressure_timeout_sec = max(
            0.1,
            float(self.get_parameter('pressure_timeout_sec').value)
        )

        divider_r1 = float(
            self.get_parameter('divider_r1_ohm').value
        )
        divider_r2 = float(
            self.get_parameter('divider_r2_ohm').value
        )
        if divider_r2 <= 0.0:
            raise ValueError('divider_r2_ohm sıfırdan büyük olmalıdır.')

        self.sensor_voltage_multiplier = (
            divider_r1 + divider_r2
        ) / divider_r2

        self.sensor_zero_voltage = float(
            self.get_parameter('sensor_zero_voltage_v').value
        )
        self.sensor_slope_pa_per_v = float(
            self.get_parameter('sensor_slope_pa_per_v').value
        )
        self.sensor_voltage_min = float(
            self.get_parameter('sensor_voltage_min_valid_v').value
        )
        self.sensor_voltage_max = float(
            self.get_parameter('sensor_voltage_max_valid_v').value
        )
        self.pressure_frame_id = str(
            self.get_parameter('pressure_frame_id').value
        )

        # ==============================================================
        # SEN0368 parametreleri
        # ==============================================================
        self.declare_parameter('water_sensor_1_pin_board', 15)
        self.declare_parameter('water_sensor_2_pin_board', 16)
        self.declare_parameter('water_sensor_1_active_high', True)
        self.declare_parameter('water_sensor_2_active_high', True)
        self.declare_parameter('water_confirm_count', 2)
        self.declare_parameter('water_publish_rate_hz', 20.0)
        self.declare_parameter('water_retry_sec', 2.0)
        self.declare_parameter('water_timeout_sec', 2.0)

        self.water_1_pin = int(
            self.get_parameter('water_sensor_1_pin_board').value
        )
        self.water_2_pin = int(
            self.get_parameter('water_sensor_2_pin_board').value
        )
        if self.water_1_pin == self.water_2_pin:
            raise ValueError('İki SEN0368 aynı GPIO pinini kullanamaz.')

        self.water_1_active_high = bool(
            self.get_parameter('water_sensor_1_active_high').value
        )
        self.water_2_active_high = bool(
            self.get_parameter('water_sensor_2_active_high').value
        )

        water_confirm_count = max(
            1,
            int(self.get_parameter('water_confirm_count').value)
        )
        self.water_1_filter = ConsecutiveBooleanFilter(
            water_confirm_count
        )
        self.water_2_filter = ConsecutiveBooleanFilter(
            water_confirm_count
        )

        self.water_rate_hz = max(
            1.0,
            float(self.get_parameter('water_publish_rate_hz').value)
        )
        self.water_retry_sec = max(
            0.5,
            float(self.get_parameter('water_retry_sec').value)
        )
        self.water_timeout_sec = max(
            0.1,
            float(self.get_parameter('water_timeout_sec').value)
        )

        # ==============================================================
        # IMU köprüsü ve durum parametreleri
        # ==============================================================
        self.declare_parameter('imu_input_topic', '/mavros/imu/data')
        self.declare_parameter('imu_output_topic', '/sara/imu/data')
        self.declare_parameter('imu_timeout_sec', 1.0)
        self.declare_parameter('status_publish_rate_hz', 2.0)

        self.imu_input_topic = str(
            self.get_parameter('imu_input_topic').value
        )
        self.imu_output_topic = str(
            self.get_parameter('imu_output_topic').value
        )
        self.imu_timeout_sec = max(
            0.1,
            float(self.get_parameter('imu_timeout_sec').value)
        )
        status_rate_hz = max(
            0.5,
            float(self.get_parameter('status_publish_rate_hz').value)
        )

        # ==============================================================
        # ROS2 yayıncıları / abonelikleri
        # ==============================================================
        self._pressure_pub = self.create_publisher(
            FluidPressure,
            '/sara/pressure',
            qos_profile_sensor_data
        )
        self._water_1_pub = self.create_publisher(
            Bool,
            '/sara/water_detect_1',
            qos_profile_sensor_data
        )
        self._water_2_pub = self.create_publisher(
            Bool,
            '/sara/water_detect_2',
            qos_profile_sensor_data
        )
        self._imu_pub = self.create_publisher(
            Imu,
            self.imu_output_topic,
            qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            DiagnosticStatus,
            '/sara/sensors/status',
            10
        )

        if self.imu_bridge_enabled:
            self._imu_sub = self.create_subscription(
                Imu,
                self.imu_input_topic,
                self._on_imu,
                qos_profile_sensor_data
            )
        else:
            self._imu_sub = None

        # ==============================================================
        # Donanım ve çalışma durumu
        # ==============================================================
        self._i2c = None
        self._ads = None
        self._pressure_channel = None
        self._pressure_ready = False
        self._pressure_filtered_voltage: Optional[float] = None
        self._next_pressure_retry_monotonic = 0.0

        self._gpio_ready = False
        self._next_gpio_retry_monotonic = 0.0

        self._last_pressure_time = None
        self._last_water_1_time = None
        self._last_water_2_time = None
        self._last_imu_time = None

        self._last_pressure_pa: Optional[float] = None
        self._last_pressure_variance: Optional[float] = None
        self._last_sensor_voltage: Optional[float] = None
        self._last_adc_voltage: Optional[float] = None

        self._water_1_raw: Optional[bool] = None
        self._water_2_raw: Optional[bool] = None
        self._water_1_stable: Optional[bool] = None
        self._water_2_stable: Optional[bool] = None

        # İlk donanım başlatma denemeleri.
        if self.pressure_enabled:
            self._initialize_pressure_hardware()

        if self.water_enabled:
            self._initialize_gpio()

        # Timer'lar birbirinden bağımsızdır. Bir sensör hatası diğer yayınları
        # durdurmaz.
        self._pressure_timer = self.create_timer(
            1.0 / self.pressure_rate_hz,
            self._pressure_timer_callback
        )
        self._water_timer = self.create_timer(
            1.0 / self.water_rate_hz,
            self._water_timer_callback
        )
        self._status_timer = self.create_timer(
            1.0 / status_rate_hz,
            self._status_timer_callback
        )

        self.get_logger().info(
            'sensor_data_node başlatıldı. '
            f'Basınç=/dev/i2c-{self.i2c_bus}, ADS1115=0x{self.ads_address:02X}, '
            f'SEN0368 pinleri=BOARD {self.water_1_pin}/{self.water_2_pin}, '
            f'IMU={self.imu_input_topic} -> {self.imu_output_topic}'
        )

    # ==================================================================
    # Basınç sensörü
    # ==================================================================
    def _initialize_pressure_hardware(self) -> None:
        if not self.pressure_enabled:
            return

        if PRESSURE_IMPORT_ERROR is not None:
            self._pressure_ready = False
            self._next_pressure_retry_monotonic = (
                time.monotonic() + 30.0
            )
            self.get_logger().error(
                'Basınç sensörü kütüphaneleri yüklenemedi: '
                f'{PRESSURE_IMPORT_ERROR}'
            )
            return

        self._close_pressure_hardware()

        try:
            self._i2c = ExtendedI2C(self.i2c_bus)

            lock_deadline = time.monotonic() + 1.0
            while not self._i2c.try_lock():
                if time.monotonic() >= lock_deadline:
                    raise TimeoutError(
                        f'/dev/i2c-{self.i2c_bus} kilidi alınamadı.'
                    )
                time.sleep(0.01)

            try:
                devices = self._i2c.scan()
            finally:
                self._i2c.unlock()

            if self.ads_address not in devices:
                found = ', '.join(
                    f'0x{address:02X}' for address in devices
                ) or 'cihaz yok'
                raise RuntimeError(
                    f'ADS1115 0x{self.ads_address:02X} bulunamadı. '
                    f'Bulunan adresler: {found}'
                )

            self._ads = ADS1115(
                self._i2c,
                address=self.ads_address
            )
            self._ads.gain = self.ads_gain
            self._ads.data_rate = self.ads_data_rate

            self._pressure_channel = AnalogIn(
                self._ads,
                ads1x15.Pin.A0
            )

            self._pressure_filtered_voltage = None
            self._pressure_ready = True

            self.get_logger().info(
                f'SEN0257/ADS1115 hazır: /dev/i2c-{self.i2c_bus}, '
                f'adres=0x{self.ads_address:02X}, kanal=A0'
            )

        except Exception as exc:
            self._pressure_ready = False
            self._next_pressure_retry_monotonic = (
                time.monotonic() + self.pressure_retry_sec
            )
            self._close_pressure_hardware()
            self.get_logger().error(
                f'Basınç sensörü başlatma hatası: {exc}'
            )

    def _close_pressure_hardware(self) -> None:
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass

        self._pressure_channel = None
        self._ads = None
        self._i2c = None

    def _read_pressure_sample(self) -> tuple[float, float]:
        """
        Bir ADC örneği döndürür:
        (adc_gerilimi, gerilim_bölücü_öncesi_sensor_gerilimi)
        """
        if self._pressure_channel is None:
            raise RuntimeError('ADS1115 A0 kanalı hazır değil.')

        adc_voltage = float(self._pressure_channel.voltage)
        sensor_voltage = (
            adc_voltage * self.sensor_voltage_multiplier
        )

        if not math.isfinite(sensor_voltage):
            raise ValueError('Sensör gerilimi sonlu bir sayı değil.')

        if not (
            self.sensor_voltage_min
            <= sensor_voltage
            <= self.sensor_voltage_max
        ):
            raise ValueError(
                f'SEN0257 gerilimi geçerli aralık dışında: '
                f'{sensor_voltage:.3f} V'
            )

        return adc_voltage, sensor_voltage

    def _pressure_timer_callback(self) -> None:
        if not self.pressure_enabled:
            return

        if not self._pressure_ready:
            if (
                time.monotonic()
                >= self._next_pressure_retry_monotonic
            ):
                self._initialize_pressure_hardware()
            return

        try:
            adc_samples = []
            sensor_voltage_samples = []

            for _ in range(self.pressure_sample_count):
                adc_voltage, sensor_voltage = (
                    self._read_pressure_sample()
                )
                adc_samples.append(adc_voltage)
                sensor_voltage_samples.append(sensor_voltage)

            average_adc_voltage = (
                sum(adc_samples) / len(adc_samples)
            )
            average_sensor_voltage = (
                sum(sensor_voltage_samples)
                / len(sensor_voltage_samples)
            )

            if self._pressure_filtered_voltage is None:
                self._pressure_filtered_voltage = (
                    average_sensor_voltage
                )
            else:
                self._pressure_filtered_voltage += (
                    self.pressure_alpha
                    * (
                        average_sensor_voltage
                        - self._pressure_filtered_voltage
                    )
                )

            pressure_pa = (
                self._pressure_filtered_voltage
                - self.sensor_zero_voltage
            ) * self.sensor_slope_pa_per_v

            # FluidPressure.variance birimi Pa²'dir.
            pressure_variance = (
                statistics.pvariance(sensor_voltage_samples)
                * self.sensor_slope_pa_per_v ** 2
            )

            msg = FluidPressure()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.pressure_frame_id
            msg.fluid_pressure = float(pressure_pa)
            msg.variance = float(pressure_variance)

            self._pressure_pub.publish(msg)

            self._last_pressure_time = self.get_clock().now()
            self._last_pressure_pa = pressure_pa
            self._last_pressure_variance = pressure_variance
            self._last_sensor_voltage = (
                self._pressure_filtered_voltage
            )
            self._last_adc_voltage = average_adc_voltage

        except Exception as exc:
            self._pressure_ready = False
            self._next_pressure_retry_monotonic = (
                time.monotonic() + self.pressure_retry_sec
            )
            self._close_pressure_hardware()
            self.get_logger().error(
                f'Basınç sensörü okuma hatası; yeniden bağlanılacak: {exc}'
            )

    # ==================================================================
    # SEN0368 sensörleri
    # ==================================================================
    def _initialize_gpio(self) -> None:
        if not self.water_enabled:
            return

        if GPIO_IMPORT_ERROR is not None:
            self._gpio_ready = False
            self._next_gpio_retry_monotonic = (
                time.monotonic() + 30.0
            )
            self.get_logger().error(
                f'Jetson.GPIO yüklenemedi: {GPIO_IMPORT_ERROR}'
            )
            return

        try:
            GPIO.setwarnings(False)

            current_mode = GPIO.getmode()
            if current_mode is None:
                GPIO.setmode(GPIO.BOARD)
            elif current_mode != GPIO.BOARD:
                raise RuntimeError(
                    'Jetson.GPIO başka bir numaralandırma modunda. '
                    'SEN0368 kodu GPIO.BOARD bekliyor.'
                )

            GPIO.setup(self.water_1_pin, GPIO.IN)
            GPIO.setup(self.water_2_pin, GPIO.IN)

            self._gpio_ready = True

            self.get_logger().info(
                f'SEN0368 GPIO hazır: BOARD '
                f'{self.water_1_pin} ve {self.water_2_pin}'
            )

        except Exception as exc:
            self._gpio_ready = False
            self._next_gpio_retry_monotonic = (
                time.monotonic() + self.water_retry_sec
            )
            self.get_logger().error(
                f'SEN0368 GPIO başlatma hatası: {exc}'
            )

    @staticmethod
    def _raw_level_to_water(
        raw_level: int,
        active_high: bool
    ) -> bool:
        if active_high:
            return bool(raw_level == GPIO.HIGH)
        return bool(raw_level == GPIO.LOW)

    def _water_timer_callback(self) -> None:
        if not self.water_enabled:
            return

        if not self._gpio_ready:
            if (
                time.monotonic()
                >= self._next_gpio_retry_monotonic
            ):
                self._initialize_gpio()
            return

        try:
            raw_level_1 = GPIO.input(self.water_1_pin)
            raw_level_2 = GPIO.input(self.water_2_pin)

            raw_water_1 = self._raw_level_to_water(
                raw_level_1,
                self.water_1_active_high
            )
            raw_water_2 = self._raw_level_to_water(
                raw_level_2,
                self.water_2_active_high
            )

            self._water_1_raw = raw_water_1
            self._water_2_raw = raw_water_2

            self._water_1_stable = self.water_1_filter.update(
                raw_water_1
            )
            self._water_2_stable = self.water_2_filter.update(
                raw_water_2
            )

            now = self.get_clock().now()

            if self._water_1_stable is not None:
                msg_1 = Bool()
                msg_1.data = bool(self._water_1_stable)
                self._water_1_pub.publish(msg_1)
                self._last_water_1_time = now

            if self._water_2_stable is not None:
                msg_2 = Bool()
                msg_2.data = bool(self._water_2_stable)
                self._water_2_pub.publish(msg_2)
                self._last_water_2_time = now

        except Exception as exc:
            self._gpio_ready = False
            self._next_gpio_retry_monotonic = (
                time.monotonic() + self.water_retry_sec
            )
            self.get_logger().error(
                f'SEN0368 okuma hatası; GPIO yeniden başlatılacak: {exc}'
            )

    # ==================================================================
    # Pixhawk IMU köprüsü
    # ==================================================================
    def _on_imu(self, msg: Imu) -> None:
        """
        MAVROS'tan gelen standart sensor_msgs/Imu mesajını Sensör Veri
        Alma Katmanı çıkışına aktarır. Yönelim hesabı navigation_node'dadır.
        """
        self._last_imu_time = self.get_clock().now()
        self._imu_pub.publish(msg)

    # ==================================================================
    # DiagnosticStatus
    # ==================================================================
    def _is_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False

        age = (
            self.get_clock().now() - stamp
        ).nanoseconds * 1e-9

        return age < timeout_sec

    @staticmethod
    def _kv(key: str, value) -> KeyValue:
        item = KeyValue()
        item.key = str(key)
        item.value = str(value)
        return item

    @staticmethod
    def _optional_text(value, fmt: str = '{}') -> str:
        if value is None:
            return 'bekleniyor'
        return fmt.format(value)

    def _status_timer_callback(self) -> None:
        pressure_fresh = (
            not self.pressure_enabled
            or self._is_fresh(
                self._last_pressure_time,
                self.pressure_timeout_sec
            )
        )
        water_1_fresh = (
            not self.water_enabled
            or self._is_fresh(
                self._last_water_1_time,
                self.water_timeout_sec
            )
        )
        water_2_fresh = (
            not self.water_enabled
            or self._is_fresh(
                self._last_water_2_time,
                self.water_timeout_sec
            )
        )
        imu_fresh = (
            not self.imu_bridge_enabled
            or self._is_fresh(
                self._last_imu_time,
                self.imu_timeout_sec
            )
        )

        errors = []
        warnings = []

        if self.pressure_enabled:
            if PRESSURE_IMPORT_ERROR is not None:
                errors.append('basınç kütüphaneleri yok')
            elif not self._pressure_ready:
                errors.append('ADS1115/SEN0257 hazır değil')
            elif not pressure_fresh:
                warnings.append('basınç verisi bekleniyor')

        if self.water_enabled:
            if GPIO_IMPORT_ERROR is not None:
                errors.append('Jetson.GPIO yok')
            elif not self._gpio_ready:
                errors.append('SEN0368 GPIO hazır değil')
            else:
                if not water_1_fresh:
                    warnings.append('SEN0368-1 verisi bekleniyor')
                if not water_2_fresh:
                    warnings.append('SEN0368-2 verisi bekleniyor')

        if self.imu_bridge_enabled and not imu_fresh:
            errors.append('Pixhawk IMU verisi yok/zaman aşımı')

        status = DiagnosticStatus()
        status.name = 'sara_sensor_data'
        status.hardware_id = 'jetson_orin_nano'

        if errors:
            status.level = DiagnosticStatus.ERROR
            status.message = '; '.join(errors + warnings)
        elif warnings:
            status.level = DiagnosticStatus.WARN
            status.message = '; '.join(warnings)
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Tüm etkin sensörler nominal'

        status.values = [
            self._kv('pressure_enabled', self.pressure_enabled),
            self._kv('pressure_hardware_ready', self._pressure_ready),
            self._kv('pressure_fresh', pressure_fresh),
            self._kv(
                'pressure_pa',
                self._optional_text(
                    self._last_pressure_pa,
                    '{:.3f}'
                )
            ),
            self._kv(
                'pressure_variance_pa2',
                self._optional_text(
                    self._last_pressure_variance,
                    '{:.3f}'
                )
            ),
            self._kv(
                'adc_voltage_v',
                self._optional_text(
                    self._last_adc_voltage,
                    '{:.6f}'
                )
            ),
            self._kv(
                'sensor_voltage_v',
                self._optional_text(
                    self._last_sensor_voltage,
                    '{:.6f}'
                )
            ),
            self._kv('water_sensors_enabled', self.water_enabled),
            self._kv('gpio_ready', self._gpio_ready),
            self._kv('water_1_fresh', water_1_fresh),
            self._kv('water_2_fresh', water_2_fresh),
            self._kv('water_1_raw', self._water_1_raw),
            self._kv('water_2_raw', self._water_2_raw),
            self._kv('water_1_stable', self._water_1_stable),
            self._kv('water_2_stable', self._water_2_stable),
            self._kv('imu_bridge_enabled', self.imu_bridge_enabled),
            self._kv('imu_input_topic', self.imu_input_topic),
            self._kv('imu_output_topic', self.imu_output_topic),
            self._kv('imu_fresh', imu_fresh),
        ]

        self._status_pub.publish(status)

    # ==================================================================
    # Kapanış
    # ==================================================================
    def destroy_node(self):
        self._close_pressure_hardware()

        if GPIO is not None and self._gpio_ready:
            # Yalnız bu düğümün kullandığı pinleri temizlemeyi dene.
            for pin in (self.water_1_pin, self.water_2_pin):
                try:
                    GPIO.cleanup(pin)
                except Exception:
                    pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataNode()

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