#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


# Aşama-1 görev durumları
WAITING = 0
FORWARD = 1
TURNING = 2
RETURNING = 3
SURFACING = 4
FINISHED = 5


class Navigation(Node):

    def __init__(self):
        super().__init__('navigation')

        # Program ilk açıldığında bekleme durumunda
        self.state = WAITING
        # Bekleme süresinin başladığı an
        self.wait_start_time = self.get_clock().now()
        # Buzzer'ın başlatılıp başlatılmadığını tutar



        self.buzzer_started = False
        # Navigasyon değişkenleri
        self.heading = 0.0
        self.depth = 2.0
        self.distance = 0.0

        # Hedefler
        self.target_heading = 0.0
        self.target_depth = 2.0

        # Simülasyon için hız (m/s)
        self.speed = 2.0

        self.race_started = False
        self.race_start_time = None


        # Navigasyon döngüsü saniyede 10 kez çalışacak
        self.timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info('Navigation node başlatıldı.')

        self.return_distance = 0.0

    def navigation_loop(self):



        if self.state == WAITING:
            elapsed = (
            self.get_clock().now() - self.wait_start_time
            ).nanoseconds / 1e9

            remaining = max(0.0, 60.0 - elapsed)

            self.get_logger().info(
                f'Durum: WAITING | Kalan süre: {remaining:.1f} saniye',
                throttle_duration_sec=2.0
             )

            # Hareketten 10 saniye önce buzzer başlatılacak
            if elapsed >= 50.0 and not self.buzzer_started:
                 self.buzzer_started = True
                 self.get_logger().info(
                'Buzzer başlatılmalı: harekete 10 saniye kaldı.'
            )

            # 60 saniye tamamlanınca ileri hareket durumuna geç
            if elapsed >= 60.0:
                self.state = FORWARD
                self.get_logger().info(
                '60 saniyelik güvenlik beklemesi tamamlandı. FORWARD durumuna geçildi.'
            )

    


        elif self.state == FORWARD:

            # Simülasyon amaçlı mesafe hesabı
            self.distance += self.speed * 0.1

            # Araç 10 metreye ulaştığında yarışma süresi başlar
            if self.distance >= 10.0 and not self.race_started:
                self.race_started = True
                self.race_start_time = self.get_clock().now()

                self.get_logger().info(
                    '10 metre çizgisi geçildi. Yarışma süresi başladı.'
                )

            # Yarışma başladıysa geçen süreyi hesapla
            if self.race_started:
                race_elapsed = (
                    self.get_clock().now() - self.race_start_time
                ).nanoseconds / 1e9

                race_text = f'{race_elapsed:.1f} saniye'
            else:
                race_text = 'Henüz başlamadı'

            self.get_logger().info(
                f'Durum: FORWARD | Mesafe: {self.distance:.1f} m | '
                f'Yarışma süresi: {race_text} | '
                f'Heading: {self.heading:.1f}° | '
                f'Derinlik: {self.depth:.1f} m',
                throttle_duration_sec=1.0
            )

            # Kıyıdan 50 metre uzaklığa ulaşınca dönüşe geç
            if self.distance >= 50.0:
                self.state = TURNING
                self.get_logger().info(
                    '50 metreye ulaşıldı. TURNING durumuna geçildi.'
                )
            # 50 metreye ulaşıldıysa dönüşe geç
            if self.distance >= 50.0:

                self.state = TURNING

                self.target_heading = 180.0

                self.get_logger().info(
                    "50 metreye ulaşıldı. TURNING durumuna geçiliyor."
                )



        elif self.state == TURNING:
            # Simülasyonda heading değerini artırarak dönüşü gösteriyoruz
            self.heading += 5.0

            if self.heading > 360.0:
                self.heading -= 360.0

            self.get_logger().info(
                f'Durum: TURNING | '
                f'Hedef Heading: {self.target_heading:.1f}° | '
                f'Mevcut Heading: {self.heading:.1f}°',
                throttle_duration_sec=1.0
            )

            # Hedef yöne yeterince yaklaşıldığında geri dönüşe geç
            if abs(self.heading - self.target_heading) <= 5.0:
                self.state = RETURNING

                self.get_logger().info(
                    '180 derece dönüş tamamlandı. RETURNING durumuna geçildi.'
                    )
                


        elif self.state == RETURNING:
            # Simülasyonda geri dönüşte gidilen mesafe
            self.return_distance += self.speed * 0.1

            # Kıyıya göre kalan yaklaşık mesafe
            distance_from_shore = 50.0 - self.return_distance

            self.get_logger().info(
                f'Durum: RETURNING | '
                f'Geri gidilen: {self.return_distance:.1f} m | '
                f'Kıyıya uzaklık: {distance_from_shore:.1f} m | '
                f'Heading: {self.heading:.1f}°',
                throttle_duration_sec=1.0
            )

            # 50 metreden 10 metre çizgisine dönmek için 40 metre geri git
            if self.return_distance >= 40.0:
                self.state = SURFACING

                self.get_logger().info(
                    '10 metre başlangıç/bitiş çizgisine ulaşıldı. '
                    'SURFACING durumuna geçildi.'
            )



        elif self.state == SURFACING:
            self.get_logger().info(
                "Durum: SURFACING | Araç yüzeye çıkıyor...",
                throttle_duration_sec=1.0
            )

            # Simülasyonda hemen görevi bitirelim
            self.state = FINISHED

        elif self.state == FINISHED:
            self.get_logger().info(
                """
        ===========================
        GÖREV TAMAMLANDI
        ===========================

        Motor durduruldu.
        Araç yüzeyde.
        Görev başarıyla tamamlandı.
        """,
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)

    node = Navigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()