#!/usr/bin/env python3
"""Práctica 2 · GNSS: convierte latitud/longitud a coordenadas locales (x este, y norte).

Tópico: /auto/gnss/fix  (sensor_msgs/msg/NavSatFix)
Se compara con la posición real (/auto/ground_truth/odom) para ver el error.
Ejercicio: calcula la desviación estándar del error y filtra con un promedio móvil.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

R_TIERRA = 6378137.0


class LectorGNSS(Node):
    def __init__(self):
        super().__init__('lector_gnss')
        self.origen = None
        self.real = None
        self.create_subscription(NavSatFix, '/auto/gnss/fix', self.cb, 10)
        self.create_subscription(Odometry, '/auto/ground_truth/odom', self.cb_real, 10)

    def cb_real(self, m):
        self.real = (m.pose.pose.position.x, m.pose.pose.position.y)

    def cb(self, m):
        if m.status.status < 0 or math.isnan(m.latitude):
            print('SIN SEÑAL GNSS (¿falla B1A03?)')
            return
        if self.origen is None and self.real is not None:
            # el primer dato se alinea con la posición real para comparar
            self.origen = (m.latitude, m.longitude, self.real)
            return
        if self.origen is None:
            return
        lat0, lon0, (x0, y0) = self.origen
        x = x0 + math.radians(m.longitude - lon0) * R_TIERRA * math.cos(math.radians(lat0))
        y = y0 + math.radians(m.latitude - lat0) * R_TIERRA
        ex, ey = x - self.real[0], y - self.real[1]
        print(f'lat {m.latitude:.7f} lon {m.longitude:.7f} → x {x:8.2f} y {y:8.2f} m | '
              f'error {math.hypot(ex, ey):5.2f} m')


rclpy.init()
try:
    rclpy.spin(LectorGNSS())
except KeyboardInterrupt:
    pass
