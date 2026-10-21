#!/usr/bin/env python3
"""Práctica 6 · Radar: detecciones con distancia y velocidad relativa.

Tópico: /auto/radar_frontal/points (PointCloud2: x, y, z, velocity; velocity < 0 = se acerca)
Ejercicio: calcula el tiempo a colisión (TTC = distancia / -velocidad) del objeto más cercano.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LectorRadar(Node):
    def __init__(self):
        super().__init__('lector_radar')
        self.create_subscription(PointCloud2, '/auto/radar_frontal/points', self.cb, 5)

    def cb(self, m):
        p = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z', 'velocity'))
        if len(p) == 0:
            print('sin detecciones')
            return
        d = np.hypot(p[:, 0], p[:, 1])
        i = int(np.argmin(d))
        v = p[i, 3]
        ttc = d[i] / -v if v < -0.1 else float('inf')
        print(f'{len(p):3d} detecciones | más cercano {d[i]:6.1f} m, vel. rel. {v:+5.1f} m/s, TTC {ttc:5.1f} s')


rclpy.init()
try:
    rclpy.spin(LectorRadar())
except KeyboardInterrupt:
    pass
