#!/usr/bin/env python3
"""Práctica 3 · LiDAR: nube de puntos a distancias por sector (frente, lados, atrás).

Tópico: /auto/lidar/points  (sensor_msgs/msg/PointCloud2: x, y, z, intensity)
Ejercicio: filtra el piso (z) y detecta el obstáculo más cercano al frente.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

SECTORES = {'frente': (-30, 30), 'izquierda': (30, 150), 'atrás': (150, 210), 'derecha': (210, 330)}


class LectorLidar(Node):
    def __init__(self):
        super().__init__('lector_lidar')
        self.create_subscription(PointCloud2, '/auto/lidar/points', self.cb, 5)
        self.primero = True

    def cb(self, m):
        pts = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        if self.primero:
            print(f'Campos: {[f.name for f in m.fields]}  ·  {m.width} puntos por barrido')
            print('Primeros 5 puntos (x, y, z):\n', np.round(pts[:5], 2))
            self.primero = False
        pts = pts[pts[:, 2] > -1.7]                     # quitar el piso (sensor a 2.1 m)
        ang = (np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) + 360) % 360
        dist = np.hypot(pts[:, 0], pts[:, 1])
        linea = []
        for nombre, (a, b) in SECTORES.items():
            sel = ((ang >= a % 360) & (ang < b % 360)) if a >= 0 else ((ang >= a + 360) | (ang < b))
            d = dist[sel].min() if sel.any() else float('inf')
            linea.append(f'{nombre} {d:5.1f} m')
        print(f'{len(pts):5d} pts | ' + ' | '.join(linea))


rclpy.init()
try:
    rclpy.spin(LectorLidar())
except KeyboardInterrupt:
    pass
