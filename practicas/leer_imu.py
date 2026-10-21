#!/usr/bin/env python3
"""Práctica 1 · IMU: lee aceleraciones y velocidades angulares en crudo.

Tópico: /auto/imu/data  (sensor_msgs/msg/Imu)
Ejercicio: estima el sesgo del giroscopio con el vehículo detenido
(promedio de gz) y compáralo cuando el profesor inyecte la falla B1A04.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class LectorIMU(Node):
    def __init__(self):
        super().__init__('lector_imu')
        self.create_subscription(Imu, '/auto/imu/data', self.cb, 10)
        self.n, self.suma_gz = 0, 0.0
        print(f'{"t [s]":>9} {"ax":>8} {"ay":>8} {"az":>8} {"gx":>8} {"gy":>8} {"gz":>8}   sesgo_gz')

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a, g = m.linear_acceleration, m.angular_velocity
        self.n += 1
        self.suma_gz += g.z
        if self.n % 5 == 0:
            print(f'{t:9.2f} {a.x:8.3f} {a.y:8.3f} {a.z:8.3f} {g.x:8.4f} {g.y:8.4f} {g.z:8.4f}   '
                  f'{self.suma_gz / self.n:+.5f}')


rclpy.init()
try:
    rclpy.spin(LectorIMU())
except KeyboardInterrupt:
    pass
