#!/usr/bin/env python3
"""Práctica 7 · Odometría por encoders (dead reckoning) contra la posición real.

Tópicos: /auto/encoders/joint_states (velocidad angular de cada rueda, rad/s)
         /auto/ground_truth/odom     (posición real, para comparar)
Modelo diferencial con las ruedas traseras: v = r(wi+wd)/2, w = r(wd-wi)/via
Ejercicio: ajusta RADIO y VIA para reducir la deriva; observa la falla C0035.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

RADIO, VIA = 0.33, 1.6


class Odometria(Node):
    def __init__(self):
        super().__init__('odometria_encoders')
        self.x = self.y = self.th = None
        self.t = None
        self.real = None
        self.create_subscription(JointState, '/auto/encoders/joint_states', self.cb, 20)
        self.create_subscription(Odometry, '/auto/ground_truth/odom', self.cb_real, 10)

    def cb_real(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.real = (p.x, p.y, math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z))
        if self.x is None:
            self.x, self.y, self.th = self.real

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t is None or self.x is None:
            self.t = t
            return
        dt, self.t = t - self.t, t
        wi, wd = m.velocity[2], m.velocity[3]
        v, w = RADIO * (wi + wd) / 2, RADIO * (wd - wi) / VIA
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt
        if self.real and int(t * 10) % 5 == 0:
            e = math.hypot(self.x - self.real[0], self.y - self.real[1])
            print(f'v {v:5.2f} m/s · estimado ({self.x:7.2f}, {self.y:7.2f}) · real ({self.real[0]:7.2f}, '
                  f'{self.real[1]:7.2f}) · deriva {e:5.2f} m')


rclpy.init()
try:
    rclpy.spin(Odometria())
except KeyboardInterrupt:
    pass
