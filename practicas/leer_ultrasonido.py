#!/usr/bin/env python3
"""Práctica 5 · Ultrasonido: 8 sensores de estacionamiento (sensor_msgs/msg/Range).

Tópicos: /auto/ultrasonido_<posición>/range   (inf = libre, nan = sensor sin respuesta)
Ejercicio: genera un aviso sonoro (texto) más rápido conforme baja la distancia.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

POS = ['del_izq_ext', 'del_izq', 'del_der', 'del_der_ext', 'tras_izq_ext', 'tras_izq', 'tras_der', 'tras_der_ext']


class LectorUS(Node):
    def __init__(self):
        super().__init__('lector_ultrasonido')
        self.d = {p: None for p in POS}
        for p in POS:
            self.create_subscription(Range, f'/auto/ultrasonido_{p}/range', lambda m, p=p: self.cb(m, p), 10)
        self.create_timer(0.5, self.mostrar)

    def cb(self, m, p):
        self.d[p] = m.range

    def mostrar(self):
        def f(v):
            if v is None:
                return '  —  '
            if math.isnan(v):
                return 'FALLA'
            return ' libre' if math.isinf(v) else f'{v:5.2f}'
        print('DEL: ' + ' '.join(f(self.d[p]) for p in POS[:4]) + '  |  TRAS: ' + ' '.join(f(self.d[p]) for p in POS[4:]))


rclpy.init()
try:
    rclpy.spin(LectorUS())
except KeyboardInterrupt:
    pass
