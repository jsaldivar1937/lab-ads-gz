#!/usr/bin/env python3
"""Práctica 9 · Lector OBD-II: decodifica las respuestas 0x7E8 (modo 01 y modo 03).

Modo 01 PID 0C → RPM = (A·256 + B)/4      PID 0D → velocidad = A km/h
       PID 05 → refrigerante = A − 40 °C  PID 11 → acelerador = A·100/255 %
Modo 03 → códigos de falla (2 bytes por código, SAE J2012)
"""
import rclpy
from can_msgs.msg import Frame
from rclpy.node import Node

PIDS = {0x0C: ('RPM', lambda d: (d[0] * 256 + d[1]) / 4, 'rpm'), 0x0D: ('Velocidad', lambda d: d[0], 'km/h'),
        0x05: ('Refrigerante', lambda d: d[0] - 40, '°C'), 0x11: ('Acelerador', lambda d: d[0] * 100 / 255, '%'),
        0x2F: ('Combustible', lambda d: d[0] * 100 / 255, '%'), 0x42: ('Voltaje', lambda d: (d[0] * 256 + d[1]) / 1000, 'V'),
        0x01: ('MIL/DTC', lambda d: f"MIL {'ON' if d[0] & 0x80 else 'OFF'}, {d[0] & 0x7F} códigos", '')}


def dtc(b1, b2):
    return 'PCBU'[b1 >> 6] + f'{(b1 >> 4) & 3:X}{b1 & 0xF:X}{b2:02X}'


class LectorOBD(Node):
    def __init__(self):
        super().__init__('lector_obd')
        self.create_subscription(Frame, '/auto/can/rx', self.cb, 100)

    def cb(self, f):
        d = [int(b) for b in f.data]
        if f.id == 0x7DF:
            print(f'  → petición modo {d[1]:02X}' + (f' PID {d[2]:02X}' if d[1] == 1 else ''))
        elif f.id == 0x7E8 and d[1] == 0x41 and d[2] in PIDS:
            nombre, fn, u = PIDS[d[2]]
            print(f'  ← {nombre}: {fn(d[3:])} {u}')
        elif f.id == 0x7E8 and d[1] == 0x43:
            cods = [dtc(d[3 + 2 * i], d[4 + 2 * i]) for i in range(min(d[2], 2))]
            print(f'  ← DTC almacenados ({d[2]}): {", ".join(cods) or "ninguno"}')


rclpy.init()
try:
    rclpy.spin(LectorOBD())
except KeyboardInterrupt:
    pass
