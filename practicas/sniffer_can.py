#!/usr/bin/env python3
"""Práctica 8 · Sniffer CAN: tramas crudas y decodificadas con el archivo DBC.

Tópico: /auto/can/rx (can_msgs/msg/Frame)   ·   DBC: /lab/config/lab_ads.dbc
Uso: python3 sniffer_can.py            (todas)
     python3 sniffer_can.py 0x1F0      (solo un ID)
Ejercicio: localiza en los bytes crudos la señal VehicleSpeed (factor 0.01, little endian).
"""
import sys

import cantools
import rclpy
from can_msgs.msg import Frame
from rclpy.node import Node

db = cantools.database.load_file('/lab/config/lab_ads.dbc')
FILTRO = int(sys.argv[1], 16) if len(sys.argv) > 1 else None


class Sniffer(Node):
    def __init__(self):
        super().__init__('sniffer_can')
        self.create_subscription(Frame, '/auto/can/rx', self.cb, 100)

    def cb(self, f):
        if FILTRO is not None and f.id != FILTRO:
            return
        datos = bytes(f.data[:f.dlc])
        t = f.header.stamp.sec + f.header.stamp.nanosec * 1e-9
        try:
            m = db.get_message_by_frame_id(f.id)
            señales = m.decode(datos)
            texto = f'{m.name}: ' + ', '.join(f'{k}={v:.2f}' if isinstance(v, float) else f'{k}={v}'
                                               for k, v in señales.items())
        except KeyError:
            texto = 'OBD-II (ver lector_obd.py)' if f.id in (0x7DF, 0x7E8) else 'desconocido'
        print(f'{t:10.3f}  {f.id:03X}  [{f.dlc}]  {datos.hex(" ").upper():23s}  {texto}')


rclpy.init()
try:
    rclpy.spin(Sniffer())
except KeyboardInterrupt:
    pass
