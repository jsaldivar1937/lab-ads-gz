#!/usr/bin/env python3
"""Eco crudo de cualquier tópico (como 'ros2 topic echo', pero desde Python).

Uso: python3 eco_crudo.py /auto/gnss/fix
     python3 eco_crudo.py /auto/lidar/points 3     (muestra 3 mensajes y termina)
"""
import sys

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_yaml
from rosidl_runtime_py.utilities import get_message

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)
TOPICO = sys.argv[1]
LIMITE = int(sys.argv[2]) if len(sys.argv) > 2 else None

rclpy.init()
nodo = Node('eco_crudo')
for _ in range(20):
    tipos = dict(nodo.get_topic_names_and_types())
    if TOPICO in tipos:
        break
    rclpy.spin_once(nodo, timeout_sec=0.25)
else:
    print(f'No encuentro {TOPICO}. Revisa con: ros2 topic list')
    sys.exit(1)
tipo = tipos[TOPICO][0]
print(f'# {TOPICO} [{tipo}]')
cuenta = 0


def cb(m):
    global cuenta
    cuenta += 1
    print(f'--- mensaje {cuenta}')
    print(message_to_yaml(m, truncate_length=16))
    if LIMITE and cuenta >= LIMITE:
        raise SystemExit


nodo.create_subscription(get_message(tipo), TOPICO, cb, 10)
try:
    rclpy.spin(nodo)
except (KeyboardInterrupt, SystemExit):
    pass
