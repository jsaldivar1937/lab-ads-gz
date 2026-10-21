#!/usr/bin/env python3
"""Puente entre dominios ROS 2 del laboratorio (7) y de prácticas (8).

7 → 8  Copia de SOLO LECTURA de los sensores (lista fija en generar_puente.py).
8 → 7  Solo los tópicos que publican los alumnos bajo /alumnos/<nombre>/...
       para verlos en la página. Ningún nodo del simulador escucha /alumnos/,
       así que desde la terminal no se puede mover el vehículo.

Los mensajes se reenvían serializados (sin decodificar): funciona con cualquier tipo.
"""
import os
import re
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from generar_puente import topicos_practicas

ORIGEN = int(os.environ.get('ROS_DOMAIN_ID_LAB', 7))

# Tópicos de alumnos que suben a la página
PATRON_ALUMNOS = re.compile(r'^/alumnos/[a-z0-9_]{1,40}(/[A-Za-z0-9_]{1,40}){1,3}$')
MAX_TOPICOS_ALUMNOS = 80
HZ_MAX_ALUMNOS = 15.0
BYTES_MAX_ALUMNOS = 4 * 1024 * 1024
OLVIDAR_S = 10.0          # sin publicador durante este tiempo → se quita


class SubidaAlumnos:
    """Descubre tópicos /alumnos/... en el dominio 8 y los reenvía al dominio 7."""

    def __init__(self, nodo_practicas, nodo_lab):
        self.np, self.nl = nodo_practicas, nodo_lab
        self.activos = {}      # tópico → {sub, pub, ultimo, sin_pub}
        self.qos_sub = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        self.qos_pub = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST)
        self.np.create_timer(2.0, self.revisar)

    def revisar(self):
        ahora = time.monotonic()
        for topico, tipos in self.np.get_topic_names_and_types():
            if topico in self.activos or not PATRON_ALUMNOS.match(topico):
                continue
            if len(self.activos) >= MAX_TOPICOS_ALUMNOS or self.np.count_publishers(topico) == 0:
                continue
            try:
                clase = get_message(tipos[0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                continue
            e = {'ultimo': 0.0, 'sin_pub': None}
            e['pub'] = self.nl.create_publisher(clase, topico, self.qos_pub)
            e['sub'] = self.np.create_subscription(clase, topico, lambda d, e=e: self.reenviar(d, e),
                                                   self.qos_sub, raw=True)
            self.activos[topico] = e
            print(f'[puente] alumno → página: {topico} ({tipos[0]})', flush=True)

        for topico, e in list(self.activos.items()):
            if self.np.count_publishers(topico) > 0:
                e['sin_pub'] = None
            elif e['sin_pub'] is None:
                e['sin_pub'] = ahora
            elif ahora - e['sin_pub'] > OLVIDAR_S:
                self.np.destroy_subscription(e['sub'])
                self.nl.destroy_publisher(e['pub'])
                del self.activos[topico]
                print(f'[puente] quitado: {topico}', flush=True)

    @staticmethod
    def reenviar(datos, e):
        ahora = time.monotonic()
        if ahora - e['ultimo'] < 1.0 / HZ_MAX_ALUMNOS or len(datos) > BYTES_MAX_ALUMNOS:
            return
        e['ultimo'] = ahora
        e['pub'].publish(datos)


def main():
    destino, topicos = topicos_practicas()
    ctx_o, ctx_d = Context(), Context()
    rclpy.init(context=ctx_o, domain_id=ORIGEN)
    rclpy.init(context=ctx_d, domain_id=destino)
    lee = Node('puente_lectura', context=ctx_o)
    escribe = Node('puente_lectura', context=ctx_d)
    for topico, (tipo, persistente) in topicos.items():
        clase = get_message(tipo)
        if persistente:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        else:
            qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        sub_qos = qos if persistente else QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                      history=HistoryPolicy.KEEP_LAST)
        pub = escribe.create_publisher(clase, topico, qos)
        lee.create_subscription(clase, topico, lambda datos, pub=pub: pub.publish(datos), sub_qos, raw=True)
    SubidaAlumnos(escribe, lee)
    print(f'[puente] {len(topicos)} tópicos: dominio {ORIGEN} → {destino} (solo lectura); '
          f'/alumnos/* : {destino} → {ORIGEN}', flush=True)
    ej_o, ej_d = MultiThreadedExecutor(2, context=ctx_o), MultiThreadedExecutor(2, context=ctx_d)
    ej_o.add_node(lee)
    ej_d.add_node(escribe)
    threading.Thread(target=ej_d.spin, daemon=True).start()
    try:
        ej_o.spin()
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(0.1)


if __name__ == '__main__':
    main()
