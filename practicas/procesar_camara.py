#!/usr/bin/env python3
"""Práctica 10 · Procesar la cámara y publicar tu propio tópico.

Lee la cámara frontal, le aplica un filtro con OpenCV y publica el resultado en
    /alumnos/<tu_nombre>/camara_procesada   (sensor_msgs/msg/Image, bgr8)
Ese tópico aparece en la página: pestaña Terminal → Datos crudos → elígelo y
verás tu video procesado en vivo (también lo ve el profesor).

Uso:
    python3 procesar_camara.py            # bordes (Canny)
    python3 procesar_camara.py gris
    python3 procesar_camara.py umbral
    python3 procesar_camara.py carril     # líneas del carril (Canny + Hough)
    python3 procesar_camara.py desenfoque

Ejercicio: copia este archivo, cambia la función procesar() y crea tu propio filtro.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

ENTRADA = '/auto/camara_frontal/image'
ALUMNO = os.environ.get('LAB_ALUMNO', 'anonimo')
SALIDA = f'/alumnos/{ALUMNO}/camara_procesada'
HZ_MAX = 5.0          # no proceses más rápido de lo necesario: el servidor es compartido


# ---------------------------------------------------------------- tu filtro
def procesar(img, modo):
    """Recibe una imagen BGR (numpy, alto × ancho × 3) y devuelve otra BGR."""
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if modo == 'gris':
        return cv2.cvtColor(gris, cv2.COLOR_GRAY2BGR)

    if modo == 'umbral':
        _, bin_ = cv2.threshold(gris, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return cv2.cvtColor(bin_, cv2.COLOR_GRAY2BGR)

    if modo == 'desenfoque':
        return cv2.GaussianBlur(img, (15, 15), 0)

    if modo == 'carril':
        alto, ancho = gris.shape
        bordes = cv2.Canny(cv2.GaussianBlur(gris, (5, 5), 0), 60, 160)
        # Región de interés: la mitad inferior (donde está el camino)
        mascara = np.zeros_like(bordes)
        poligono = np.array([[(0, alto), (ancho, alto), (int(ancho * 0.6), int(alto * 0.55)),
                              (int(ancho * 0.4), int(alto * 0.55))]], np.int32)
        cv2.fillPoly(mascara, poligono, 255)
        lineas = cv2.HoughLinesP(bordes & mascara, 1, np.pi / 180, 30, minLineLength=25, maxLineGap=20)
        salida = img.copy()
        cv2.polylines(salida, poligono, True, (255, 160, 0), 1)
        for x1, y1, x2, y2 in (lineas.reshape(-1, 4) if lineas is not None else []):
            cv2.line(salida, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
        return salida

    # bordes (por defecto)
    bordes = cv2.Canny(cv2.GaussianBlur(gris, (5, 5), 0), 60, 160)
    return cv2.cvtColor(bordes, cv2.COLOR_GRAY2BGR)


# ---------------------------------------------------------------- nodo ROS 2
class ProcesarCamara(Node):
    def __init__(self, modo):
        super().__init__('procesar_camara')
        self.modo = modo
        self.pub = self.create_publisher(Image, SALIDA, 2)
        self.create_subscription(Image, ENTRADA, self.cb, 2)
        self.ultimo, self.n = 0.0, 0
        print(f'Leyendo   {ENTRADA}')
        print(f'Publicando {SALIDA}  (filtro: {modo})')
        print('En la página: Terminal → Datos crudos → elige tu tópico.  Ctrl+C para salir.\n')

    def cb(self, m):
        ahora = time.monotonic()
        if ahora - self.ultimo < 1.0 / HZ_MAX:
            return
        self.ultimo = ahora

        # 1) Mensaje ROS → arreglo numpy (los bytes vienen "crudos" en m.data)
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # 2) Procesar
        t0 = time.perf_counter()
        res = np.ascontiguousarray(procesar(img, self.modo), dtype=np.uint8)
        ms = (time.perf_counter() - t0) * 1000
        cv2.putText(res, f'{ALUMNO} · {self.modo} · {ms:.1f} ms', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 3) Arreglo numpy → mensaje ROS y publicar
        salida = Image()
        salida.header = m.header                 # misma marca de tiempo que la imagen original
        salida.height, salida.width = res.shape[:2]
        salida.encoding = 'bgr8'
        salida.step = res.shape[1] * 3
        salida.data = res.tobytes()
        self.pub.publish(salida)

        self.n += 1
        if self.n % 25 == 1:
            print(f'{self.n:5d} imágenes publicadas · {m.width}x{m.height} · {ms:.1f} ms por imagen')


def main():
    modo = sys.argv[1] if len(sys.argv) > 1 else 'bordes'
    rclpy.init()
    nodo = ProcesarCamara(modo)
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
