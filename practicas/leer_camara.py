#!/usr/bin/env python3
"""Práctica 4 · Cámara: imagen cruda a arreglo numpy; guarda una imagen cada 2 s.

Tópicos: /auto/camara_frontal/image (sensor_msgs/msg/Image, rgb8)
         /auto/camara_frontal/camera_info (intrínsecos K)
Ejercicio: detecta las líneas del carril (Canny + Hough) sobre las imágenes guardadas.
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class LectorCamara(Node):
    def __init__(self):
        super().__init__('lector_camara')
        self.create_subscription(Image, '/auto/camara_frontal/image', self.cb, 2)
        self.create_subscription(CameraInfo, '/auto/camara_frontal/camera_info', self.cb_info, 2)
        self.k, self.t, self.n = None, 0.0, 0

    def cb_info(self, m):
        if self.k is None:
            self.k = np.array(m.k).reshape(3, 3)
            print('Matriz intrínseca K:\n', np.round(self.k, 1))

    def cb(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if time.time() - self.t > 2:
            self.t = time.time()
            self.n += 1
            nombre = f'cuadro_{self.n:03d}.png'
            cv2.imwrite(nombre, cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else img)
            print(f'{m.width}x{m.height} {m.encoding} · brillo medio {img.mean():.1f} · guardado {nombre}')


rclpy.init()
try:
    rclpy.spin(LectorCamara())
except KeyboardInterrupt:
    pass
