#!/usr/bin/env python3
"""Núcleo del Laboratorio Virtual ADS sobre Gazebo Harmonic + ROS 2 Jazzy.

Mantiene la misma interfaz ROS 2 que la versión con CARLA (/auto/...), así que
la web, la ECU y el código de los alumnos no cambian.

- Dinámica del vehículo (modelo de bicicleta) → velocidad a Gazebo (VelocityControl).
- Autopiloto y tráfico: seguimiento de ruta (pure pursuit) con cesión de paso.
- Sensores de Gazebo (cámaras, LiDAR, IMU, GNSS) se reenvían a /auto/... aplicando fallas.
- Sensores emulados con la geometría de la ciudad: ultrasonido, radar, encoders,
  invasión de carril; colisiones desde el sensor de contacto.
- ECU virtual (bus CAN, OBD-II, DTC) idéntica a la versión anterior.
"""
import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, NavSatFix, NavSatStatus, PointCloud2, Range
from std_msgs.msg import Bool, Empty, Float32MultiArray, Header, Int32MultiArray, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

import fallas
from conversiones import a_bytes_ros, a_stamp, nube_puntos, rpy_a_quat
from ecu import ECU
from geometria import Geometria, esquinas

CFG = yaml.safe_load(Path(os.environ.get('LAB_CONFIG', '/lab/config/lab.yaml')).read_text(encoding='utf-8'))
DATA = Path(os.environ.get('LAB_DATA', '/data'))
AUTO = CFG['equipos'][0]['nombre']
MUNDO = CFG.get('simulador', {}).get('mundo', 'ciudad')
RUEDAS = ['del_izq', 'del_der', 'tras_izq', 'tras_der']
WATCHDOG_S = 1.0
TIPOS_SIN_TF = ('sensor.other.collision', 'sensor.other.lane_invasion')
TAM_AUTO = (2.2, 0.9)          # medio largo, medio ancho de los autos de tráfico


def log(m):
    print(f'[nucleo] {m}', flush=True)


def expandir_sensores(specs):
    salida = []
    for s in specs:
        if s.get('arreglo'):
            for clave, pose in s['arreglo'].items():
                sub = {k: v for k, v in s.items() if k != 'arreglo'}
                sub.update(nombre=f"{s['nombre']}_{clave}", pos=pose.get('pos', [0, 0, 0]),
                           rot=pose.get('rot', [0, 0, 0]))
                salida.append(sub)
        else:
            salida.append(s)
    return salida


def yaw_de_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ============================================================ seguimiento de ruta
class Seguidor:
    """Pure pursuit sobre una ruta cerrada, con cesión de paso sencilla."""

    def __init__(self, ruta, idx=0, vel=7.0, distancia_ejes=2.7):
        self.ruta, self.idx, self.vel, self.L = ruta, idx, vel, distancia_ejes
        self.detenido = 0.0          # s detenido cediendo el paso (para romper bloqueos)

    def reubicar(self, x, y):
        self.idx = int(np.argmin(np.hypot(self.ruta[:, 0] - x, self.ruta[:, 1] - y)))

    def paso(self, x, y, yaw, v, otros, mi_id, esperar_cruce=None):
        n = len(self.ruta)
        ventana = [(self.idx + k) % n for k in range(20)]
        d = np.hypot(self.ruta[ventana, 0] - x, self.ruta[ventana, 1] - y)
        self.idx = ventana[int(np.argmin(d))]
        ld = max(4.0, 0.6 * abs(v) + 2.0)
        j = (self.idx + int(ld / 2.0)) % n
        tx, ty = self.ruta[j]
        alfa = ang(math.atan2(ty - y, tx - x) - yaw)
        delta = math.atan2(2 * self.L * math.sin(alfa), ld)
        # curva próxima: bajar la velocidad antes de girar
        k2 = (self.idx + 6) % n
        giro_prox = abs(ang(math.atan2(self.ruta[k2, 1] - self.ruta[self.idx, 1],
                                       self.ruta[k2, 0] - self.ruta[self.idx, 0]) - yaw))
        objetivo = self.vel * (0.5 if max(abs(alfa), giro_prox) > 0.35 else 1.0)
        c, s = math.cos(yaw), math.sin(yaw)
        bloqueado = self.detenido > 3.0           # tras 3 s cediendo, avanza con cuidado
        for (ox, oy, oyaw, oid) in otros:
            lx, ly = c * (ox - x) + s * (oy - y), -s * (ox - x) + c * (oy - y)
            if lx <= 0 or lx > 16:
                continue
            mismo_sentido = math.cos(ang(oyaw - yaw)) > 0.3
            if (abs(ly) < 2.0 and mismo_sentido) or (abs(ly) < 1.0 and lx < 8):
                # alguien adelante en mi carril (o de frente en mi trayectoria)
                if not (bloqueado and abs(math.cos(ang(oyaw - yaw))) < 0.7):
                    objetivo = min(objetivo, 0.0 if lx < 6.5 else self.vel * (lx - 6.5) / 9)
            elif oid == 'auto' and abs(ly) < 9 and lx < 13 and abs(math.cos(ang(oyaw - yaw))) < 0.5 \
                    and not bloqueado:
                objetivo = min(objetivo, 0.0)                   # el auto de los alumnos tiene preferencia
        if esperar_cruce is not None:                           # cruce ocupado: alto antes de entrar
            objetivo = min(objetivo, 0.0 if esperar_cruce < 10.5 else self.vel * (esperar_cruce - 10.5) / 4)
        return objetivo, delta

    def actualizar_espera(self, v, dt):
        self.detenido = self.detenido + dt if abs(v) < 0.2 else 0.0
        if self.detenido > 6.0:
            self.detenido = 0.0


# ============================================================ cruces
class Cruces:
    """Reserva de intersecciones: solo un vehículo automático cruza a la vez."""

    def __init__(self, coords):
        self.centros = [(cx, cy) for cx in coords for cy in coords]
        self.duenos = {}             # índice de cruce → (id, tiempo)

    def consultar(self, vid, x, y, yaw, ahora):
        """Devuelve la distancia al cruce si hay que esperar; None si puede seguir."""
        for k, (cx, cy) in enumerate(self.centros):
            d = math.hypot(cx - x, cy - y)
            acercando = math.cos(yaw) * (cx - x) + math.sin(yaw) * (cy - y) > 0
            dueno = self.duenos.get(k)
            if dueno and dueno[0] == vid and (d > 11 and not acercando or ahora - dueno[1] > 20):
                del self.duenos[k]                          # ya salió del cruce
                continue
            if d < 15 and acercando:
                if dueno is None or ahora - dueno[1] > 20:
                    self.duenos[k] = (vid, ahora)
                elif dueno[0] != vid:
                    return d
        return None


# ============================================================ nodo
class Nucleo(Node):
    def __init__(self):
        super().__init__('nucleo_lab')
        self.grupo_sens = ReentrantCallbackGroup()
        self.grupo_ciclo = MutuallyExclusiveCallbackGroup()
        rapido = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.qos_rapido = rapido

        self.geo = Geometria(DATA / 'ciudad.json')
        self.sensores_cfg = expandir_sensores(CFG['sensores'])
        self.specs = {s['nombre']: s for s in self.sensores_cfg}
        veh = CFG.get('vehiculo', {})
        self.L = float(veh.get('distancia_ejes', 2.7))
        self.dmax = math.radians(float(veh.get('max_giro', 35)))
        self.amax = float(veh.get('acel_max', 3.5))
        self.bmax = float(veh.get('frenado_max', 8.0))
        self.vmax = float(veh.get('vel_max', 22.0))
        self.radio = float(veh.get('radio_rueda', 0.33))
        self.via = 1.6
        enc = CFG.get('encoders', {}) or {}
        self.cpr = int(enc.get('ticks_por_vuelta', 1024))
        self.ruido_enc = float(enc.get('ruido_velocidad', 0.02))
        self.tf_gt = bool(CFG.get('simulador', {}).get('tf_ground_truth', True))
        self.rng = np.random.default_rng(7)

        # estado
        self.t_sim = 0.0
        self.t_prev = None
        self.poses = {}                 # nombre → (x, y, z, yaw)
        self.pose_prev = None
        self.pose_auto_t = None
        self.vx_med = 0.0
        self.wz_med = 0.0
        self.v = 0.0                    # velocidad del modelo dinámico
        self.delta = 0.0
        self.modo, self.cmd, self.cmd_wall = 'libre', None, 0.0
        self.integral = 0.0
        self.control = SimpleNamespace(throttle=0.0, brake=1.0, steer=0.0, reverse=False, gear=1, hand_brake=False)
        self.ang_ruedas = np.zeros(4)
        self.us = {}
        self.esquinas_prev = None
        self.ultimo_evento = {}
        self.contactos = {}
        self.conteo = {}
        self.ultimo_frame = None        # para flujo óptico
        self.gris_prev = None
        self.camara_extra = ''
        self.subs_extra = []
        self.ciclos = 0
        self.t_rep, self.sim_rep, self.ciclos_rep = time.monotonic(), 0.0, 0
        self.t_us = self.t_radar = -1e9

        d = self.geo.datos
        self.spawn = d['ego']
        self.seguidor_ego = Seguidor(self.geo.rutas[d['ego']['ruta']], 0, 7.0, self.L)
        vel_npc = float(CFG.get('trafico', {}).get('velocidad', 7.0))
        self.cruces = Cruces(d['coords'])
        self.npcs = []
        for k, n in enumerate(d['npcs']):
            self.npcs.append({'nombre': n['nombre'], 'id': k, 'v': 0.0, 'pose_prev': None, 'vel': np.zeros(2),
                              'seg': Seguidor(self.geo.rutas[n['ruta']], n['inicio'], vel_npc, 2.7),
                              'pub': self.create_publisher(Twist, f'/gz/{n["nombre"]}/cmd_vel', 10)})

        # ------------------------------------------------ publicadores
        p = self.create_publisher
        self.pub_cmd = p(Twist, f'/gz/{AUTO}/cmd_vel', 10)
        self.pub_odom = p(Odometry, f'/{AUTO}/ground_truth/odom', 10)
        self.pub_estado = p(String, f'/{AUTO}/vehiculo/estado', 10)
        self.pub_eventos = p(String, f'/{AUTO}/eventos', 10)
        self.pub_enc = p(JointState, f'/{AUTO}/encoders/joint_states', 10)
        self.pub_ticks = p(Int32MultiArray, f'/{AUTO}/encoders/ticks', 10)
        self.pub_lab = p(String, '/lab/estado', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.pubs = {}
        for s in self.sensores_cfg:
            n, t = s['nombre'], s['tipo']
            base = f'/{AUTO}/{n}'
            if t.startswith('sensor.camera'):
                self.pubs[n] = {'img': p(Image, base + '/image', 5), 'info': p(CameraInfo, base + '/camera_info', 5)}
                if t == 'sensor.camera.semantic_segmentation':
                    self.pubs[n]['labels'] = p(Image, base + '/labels', 5)
                if t == 'sensor.camera.optical_flow':
                    self.pubs[n]['flow'] = p(Image, base + '/flow', 5)
            elif t in ('sensor.lidar.ray_cast', 'sensor.other.radar'):
                self.pubs[n] = p(PointCloud2, base + '/points', 5)
            elif t == 'sensor.other.imu':
                self.pubs[n] = p(Imu, base + '/data', 10)
            elif t == 'sensor.other.gnss':
                self.pubs[n] = p(NavSatFix, base + '/fix', 10)
            elif t == 'sensor.other.obstacle':
                self.pubs[n] = p(Range, base + '/range', 10)

        # ------------------------------------------------ suscripciones
        s = self.create_subscription
        g = self.grupo_sens
        s(Clock, '/clock', self._cb_clock, 10, callback_group=g)
        for nombre in [AUTO] + [n['nombre'] for n in self.npcs]:
            s(TransformStamped, f'/gz/{nombre}/pose', lambda m, nombre=nombre: self._cb_pose(m, nombre), 10,
              callback_group=g)
        s(Contacts, f'/gz/{AUTO}/contacto', self._cb_contacto, 10, callback_group=g)
        for spec in self.sensores_cfg:
            n, t = spec['nombre'], spec['tipo']
            if t == 'sensor.camera.rgb':
                s(Image, f'/gz/{AUTO}/{n}/image', lambda m, n=n: self._cb_rgb(m, n), rapido, callback_group=g)
            elif t == 'sensor.lidar.ray_cast':
                s(PointCloud2, f'/gz/{AUTO}/{n}/points', lambda m, n=n: self._cb_lidar(m, n), rapido, callback_group=g)
            elif t == 'sensor.other.imu':
                s(Imu, f'/gz/{AUTO}/{n}', lambda m, n=n: self._cb_imu(m, n), 20, callback_group=g)
            elif t == 'sensor.other.gnss':
                s(NavSatFix, f'/gz/{AUTO}/{n}', lambda m, n=n: self._cb_gnss(m, n), 10, callback_group=g)

        s(AckermannDriveStamped, f'/{AUTO}/cmd/ackermann',
          lambda m: self._comando('ackermann', (m.drive.speed, m.drive.steering_angle)), 10, callback_group=g)
        s(Float32MultiArray, f'/{AUTO}/cmd/directo',
          lambda m: self._comando('directo', tuple((list(m.data) + [0.0] * 4)[:4])), 10, callback_group=g)
        s(Bool, f'/{AUTO}/cmd/autopilot', self._cb_autopiloto, 10, callback_group=g)
        s(Empty, f'/{AUTO}/reset', lambda m: self.reiniciar(), 10, callback_group=g)
        s(String, '/lab/clima', lambda m: log(f'Clima "{m.data}": en Gazebo la iluminación es fija'), 10)
        s(String, '/lab/camara_extra', lambda m: self.cambiar_extra(m.data), 10, callback_group=g)
        s(String, '/lab/fallas', self._cb_falla, 10, callback_group=g)
        s(Empty, '/lab/borrar_dtc', lambda m: fallas.borrar_dtc(), 10, callback_group=g)

        # ECU con parámetros de tren motriz equivalentes
        fis = SimpleNamespace(forward_gears=[SimpleNamespace(ratio=r) for r in (3.6, 2.1, 1.4, 1.0, 0.8)],
                              final_ratio=3.9, max_rpm=6500.0,
                              wheels=[SimpleNamespace(max_steer_angle=math.degrees(self.dmax))])
        self.ecu = ECU(self, AUTO, SimpleNamespace(get_physics_control=lambda: fis),
                       (CFG.get('ecu') or {}).get('dbc', '/lab/config/lab_ads.dbc'))

        self.publicar_tf_estaticos()
        paso = float(CFG.get('simulador', {}).get('paso_control', 0.05))
        self.create_timer(paso, self.ciclo, callback_group=self.grupo_ciclo)
        log(f'Listo. Vehículo "{AUTO}", {len(self.npcs)} autos de tráfico, ECU virtual activa.')

    # ============================================================ entradas
    def _cb_clock(self, m):
        self.t_sim = m.clock.sec + m.clock.nanosec * 1e-9

    def _cb_pose(self, m, nombre):
        tr = m.transform
        self.poses[nombre] = (tr.translation.x, tr.translation.y, tr.translation.z, yaw_de_quat(tr.rotation))
        if nombre == AUTO:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.pose_auto_t = (t, self.poses[nombre])

    def _comando(self, modo, cmd):
        if self.modo == 'autopilot':
            log('Autopiloto desactivado por comando manual')
        self.modo, self.cmd, self.cmd_wall = modo, cmd, time.monotonic()

    def _cb_autopiloto(self, m):
        if m.data:
            x, y, _, _ = self.poses.get(AUTO, (self.spawn['x'], self.spawn['y'], 0, 0))
            self.seguidor_ego.reubicar(x, y)
            self.modo = 'autopilot'
        else:
            self.modo, self.cmd = 'libre', None
        log(f'Autopiloto {"ON" if m.data else "OFF"}')

    def _cb_falla(self, m):
        try:
            d = json.loads(m.data)
            if fallas.poner(d['codigo'], bool(d['activo'])):
                log(f'Falla {d["codigo"]}: {"ACTIVADA" if d["activo"] else "desactivada"}')
        except (ValueError, KeyError, TypeError):
            log(f'Mensaje de falla inválido: {m.data}')

    def _evento(self, datos, clave, espera=1.0):
        ahora = time.monotonic()
        if ahora - self.ultimo_evento.get(clave, 0) < espera:
            return
        self.ultimo_evento[clave] = ahora
        datos.update(t=round(self.t_sim, 3), equipo=AUTO)
        self.pub_eventos.publish(String(data=json.dumps(datos)))

    def _cb_contacto(self, m):
        for c in m.contacts:
            otro = c.collision2.name if AUTO in c.collision1.name else c.collision1.name
            if any(k in otro for k in ('suelo', 'calles', 'lote', 'manzanas')):
                continue
            nombre = otro.split('::')[0]
            ahora = time.monotonic()
            nuevo = ahora - self.contactos.get(nombre, 0) > 2.0     # contacto continuo = un solo evento
            self.contactos[nombre] = ahora
            impacto = abs(self.vx_med)
            if impacto > 0.3:
                self.v = 0.0
            if nuevo:
                self._evento({'tipo': 'colision', 'con': nombre, 'impulso': round(impacto * 1500, 1)},
                             'col:' + nombre, 0.0)
            break

    def reiniciar(self):
        sp = self.spawn
        qz, qw = math.sin(sp['yaw'] / 2), math.cos(sp['yaw'] / 2)
        req = (f'name: "{AUTO}" position: {{x: {sp["x"]} y: {sp["y"]} z: 0.02}} '
               f'orientation: {{x: 0 y: 0 z: {qz} w: {qw}}}')
        self.v, self.delta, self.integral = 0.0, 0.0, 0.0
        self.modo, self.cmd = 'libre', None

        def llamar():
            try:
                subprocess.run(['gz', 'service', '-s', f'/world/{MUNDO}/set_pose', '--reqtype', 'gz.msgs.Pose',
                                '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                               check=False, timeout=8, capture_output=True)
                log('Vehículo reiniciado en su punto de salida')
            except (OSError, subprocess.SubprocessError) as e:
                log(f'No se pudo reiniciar: {e}')
        threading.Thread(target=llamar, daemon=True).start()

    # ============================================================ sensores de Gazebo → /auto
    def _contar(self, n):
        self.conteo[n] = self.conteo.get(n, 0) + 1

    def _camera_info(self, n, header, w, h):
        a = self.specs[n].get('attrs', {})
        fov = math.radians(float(a.get('fov', 90)))
        f = w / (2 * math.tan(fov / 2))
        info = CameraInfo(header=header, width=w, height=h, distortion_model='plumb_bob')
        info.d = [0.0] * 5
        info.k = [f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, w / 2, 0.0, 0.0, f, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _reenviar_imagen(self, m, n, clave='img'):
        m.header.frame_id = f'{AUTO}/{n}_optical'
        if fallas.activa('B1A01'):
            m.data = a_bytes_ros(np.zeros(len(m.data), np.uint8))
        self.pubs[n][clave].publish(m)
        if clave == 'img':
            self.pubs[n]['info'].publish(self._camera_info(n, m.header, m.width, m.height))
            self._contar(n)

    def _cb_rgb(self, m, n):
        self.ultimo_frame = m
        self._reenviar_imagen(m, n)
        if self.camara_extra and self.specs.get(self.camara_extra, {}).get('tipo') == 'sensor.camera.optical_flow':
            self._flujo(m)

    def _flujo(self, m):
        n = self.camara_extra
        a = self.specs[n].get('attrs', {})
        w, h = int(a.get('image_size_x', 320)), int(a.get('image_size_y', 180))
        try:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        except ValueError:
            return
        gris = cv2.cvtColor(cv2.resize(img, (w, h)), cv2.COLOR_RGB2GRAY)
        if self.gris_prev is not None:
            flujo = cv2.calcOpticalFlowFarneback(self.gris_prev, gris, None, 0.5, 2, 13, 2, 5, 1.1, 0)
            mag, angulo = cv2.cartToPolar(flujo[..., 0], flujo[..., 1])
            hsv = np.zeros((h, w, 3), np.uint8)
            hsv[..., 0] = (angulo * 90 / np.pi).astype(np.uint8)
            hsv[..., 1] = 255
            hsv[..., 2] = np.clip(mag * 25, 0, 255).astype(np.uint8)
            color = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            hdr = Header(stamp=m.header.stamp, frame_id=f'{AUTO}/{n}_optical')
            self.pubs[n]['img'].publish(self._imagen(hdr, color, 'bgr8'))
            self.pubs[n]['flow'].publish(self._imagen(hdr, flujo.astype(np.float32), '32FC2'))
            self.pubs[n]['info'].publish(self._camera_info(n, hdr, w, h))
            self._contar(n)
        self.gris_prev = gris

    @staticmethod
    def _imagen(header, arr, enc):
        arr = np.ascontiguousarray(arr)
        return Image(header=header, height=arr.shape[0], width=arr.shape[1], encoding=enc,
                     is_bigendian=0, step=arr.strides[0], data=a_bytes_ros(arr))

    def _cb_lidar(self, m, n):
        if m.width * m.height == 0:
            return
        campos = {f.name: f.offset for f in m.fields}
        paso = m.point_step
        crudo = np.frombuffer(m.data, np.uint8).reshape(-1, paso)

        def col(nombre):
            o = campos.get(nombre)
            if o is None:
                return np.zeros(len(crudo), np.float32)
            return crudo[:, o:o + 4].copy().view(np.float32).ravel()
        pts = np.stack([col('x'), col('y'), col('z'), col('intensity')], axis=1)
        pts = pts[np.isfinite(pts[:, :3]).all(axis=1)]
        if fallas.activa('B1A02'):
            pts = pts[self.rng.random(len(pts)) < 0.3]
            pts[:, :3] += self.rng.normal(0, 0.15, (len(pts), 3)).astype(np.float32)
        hdr = Header(stamp=m.header.stamp, frame_id=f'{AUTO}/{n}')
        self.pubs[n].publish(nube_puntos(hdr, pts, ['x', 'y', 'z', 'intensity']))
        self._contar(n)

    def _cb_imu(self, m, n):
        m.header.frame_id = f'{AUTO}/{n}'
        if fallas.activa('B1A04'):
            m.angular_velocity.z += 0.05
            m.linear_acceleration.x += 0.5
        a = self.specs[n].get('attrs', {})
        sa, sg = float(a.get('noise_accel', 0.05)) ** 2, float(a.get('noise_gyro', 0.005)) ** 2
        m.linear_acceleration_covariance = [sa, 0.0, 0.0, 0.0, sa, 0.0, 0.0, 0.0, sa]
        m.angular_velocity_covariance = [sg, 0.0, 0.0, 0.0, sg, 0.0, 0.0, 0.0, sg]
        self.pubs[n].publish(m)
        self._contar(n)

    def _cb_gnss(self, m, n):
        m.header.frame_id = f'{AUTO}/{n}'
        m.status.service = NavSatStatus.SERVICE_GPS
        if fallas.activa('B1A03'):
            m.status.status = NavSatStatus.STATUS_NO_FIX
            m.latitude = m.longitude = m.altitude = float('nan')
        else:
            m.status.status = NavSatStatus.STATUS_FIX
            a = self.specs[n].get('attrs', {})
            h, v = float(a.get('noise_horizontal', 1.5)) ** 2, float(a.get('noise_vertical', 3.0)) ** 2
            m.position_covariance = [h, 0.0, 0.0, 0.0, h, 0.0, 0.0, 0.0, v]
            m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pubs[n].publish(m)
        self._contar(n)

    # ------------------------------------------------ cámaras bajo demanda
    def cambiar_extra(self, nombre):
        for sub in self.subs_extra:
            self.destroy_subscription(sub)
        self.subs_extra, self.gris_prev = [], None
        spec = self.specs.get(nombre)
        if not spec or not spec.get('bajo_demanda'):
            self.camara_extra = ''
            log('Cámara adicional: ninguna')
            return
        t, g, q = spec['tipo'], self.grupo_sens, self.qos_rapido
        if t == 'sensor.camera.depth':
            self.subs_extra.append(self.create_subscription(
                Image, f'/gz/{AUTO}/{nombre}/image', lambda m: self._reenviar_imagen(m, nombre), q, callback_group=g))
        elif t == 'sensor.camera.semantic_segmentation':
            self.subs_extra.append(self.create_subscription(
                Image, f'/gz/{AUTO}/{nombre}/colored_map', lambda m: self._reenviar_imagen(m, nombre), q,
                callback_group=g))
            self.subs_extra.append(self.create_subscription(
                Image, f'/gz/{AUTO}/{nombre}/labels_map', lambda m: self._reenviar_imagen(m, nombre, 'labels'), q,
                callback_group=g))
        self.camara_extra = nombre
        log(f'Cámara adicional: {nombre}')

    # ============================================================ ciclo principal
    def ciclo(self):
        t = self.t_sim
        if self.t_prev is None or AUTO not in self.poses:
            self.t_prev = t
            return
        dt = t - self.t_prev
        if dt <= 1e-4:
            return
        dt = min(dt, 0.25)
        self.t_prev = t
        self.ciclos += 1

        x, y, z, yaw = self.poses[AUTO]
        # velocidad medida con las marcas de tiempo de las poses (robusto si la simulación va lenta)
        if self.pose_auto_t is not None:
            tp, (x, y, z, yaw) = self.pose_auto_t
            if self.pose_prev is not None and tp > self.pose_prev[0] + 1e-4:
                t0, (px, py, _, pyaw) = self.pose_prev
                dtp = tp - t0
                vx = (math.cos(yaw) * (x - px) + math.sin(yaw) * (y - py)) / dtp
                self.vx_med = 0.6 * self.vx_med + 0.4 * vx
                self.wz_med = 0.6 * self.wz_med + 0.4 * ang(yaw - pyaw) / dtp
            if self.pose_prev is None or tp > self.pose_prev[0] + 1e-4:
                self.pose_prev = (tp, (x, y, z, yaw))

        otros = self.trafico(dt, x, y, yaw)
        self.dinamica(dt, x, y, yaw, otros)
        stamp = a_stamp(t)
        self.publicar_estado(stamp, x, y, z, yaw)
        omegas = self.encoders(stamp, dt)
        self.ecu.actualizar(t, dt, self.vx_med, self.wz_med, self.control, omegas, self.radio)
        self.ultrasonido_y_radar(t, stamp, x, y, yaw)
        self.invasion_carril(x, y, yaw)
        self.reportar()

    # ------------------------------------------------ tráfico
    def trafico(self, dt, ex, ey, eyaw):
        """Mueve los autos de tráfico; devuelve sus posiciones para cesión de paso y sensores."""
        posiciones = []
        for n in self.npcs:
            p = self.poses.get(n['nombre'])
            if p:
                posiciones.append((p[0], p[1], p[3], n['id']))
        for n in self.npcs:
            p = self.poses.get(n['nombre'])
            if not p:
                continue
            x, y, _, yaw = p
            if n['pose_prev'] is not None:
                n['vel'] = (np.array([x, y]) - n['pose_prev']) / dt
            n['pose_prev'] = np.array([x, y])
            otros = [o for o in posiciones if o[3] != n['id']] + [(ex, ey, eyaw, 'auto')]
            n['seg'].actualizar_espera(n['v'], dt)
            espera = self.cruces.consultar(n['id'], x, y, yaw, self.t_sim)
            objetivo, delta = n['seg'].paso(x, y, yaw, n['v'], otros, n['id'], espera)
            dv = np.clip(objetivo - n['v'], -6.0 * dt, 2.5 * dt)
            n['v'] = max(0.0, n['v'] + dv)
            tw = Twist()
            tw.linear.x = n['v']
            tw.angular.z = n['v'] * math.tan(delta) / 2.7
            n['pub'].publish(tw)
        return posiciones

    # ------------------------------------------------ dinámica del vehículo
    def dinamica(self, dt, x, y, yaw, otros):
        edad = time.monotonic() - self.cmd_wall
        acel = freno = giro = 0.0
        reversa = False
        if self.modo == 'autopilot':
            self.seguidor_ego.actualizar_espera(self.v, dt)
            espera = self.cruces.consultar('ego', x, y, yaw, self.t_sim)
            objetivo, delta = self.seguidor_ego.paso(x, y, yaw, self.v, otros, 'ego', espera)
            acel, freno = self.pi_velocidad(objetivo, dt)
            giro = delta / self.dmax
        elif self.cmd is None or edad > WATCHDOG_S:
            freno = 1.0
            self.integral = 0.0
        elif self.modo == 'ackermann':
            v_ref, delta = self.cmd
            reversa = v_ref < -0.01
            acel, freno = self.pi_velocidad(abs(v_ref), dt, reversa)
            giro = delta / self.dmax
        elif self.modo == 'directo':
            a, g, f, r = self.cmd
            acel, giro, freno, reversa = float(np.clip(a, 0, 1)), float(np.clip(g, -1, 1)), float(np.clip(f, 0, 1)), r > 0.5
        giro = float(np.clip(giro, -1, 1))

        # dirección con velocidad limitada del actuador
        objetivo_d = giro * self.dmax
        self.delta += float(np.clip(objetivo_d - self.delta, -1.2 * dt, 1.2 * dt))
        sentido = -1.0 if reversa else 1.0
        vlim = 5.0 if reversa else self.vmax
        a = acel * self.amax * max(0.0, 1 - abs(self.v) / vlim) * sentido
        if abs(self.v) > 0.05:
            a -= math.copysign(freno * self.bmax + 0.25 + 0.015 * self.v * self.v, self.v)
        v_nueva = self.v + a * dt
        if self.v != 0 and v_nueva * self.v < 0 and acel * sentido * self.v <= 0:
            v_nueva = 0.0                                      # frenar no invierte el sentido
        if abs(v_nueva) < 0.05 and acel == 0:
            v_nueva = 0.0
        self.v = float(np.clip(v_nueva, -5.0, self.vmax))
        tw = Twist()
        tw.linear.x = self.v
        tw.angular.z = self.v * math.tan(self.delta) / self.L
        self.pub_cmd.publish(tw)

        marcha = -1 if reversa else (1 if abs(self.v) < 4 else 2 if abs(self.v) < 8.5 else 3 if abs(self.v) < 14
                                     else 4 if abs(self.v) < 19 else 5)
        self.control = SimpleNamespace(throttle=acel, brake=freno, steer=-self.delta / self.dmax,
                                       reverse=reversa, gear=marcha, hand_brake=False)

    def pi_velocidad(self, objetivo, dt, reversa=False):
        medida = -self.v if reversa else self.v
        err = objetivo - medida
        self.integral = float(np.clip(self.integral + err * dt, -5, 5))
        u = 0.35 * err + 0.1 * self.integral + 0.03 * objetivo
        if objetivo < 0.05 and abs(medida) < 0.3:
            self.integral = 0.0
            return 0.0, 1.0
        return (float(min(u, 1.0)), 0.0) if u >= 0 else (0.0, float(min(-u, 1.0)))

    # ------------------------------------------------ salidas
    def publicar_estado(self, stamp, x, y, z, yaw):
        q = rpy_a_quat(0.0, 0.0, yaw)
        od = Odometry()
        od.header = Header(stamp=stamp, frame_id='map')
        od.child_frame_id = f'{AUTO}/base_link'
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = x, y, z
        od.pose.pose.orientation = q
        od.twist.twist.linear.x = self.vx_med
        od.twist.twist.angular.z = self.wz_med
        self.pub_odom.publish(od)
        if self.tf_gt:
            self.tf.sendTransform(self._tf('map', f'{AUTO}/base_link', (x, y, z), q, stamp))
        if self.ciclos % 2 == 0:
            c = self.control
            self.pub_estado.publish(String(data=json.dumps({
                't': round(self.t_sim, 2), 'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 3),
                'v': round(self.vx_med, 2), 'acel': round(c.throttle, 2), 'giro': round(-c.steer, 2),
                'freno': round(c.brake, 2), 'reversa': c.reverse, 'modo': self.modo, 'us': self.us})))

    def encoders(self, stamp, dt):
        vx, wz = self.vx_med, self.wz_med
        mitad = self.via / 2
        vi, vd = vx - wz * mitad, vx + wz * mitad
        omega = np.array([vi, vd, vi, vd]) / self.radio
        medida = omega * (1 + self.rng.normal(0, self.ruido_enc, 4))
        avance = omega * dt
        if fallas.activa('C0035'):
            medida[0], avance[0] = 0.0, 0.0
        self.ang_ruedas += avance
        ticks = np.floor(self.ang_ruedas / (2 * math.pi) * self.cpr).astype(int)
        js = JointState()
        js.header = Header(stamp=stamp, frame_id=f'{AUTO}/base_link')
        js.name = [f'{AUTO}/rueda_{r}' for r in RUEDAS]
        js.position = (ticks * 2 * math.pi / self.cpr).tolist()
        js.velocity = medida.tolist()
        self.pub_enc.publish(js)
        self.pub_ticks.publish(Int32MultiArray(data=ticks.tolist()))
        return omega.tolist()

    def _cajas_npc(self):
        filas = []
        for n in self.npcs:
            p = self.poses.get(n['nombre'])
            if p:
                filas.append([p[0], p[1], p[3], TAM_AUTO[0], TAM_AUTO[1]])
        return np.array(filas, float).reshape(-1, 5)

    def ultrasonido_y_radar(self, t, stamp, x, y, yaw):
        cajas = self._cajas_npc()
        c, s = math.cos(yaw), math.sin(yaw)
        # ---- ultrasonido (10 Hz)
        if t - self.t_us >= 0.095:
            self.t_us = t
            for spec in self.sensores_cfg:
                if spec['tipo'] != 'sensor.other.obstacle':
                    continue
                n = spec['nombre']
                px, py = spec['pos'][0], spec['pos'][1]
                ox, oy = x + c * px - s * py, y + s * px + c * py
                a0 = yaw + math.radians(spec['rot'][2])
                angs = a0 + np.linspace(-0.21, 0.21, 5)
                dist, _, _, _ = self.geo.rayos(np.tile([ox, oy], (5, 1)), np.stack([np.cos(angs), np.sin(angs)], 1),
                                               5.0, cajas)
                r = float(np.min(dist))
                msg = Range(header=Header(stamp=stamp, frame_id=f'{AUTO}/{n}'), radiation_type=Range.ULTRASOUND,
                            field_of_view=0.5, min_range=0.2, max_range=5.0)
                if fallas.activa('B1A05') and '_del' in n:
                    msg.range = float('nan')
                    self.us[n] = -2.0
                elif math.isfinite(r):
                    msg.range = float(max(0.2, r + self.rng.normal(0, 0.02)))
                    self.us[n] = round(msg.range, 2)
                else:
                    msg.range = float('inf')
                    self.us[n] = -1.0
                self.pubs[n].publish(msg)
                self._contar(n)
        # ---- radar (10 Hz)
        if t - self.t_radar >= 0.095:
            self.t_radar = t
            for spec in self.sensores_cfg:
                if spec['tipo'] != 'sensor.other.radar':
                    continue
                n, a = spec['nombre'], spec.get('attrs', {})
                fov, alcance = math.radians(float(a.get('horizontal_fov', 30))) / 2, float(a.get('range', 100))
                k = int(a.get('rayos', 41))
                px, py = spec['pos'][0], spec['pos'][1]
                ox, oy = x + c * px - s * py, y + s * px + c * py
                rel = np.linspace(-fov, fov, k)
                angs = yaw + rel
                dirs = np.stack([np.cos(angs), np.sin(angs)], 1)
                dist, idx, dinam, nest = self.geo.rayos(np.tile([ox, oy], (k, 1)), dirs, alcance, cajas)
                ok = np.isfinite(dist)
                v_ego = np.array([c, s]) * self.vx_med
                vel = np.zeros(k)
                for i in np.nonzero(ok)[0]:
                    v_obj = self.npcs[idx[i] - nest]['vel'] if dinam[i] and idx[i] - nest < len(self.npcs) else np.zeros(2)
                    vel[i] = float(np.dot(v_obj - v_ego, dirs[i]))
                d = dist[ok] + self.rng.normal(0, float(a.get('noise_stddev', 0.1)), ok.sum())
                pts = np.stack([d * np.cos(rel[ok]), d * np.sin(rel[ok]), np.zeros(ok.sum()), vel[ok]], 1)
                self.pubs[n].publish(nube_puntos(Header(stamp=stamp, frame_id=f'{AUTO}/{n}'),
                                                 pts.astype(np.float32), ['x', 'y', 'z', 'velocity']))
                self._contar(n)

    def invasion_carril(self, x, y, yaw):
        actuales = esquinas(x, y, yaw)
        if self.esquinas_prev is not None:
            for i in self.geo.cruces_de_marca(self.esquinas_prev, actuales):
                tipo = self.geo.marcas[i][4]
                self._evento({'tipo': 'invasion_carril', 'marcas': [tipo]}, f'carril:{i}', 1.5)
        self.esquinas_prev = actuales

    def reportar(self):
        ahora = time.monotonic()
        if ahora - self.t_rep < 2.0:
            return
        dt = ahora - self.t_rep
        rtf = (self.t_sim - self.sim_rep) / dt if self.sim_rep else 1.0
        fps = (self.ciclos - self.ciclos_rep) / dt
        sensores = [{'nombre': n, 'tipo': self.specs[n]['tipo'], 'hz': round(c / dt, 1)} for n, c in self.conteo.items()]
        self.conteo = {}
        self.t_rep, self.sim_rep, self.ciclos_rep = ahora, self.t_sim, self.ciclos
        self.pub_lab.publish(String(data=json.dumps({
            'fps': round(fps, 1), 'rtf': round(rtf, 2), 'clima': 'fijo', 'equipos': [AUTO],
            'sensores': sensores, 'camara_extra': self.camara_extra, 'simulador': 'gazebo'})))
        if int(ahora) % 10 < 2:
            log(f'Ciclos {fps:.1f} Hz | tiempo real {rtf:.2f}x | ' +
                ', '.join(f'{x["nombre"]}={x["hz"]}' for x in sensores if not x['nombre'].startswith('ultra')))

    # ------------------------------------------------ TF
    def _tf(self, padre, hijo, pos, q, stamp=None):
        t = TransformStamped()
        if stamp is not None:
            t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = padre, hijo
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos)
        t.transform.rotation = q
        return t

    def publicar_tf_estaticos(self):
        tfs = []
        for s in self.sensores_cfg:
            if s['tipo'] in TIPOS_SIN_TF:
                continue
            marco = f'{AUTO}/{s["nombre"]}'
            r, p, y = (math.radians(a) for a in s.get('rot', [0, 0, 0]))
            tfs.append(self._tf(f'{AUTO}/base_link', marco, s.get('pos', [0, 0, 0]), rpy_a_quat(r, p, y)))
            if s['tipo'].startswith('sensor.camera'):
                tfs.append(self._tf(marco, marco + '_optical', [0, 0, 0], rpy_a_quat(-math.pi / 2, 0, -math.pi / 2)))
        self.tf_static.sendTransform(tfs)


def main():
    rclpy.init()
    nodo = Nucleo()
    ej = MultiThreadedExecutor(num_threads=4)
    ej.add_node(nodo)
    try:
        ej.spin()
    except (KeyboardInterrupt, Exception) as e:  # noqa: BLE001
        if not isinstance(e, KeyboardInterrupt) and rclpy.ok():
            raise
    finally:
        try:
            nodo.pub_cmd.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        ej.shutdown()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
