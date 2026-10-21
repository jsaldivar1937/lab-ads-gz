#!/usr/bin/env python3
"""Servidor web del Laboratorio Virtual ADS (Flask + ROS 2).

Un solo vehículo compartido:
- Todos los conectados ven cámara, LiDAR/radar, mapa y telemetría.
- Solo conduce quien tenga permiso del profesor (uno a la vez).
- El profesor entra por /profesor (usuario y clave de .env; por defecto
  profesor / profesor) y controla permisos, autopiloto, reinicio y clima.

Rutas principales:
  /api/stream     Telemetría en vivo (Server-Sent Events)
  /stream/<cam>   Video MJPEG
  /api/nube/<s>   LiDAR / radar reducidos (float32)
  /api/cmd        Comandos de manejo (solo con permiso)
  /api/archivos   Archivos del alumno (subir, listar, descargar)
"""
import collections
import json
import math
import os
import re
import secrets
import sys
import threading
import time
import unicodedata
from pathlib import Path

import cantools
import cv2
import numpy as np
import rclpy
import yaml
from can_msgs.msg import Frame
from flask import Flask, Response, abort, jsonify, request, send_file, send_from_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, Imu, JointState, NavSatFix, PointCloud2
from std_msgs.msg import Bool, Empty, Float32MultiArray, String

CFG_PATH = Path(os.environ.get('LAB_CONFIG', '/lab/config/lab.yaml'))
DATA_DIR = Path(os.environ.get('LAB_DATA', '/data'))
STATIC = Path(__file__).parent / 'static'
# Acceso del profesor (por defecto profesor / profesor; cámbialo en .env)
USUARIO_PROFESOR = os.environ.get('LAB_USUARIO_PROFESOR') or 'profesor'
CLAVE_PROFESOR = os.environ.get('LAB_CLAVE_PROFESOR') or 'profesor'
FPS_MAX = float(os.environ.get('LAB_FPS_VIDEO', 6))
CALIDAD_JPEG = int(os.environ.get('LAB_CALIDAD_JPEG', 65))
MAX_PUNTOS = int(os.environ.get('LAB_MAX_PUNTOS', 3000))
PERIODO_SSE = 0.15            # s entre mensajes de telemetría
AUSENTE_S = 20.0              # sin conexión por más de esto → desconectado
CLIMAS = ['ClearNoon', 'CloudyNoon', 'WetNoon', 'WetCloudyNoon', 'SoftRainNoon', 'MidRainyNoon',
          'HardRainNoon', 'ClearSunset', 'CloudySunset', 'WetSunset', 'HardRainSunset',
          'ClearNight', 'CloudyNight', 'WetNight', 'HardRainNight']

cfg = yaml.safe_load(CFG_PATH.read_text(encoding='utf-8'))
AUTO = cfg['equipos'][0]['nombre']
SENSORES = [s for s in cfg['sensores'] if s.get('activo', True)]
ULTRASONIDOS = [{'nombre': f"{s['nombre']}_{k}", 'pos': v.get('pos', [0, 0, 0]), 'rot': v.get('rot', [0, 0, 0])}
                for s in SENSORES if s.get('arreglo') for k, v in s['arreglo'].items()]
# Carpeta personal de cada alumno (volumen compartido con la terminal)
CARPETAS = Path(os.environ.get('LAB_TRABAJO', '/trabajo')) / 'alumnos'
MAX_ARCHIVO_MB = 10
RUTA_DBC = Path((cfg.get('ecu') or {}).get('dbc', '/lab/config/lab_ads.dbc'))
DBC = cantools.database.load_file(str(RUTA_DBC)) if RUTA_DBC.exists() else None

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'sim'))
from fallas import CATALOGO  # noqa: E402  (catálogo único compartido con el núcleo)

PIDS = {0x0C: ('RPM', lambda d: (d[0] * 256 + d[1]) / 4, 'rpm'),
        0x0D: ('Velocidad', lambda d: d[0], 'km/h'),
        0x05: ('Refrigerante', lambda d: d[0] - 40, '°C'),
        0x11: ('Acelerador', lambda d: round(d[0] * 100 / 255, 1), '%'),
        0x2F: ('Combustible', lambda d: round(d[0] * 100 / 255, 1), '%'),
        0x42: ('Voltaje módulo', lambda d: (d[0] * 256 + d[1]) / 1000, 'V'),
        0x01: ('Estado monitores', lambda d: f"MIL={'ON' if d[0] & 0x80 else 'OFF'}, DTC={d[0] & 0x7F}", '')}


def dtc_de_bytes(b1, b2):
    return 'PCBU'[b1 >> 6] + f'{(b1 >> 4) & 0x3:X}{b1 & 0xF:X}{b2:02X}'


def decodificar_can(can_id, datos):
    """Devuelve (nombre, texto de señales) para la tabla del sniffer."""
    if can_id == 0x7DF:
        modo = datos[1]
        if modo == 0x01:
            return 'OBD petición', f'Modo 01 · PID {datos[2]:02X} ({PIDS.get(datos[2], ("?",))[0]})'
        return 'OBD petición', f'Modo {modo:02X}'
    if can_id == 0x7E8:
        modo = datos[1]
        if modo == 0x41 and datos[2] in PIDS:
            nombre, f, unidad = PIDS[datos[2]]
            try:
                return 'OBD respuesta', f'{nombre} = {f(datos[3:])} {unidad}'.strip()
            except (IndexError, ValueError):
                return 'OBD respuesta', 'datos incompletos'
        if modo == 0x43:
            n = datos[2]
            cods = [dtc_de_bytes(datos[3 + 2 * i], datos[4 + 2 * i]) for i in range(min(n, 2))]
            return 'OBD respuesta', f'Modo 03 · {n} DTC: {", ".join(cods) or "ninguno"}'
        return 'OBD respuesta', f'Modo {modo:02X}'
    if DBC is not None:
        try:
            m = DBC.get_message_by_frame_id(can_id)
            sen = m.decode(bytes(datos), decode_choices=False)
            texto = ', '.join(f'{k}={round(v, 2) if isinstance(v, float) else v}' for k, v in sen.items())
            return m.name, texto
        except (KeyError, ValueError):
            pass
    return '?', ''


def nombre_sensor(tipo):
    return next((s['nombre'] for s in SENSORES if s['tipo'] == tipo), None)


def imagen_a_jpeg(m):
    """sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8, mono8, 32FC1) → JPEG, o None."""
    try:
        crudo = np.frombuffer(m.data, np.uint8)
        if m.encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            c = 4 if m.encoding.endswith('a8') else 3
            img = crudo.reshape(m.height, m.step // c if m.step else m.width, c)[:, :m.width]
            img = cv2.cvtColor(img, {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                                     'bgra8': cv2.COLOR_BGRA2BGR}.get(m.encoding, -1)) \
                if m.encoding != 'bgr8' else img
        elif m.encoding in ('mono8', '8UC1'):
            img = crudo.reshape(m.height, m.step or m.width)[:, :m.width]
        elif m.encoding == '32FC1':
            d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
            img = cv2.applyColorMap((255 - np.clip(np.nan_to_num(d, posinf=80), 0, 80) / 80 * 255)
                                    .astype(np.uint8), cv2.COLORMAP_TURBO)
        else:
            return None
        ok, buf = cv2.imencode('.jpg', np.ascontiguousarray(img), [cv2.IMWRITE_JPEG_QUALITY, CALIDAD_JPEG])
        return buf.tobytes() if ok else None
    except (ValueError, cv2.error):
        return None


def jpeg_sin_senal():
    img = np.full((360, 640, 3), 28, np.uint8)
    cv2.putText(img, 'Sin senal', (250, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (140, 140, 140), 2)
    return cv2.imencode('.jpg', img)[1].tobytes()


SIN_SENAL = jpeg_sin_senal()


# ============================================================ usuarios y permisos
def carpeta_de(nombre):
    """'José Pérez' → 'jose_perez' (nombre de carpeta seguro para la terminal)."""
    t = unicodedata.normalize('NFKD', nombre).encode('ascii', 'ignore').decode().lower()
    return re.sub(r'[^a-z0-9]+', '_', t).strip('_')[:40] or 'anonimo'


def nombre_archivo_seguro(nombre):
    base = os.path.basename((nombre or '').replace('\\', '/'))
    base = unicodedata.normalize('NFKD', base).encode('ascii', 'ignore').decode()
    base = re.sub(r'[^A-Za-z0-9._-]+', '_', base).strip('._')[:80]
    return base or None


class Sala:
    def __init__(self):
        self.lock = threading.Lock()
        self.usuarios = {}        # token → {uid, nombre, rol, visto}
        self.conductor = None     # token de quien conduce

    def unirse(self, nombre):
        nombre = (nombre or '').strip()[:40] or 'Anónimo'
        token = secrets.token_urlsafe(24)
        with self.lock:
            self.usuarios[token] = {'uid': secrets.token_hex(4), 'nombre': nombre,
                                    'carpeta': carpeta_de(nombre),
                                    'rol': 'alumno', 'visto': time.time()}
        return token

    def valido(self, token):
        with self.lock:
            return token in self.usuarios

    def visto(self, token):
        with self.lock:
            u = self.usuarios.get(token)
            if u:
                u['visto'] = time.time()

    def hacer_profesor(self, token, usuario, clave):
        ok_usuario = secrets.compare_digest((usuario or '').strip().lower().encode(),
                                            USUARIO_PROFESOR.lower().encode())
        ok_clave = secrets.compare_digest((clave or '').encode(), CLAVE_PROFESOR.encode())
        if not (ok_usuario and ok_clave):
            return False
        with self.lock:
            if token not in self.usuarios:
                return False
            self.usuarios[token]['rol'] = 'profesor'
        return True

    def carpeta(self, token):
        with self.lock:
            u = self.usuarios.get(token)
            return u and u.get('carpeta')

    def es_profesor(self, token):
        with self.lock:
            u = self.usuarios.get(token)
            return bool(u and u['rol'] == 'profesor')

    def puede_conducir(self, token):
        with self.lock:
            u = self.usuarios.get(token)
            return bool(u and (token == self.conductor or u['rol'] == 'profesor'))

    def dar_control(self, uid):
        with self.lock:
            anterior = self.conductor
            self.conductor = next((t for t, u in self.usuarios.items() if u['uid'] == uid), None) \
                if uid else None
            return anterior != self.conductor

    def limpiar(self):
        """Quita el control a quien se desconectó y olvida usuarios viejos."""
        ahora = time.time()
        quitado = False
        with self.lock:
            if self.conductor and ahora - self.usuarios.get(self.conductor, {}).get('visto', 0) > AUSENTE_S:
                self.conductor = None
                quitado = True
            for t in [t for t, u in self.usuarios.items() if ahora - u['visto'] > 3600]:
                del self.usuarios[t]
        return quitado

    def estado_para(self, token):
        ahora = time.time()
        with self.lock:
            yo = self.usuarios.get(token, {})
            cond = self.usuarios.get(self.conductor)
            datos = {
                'yo': yo.get('nombre'), 'rol': yo.get('rol'),
                'conduzco': token == self.conductor or yo.get('rol') == 'profesor',
                'conductor': cond['nombre'] if cond else None,
                'conectados': sum(1 for u in self.usuarios.values() if ahora - u['visto'] < AUSENTE_S),
            }
            if yo.get('rol') == 'profesor':
                datos['usuarios'] = sorted(
                    [{'uid': u['uid'], 'nombre': u['nombre'], 'rol': u['rol'],
                      'conduce': t == self.conductor}
                     for t, u in self.usuarios.items() if ahora - u['visto'] < AUSENTE_S],
                    key=lambda u: (u['rol'] != 'profesor', u['nombre'].lower()))
            return datos


sala = Sala()


# ============================================================ puente ROS 2
class Flujo:
    def __init__(self):
        self.jpeg, self.id, self.ultimo, self.espectadores = None, 0, 0.0, 0
        self.cond = threading.Condition()


class PuenteWeb(Node):
    def __init__(self):
        super().__init__('puente_web')
        rapido = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.lock = threading.Lock()
        self.flujos, self.nubes = {}, {}
        self.tele = {'estado': None, 'imu': None, 'gnss': None, 'enc': None, 'lab': None,
                     'ecu': None, 'radar': None, 'lidar_n': 0}
        self.can_tabla = {}                          # id → resumen
        self.can_log = collections.deque(maxlen=60)
        self.can_seq = 0
        self.tray = []
        self.eventos, self.id_evento = [], 0

        for s in SENSORES:
            t, n = s['tipo'], s['nombre']
            if t.startswith('sensor.camera'):
                self.flujos[n] = Flujo()
                self.create_subscription(Image, f'/{AUTO}/{n}/image',
                                         lambda m, k=n: self._cb_img(m, k), rapido)
            elif t in ('sensor.lidar.ray_cast', 'sensor.other.radar'):
                self.create_subscription(PointCloud2, f'/{AUTO}/{n}/points',
                                         lambda m, k=n, t=t: self._cb_nube(m, k, t), rapido)

        self.create_subscription(String, f'/{AUTO}/vehiculo/estado', self._cb_estado, 10)
        self.create_subscription(String, f'/{AUTO}/eventos', self._cb_evento, 10)
        self.create_subscription(String, '/lab/estado',
                                 lambda m: self._poner('lab', json.loads(m.data)), 10)
        self.create_subscription(JointState, f'/{AUTO}/encoders/joint_states',
                                 lambda m: self._poner('enc', [round(w, 2) for w in m.velocity]), rapido)
        if nombre_sensor('sensor.other.imu'):
            self.create_subscription(Imu, f'/{AUTO}/{nombre_sensor("sensor.other.imu")}/data',
                                     lambda m: self._poner('imu', {
                                         'ax': round(m.linear_acceleration.x, 3),
                                         'ay': round(m.linear_acceleration.y, 3),
                                         'az': round(m.linear_acceleration.z, 3),
                                         'gx': round(m.angular_velocity.x, 4),
                                         'gy': round(m.angular_velocity.y, 4),
                                         'gz': round(m.angular_velocity.z, 4)}), rapido)
        if nombre_sensor('sensor.other.gnss'):
            self.create_subscription(NavSatFix, f'/{AUTO}/{nombre_sensor("sensor.other.gnss")}/fix',
                                     self._cb_gnss, rapido)
        self.create_subscription(String, f'/{AUTO}/ecu/estado',
                                 lambda m: self._poner('ecu', json.loads(m.data)), 10)
        self.create_subscription(Frame, f'/{AUTO}/can/rx', self._cb_can, 200)
        self.pub_falla = self.create_publisher(String, '/lab/fallas', 10)
        self.pub_borrar = self.create_publisher(Empty, '/lab/borrar_dtc', 10)
        self.pub_camara = self.create_publisher(String, '/lab/camara_extra', 10)

        self.pub_cmd = self.create_publisher(Float32MultiArray, f'/{AUTO}/cmd/directo', 10)
        self.pub_auto = self.create_publisher(Bool, f'/{AUTO}/cmd/autopilot', 10)
        self.pub_reset = self.create_publisher(Empty, f'/{AUTO}/reset', 10)
        self.pub_clima = self.create_publisher(String, '/lab/clima', 10)

    # ------------------------------------------------ telemetría
    def _poner(self, clave, valor):
        with self.lock:
            self.tele[clave] = valor

    def _cb_estado(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.tele['estado'] = d
            u = self.tray[-1] if self.tray else None
            if u is None or math.hypot(d['x'] - u[0], d['y'] - u[1]) > 0.5:
                self.tray.append([d['x'], d['y']])
                if len(self.tray) > 2000:
                    self.tray = self.tray[-2000:]

    def _cb_gnss(self, m):
        fijo = m.status.status >= 0 and math.isfinite(m.latitude)
        self._poner('gnss', [round(m.latitude, 7), round(m.longitude, 7), round(m.altitude, 2)]
                    if fijo else None)

    def _cb_nube(self, m, clave, tipo):
        self.nubes[clave] = m
        if tipo == 'sensor.lidar.ray_cast':
            self._poner('lidar_n', m.width)
        elif m.width:
            arr = np.frombuffer(m.data, np.float32).reshape(-1, m.point_step // 4)
            d = np.linalg.norm(arr[:, :3], axis=1)
            i = int(np.argmin(d))
            self._poner('radar', {'n': int(m.width), 'dist': round(float(d[i]), 1),
                                  'vel': round(float(arr[i, 3]), 1)})
        else:
            self._poner('radar', {'n': 0, 'dist': None, 'vel': None})

    def _cb_can(self, f):
        datos = [int(b) for b in f.data[:f.dlc]]
        ahora = time.monotonic()
        try:
            nombre, senales = decodificar_can(f.id, datos)
        except Exception as e:  # noqa: BLE001  (una trama rara no debe detener el servidor)
            nombre, senales = '?', f'error al decodificar: {e}'
        hexa = ' '.join(f'{b:02X}' for b in datos)
        with self.lock:
            fila = self.can_tabla.setdefault(f.id, {'id': f.id, 'n': 0, 't': collections.deque(maxlen=20)})
            fila.update(nombre=nombre, dlc=f.dlc, datos=hexa, senales=senales)
            fila['n'] += 1
            fila['t'].append(ahora)
            self.can_seq += 1
            t = f.header.stamp.sec + f.header.stamp.nanosec * 1e-9
            self.can_log.append({'s': self.can_seq, 't': round(t, 3), 'id': f.id, 'nombre': nombre,
                                 'datos': hexa, 'senales': senales})

    def can_instantanea(self, desde):
        with self.lock:
            tabla = []
            for fila in sorted(self.can_tabla.values(), key=lambda x: x['id']):
                ts = fila['t']
                hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0
                vivo = bool(ts) and time.monotonic() - ts[-1] < 2.0
                tabla.append({k: fila[k] for k in ('id', 'nombre', 'dlc', 'datos', 'senales', 'n')} |
                             {'hz': round(hz, 1), 'vivo': vivo})
            nuevos = [x for x in self.can_log if x['s'] > desde][-25:]
            return tabla, nuevos, self.can_seq

    def _cb_evento(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.id_evento += 1
            d['id'] = self.id_evento
            self.eventos = (self.eventos + [d])[-50:]

    def instantanea(self, desde_evento):
        with self.lock:
            return dict(self.tele), [e for e in self.eventos if e['id'] > desde_evento], self.id_evento

    def trayectoria(self):
        with self.lock:
            return list(self.tray)

    def reiniciar_trayectoria(self):
        with self.lock:
            self.tray = []

    # ------------------------------------------------ video y nubes
    def _cb_img(self, m, clave):
        f = self.flujos[clave]
        ahora = time.monotonic()
        if f.espectadores == 0 or ahora - f.ultimo < 1.0 / FPS_MAX:
            return
        f.ultimo = ahora
        try:
            if m.encoding in ('bgr8', 'rgb8'):
                img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif m.encoding == '32FC1':
                d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
                img = cv2.applyColorMap((255 - np.clip(d, 0, 80) / 80.0 * 255).astype(np.uint8),
                                        cv2.COLORMAP_TURBO)
            elif m.encoding == 'mono8':
                img = cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width),
                                   cv2.COLOR_GRAY2BGR)
            else:
                return
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, CALIDAD_JPEG])
        except ValueError:
            return
        if ok:
            with f.cond:
                f.jpeg, f.id = buf.tobytes(), f.id + 1
                f.cond.notify_all()

    def nube(self, sensor):
        m = self.nubes.get(sensor)
        if m is None or m.width == 0:
            return b''
        paso = m.point_step // 4
        arr = np.frombuffer(m.data, np.float32).reshape(-1, paso)[:, :4]
        if len(arr) > MAX_PUNTOS:
            arr = arr[np.linspace(0, len(arr) - 1, MAX_PUNTOS).astype(int)]
        return np.ascontiguousarray(arr, dtype=np.float32).tobytes()

    # ------------------------------------------------ comandos
    def manejar(self, acel, giro, freno, reversa):
        self.pub_cmd.publish(Float32MultiArray(data=[acel, giro, freno, reversa]))

    def frenar(self):
        self.manejar(0.0, 0.0, 1.0, 0.0)


class Eco:
    """Suscripciones bajo demanda para ver cualquier tópico en crudo desde la web."""
    PREFIJOS = ('/auto/', '/lab/', '/alumnos/', '/clock', '/tf')

    def __init__(self, nodo):
        self.nodo = nodo
        self.lock = threading.Lock()
        self.subs = {}           # tópico → {sub, tipo, msg, tiempos, acceso}

    def topicos(self):
        return sorted([{'nombre': n, 'tipo': t[0]} for n, t in self.nodo.get_topic_names_and_types()
                       if n.startswith(self.PREFIJOS)], key=lambda x: x['nombre'])

    def obtener(self, topico):
        with self.lock:
            e = self.subs.get(topico)
            if e is None:
                tipos = dict(self.nodo.get_topic_names_and_types())
                if topico not in tipos or not topico.startswith(self.PREFIJOS):
                    return None
                tipo = tipos[topico][0]
                e = {'tipo': tipo, 'msg': None, 'tiempos': collections.deque(maxlen=30), 'n': 0}
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

                def cb(m, e=e):
                    e['msg'] = m
                    e['n'] += 1
                    e['tiempos'].append(time.monotonic())
                e['sub'] = self.nodo.create_subscription(get_message(tipo), topico, cb, qos)
                self.subs[topico] = e
            e['acceso'] = time.monotonic()
            return e

    def texto(self, e):
        m = e['msg']
        if m is None:
            return 'Esperando mensajes…'
        d = message_to_ordereddict(m, truncate_length=24)
        return yaml.safe_dump(json.loads(json.dumps(d, default=str)), sort_keys=False, allow_unicode=True,
                              width=120)

    @staticmethod
    def hz(e):
        t = e['tiempos']
        return round((len(t) - 1) / (t[-1] - t[0]), 1) if len(t) > 1 and t[-1] > t[0] else 0.0

    def limpiar(self):
        with self.lock:
            for topico in [k for k, e in self.subs.items() if time.monotonic() - e['acceso'] > 15]:
                self.nodo.destroy_subscription(self.subs.pop(topico)['sub'])


rclpy.init()
puente = PuenteWeb()
eco = Eco(puente)
threading.Thread(target=rclpy.spin, args=(puente,), daemon=True).start()


def vigilante():
    while True:
        time.sleep(2)
        if sala.limpiar():
            puente.frenar()
        eco.limpiar()


threading.Thread(target=vigilante, daemon=True).start()

app = Flask(__name__, static_folder=str(STATIC), static_url_path='/static')
app.config['MAX_CONTENT_LENGTH'] = MAX_ARCHIVO_MB * 1024 * 1024


def token_de_peticion():
    datos = request.get_json(silent=True) or {}
    return datos.get('token') or request.args.get('token', '')


# ============================================================ rutas
@app.route('/')
def inicio():
    return send_from_directory(STATIC, 'index.html')


@app.route('/profesor')
def vista_profesor():
    return send_from_directory(STATIC, 'profesor.html')


@app.route('/api/config')
def api_config():
    e = cfg['equipos'][0]
    return jsonify({
        'auto': AUTO, 'color': e.get('color', '245,165,36'),
        'camaras': [{'nombre': s['nombre'], 'tipo': s['tipo'].split('.')[-1],
                     'bajo_demanda': bool(s.get('bajo_demanda')), 'hz': s.get('hz'),
                     'res': [s.get('attrs', {}).get('image_size_x'), s.get('attrs', {}).get('image_size_y')]}
                    for s in SENSORES if s['tipo'].startswith('sensor.camera')],
        'ultrasonido': ULTRASONIDOS,
        'sensores': [{'nombre': s['nombre'], 'tipo': s['tipo'], 'hz': s.get('hz'),
                      'pos': s.get('pos'), 'attrs': s.get('attrs', {}),
                      'bajo_demanda': bool(s.get('bajo_demanda')),
                      'cantidad': len(s.get('arreglo') or {}) or 1} for s in SENSORES],
        'fallas': [{'codigo': c, **v} for c, v in CATALOGO.items()],
        'can_mensajes': [{'id': m.frame_id, 'nombre': m.name, 'comentario': m.comment or ''}
                         for m in DBC.messages] if DBC else [],
        'lidar': [{'nombre': s['nombre'], 'pos': s.get('pos', [0, 0, 0])}
                  for s in SENSORES if s['tipo'] == 'sensor.lidar.ray_cast'],
        'radar': [{'nombre': s['nombre'], 'pos': s.get('pos', [0, 0, 0])}
                  for s in SENSORES if s['tipo'] == 'sensor.other.radar'],
        'climas': [], 'clima': '',          # en Gazebo la iluminación es fija
        'mapa': cfg.get('simulador', {}).get('mundo', ''),
        'simulador': 'Gazebo Harmonic · ROS 2 Jazzy',
    })


@app.route('/api/mapa')
def api_mapa():
    ruta = DATA_DIR / 'mapa.json'
    if not ruta.exists():
        return jsonify({'error': 'El mapa aún no se genera; ¿está corriendo el núcleo?'}), 503
    return Response(ruta.read_text(), mimetype='application/json')


@app.route('/api/unirse', methods=['POST'])
def api_unirse():
    datos = request.get_json(silent=True) or {}
    return jsonify({'token': sala.unirse(datos.get('nombre'))})


@app.route('/api/yo')
def api_yo():
    t = token_de_peticion()
    if not sala.valido(t):
        return jsonify({'valido': False}), 401
    return jsonify({'valido': True, **sala.estado_para(t)})


@app.route('/api/profesor', methods=['POST'])
def api_profesor():
    datos = request.get_json(silent=True) or {}
    if sala.hacer_profesor(datos.get('token'), datos.get('usuario'), datos.get('clave')):
        return jsonify({'ok': True})
    return jsonify({'ok': False, 'error': 'Usuario o clave incorrectos'}), 403


@app.route('/api/stream')
def api_stream():
    token = request.args.get('token', '')
    if not sala.valido(token):
        abort(401)

    def generar():
        desde = 0
        desde_can = 0
        primero = True
        vuelta = 0
        while True:
            sala.visto(token)
            tele, eventos, desde = puente.instantanea(desde)
            paquete = {**tele, 'eventos': eventos, 'control': sala.estado_para(token)}
            tabla, nuevos, desde_can = puente.can_instantanea(desde_can)
            paquete['can_log'] = nuevos
            if vuelta % 2 == 0:
                paquete['can'] = tabla
            vuelta += 1
            if primero:
                paquete['tray'] = puente.trayectoria()
                primero = False
            yield f'data: {json.dumps(paquete)}\n\n'
            time.sleep(PERIODO_SSE)

    return Response(generar(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no'})


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    datos = request.get_json(silent=True) or {}
    t = datos.get('token', '')
    if not sala.puede_conducir(t):
        return jsonify({'ok': False, 'error': 'Sin permiso para conducir'}), 403
    d = (list(datos.get('d', [])) + [0, 0, 0, 0])[:4]
    try:
        acel, giro, freno, rev = (float(x) for x in d)
    except (TypeError, ValueError):
        abort(400)
    puente.manejar(float(np.clip(acel, 0, 1)), float(np.clip(giro, -1, 1)),
                   float(np.clip(freno, 0, 1)), 1.0 if rev > 0.5 else 0.0)
    sala.visto(t)
    return jsonify({'ok': True})


@app.route('/api/admin/control', methods=['POST'])
def api_admin_control():
    datos = request.get_json(silent=True) or {}
    if not sala.es_profesor(datos.get('token')):
        abort(403)
    if sala.dar_control(datos.get('uid')):
        puente.frenar()
    return jsonify({'ok': True})


@app.route('/api/admin/accion', methods=['POST'])
def api_admin_accion():
    datos = request.get_json(silent=True) or {}
    if not sala.es_profesor(datos.get('token')):
        abort(403)
    accion, valor = datos.get('accion'), datos.get('valor')
    if accion == 'autopilot':
        puente.pub_auto.publish(Bool(data=bool(valor)))
    elif accion == 'reset':
        puente.pub_reset.publish(Empty())
        puente.reiniciar_trayectoria()
    elif accion == 'clima' and valor in CLIMAS:
        puente.pub_clima.publish(String(data=valor))
    else:
        abort(400)
    return jsonify({'ok': True})


@app.route('/api/admin/falla', methods=['POST'])
def api_admin_falla():
    datos = request.get_json(silent=True) or {}
    if not sala.es_profesor(datos.get('token')):
        abort(403)
    if datos.get('codigo') not in CATALOGO:
        abort(400)
    puente.pub_falla.publish(String(data=json.dumps({'codigo': datos['codigo'],
                                                     'activo': bool(datos.get('activo'))})))
    return jsonify({'ok': True})


@app.route('/api/admin/borrar_dtc', methods=['POST'])
def api_admin_borrar_dtc():
    if not sala.es_profesor((request.get_json(silent=True) or {}).get('token')):
        abort(403)
    puente.pub_borrar.publish(Empty())
    return jsonify({'ok': True})


@app.route('/api/admin/camara', methods=['POST'])
def api_admin_camara():
    datos = request.get_json(silent=True) or {}
    if not sala.es_profesor(datos.get('token')):
        abort(403)
    puente.pub_camara.publish(String(data=str(datos.get('nombre') or '')))
    return jsonify({'ok': True})


@app.route('/api/topicos')
def api_topicos():
    if not sala.valido(request.args.get('token', '')):
        abort(401)
    return jsonify(eco.topicos())


@app.route('/api/eco')
def api_eco():
    token, topico = request.args.get('token', ''), request.args.get('topico', '')
    if not sala.valido(token):
        abort(401)
    if eco.obtener(topico) is None:
        abort(404)

    def generar():
        while True:
            e = eco.obtener(topico)
            if e is None:
                break
            paquete = {'topico': topico, 'tipo': e['tipo'], 'hz': eco.hz(e), 'n': e['n'], 'texto': eco.texto(e)}
            yield f'data: {json.dumps(paquete)}\n\n'
            time.sleep(0.3)

    return Response(generar(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no'})


# ---------------------------------------------------------- archivos del alumno
def carpeta_alumno():
    c = sala.carpeta(token_de_peticion() or request.form.get('token', ''))
    if not c:
        abort(401)
    d = CARPETAS / c
    d.mkdir(parents=True, exist_ok=True)
    return c, d


@app.route('/api/archivos')
def api_archivos():
    c, d = carpeta_alumno()
    lista = sorted((f for f in d.iterdir() if f.is_file()), key=lambda f: -f.stat().st_mtime)
    return jsonify({'carpeta': c, 'ruta': f'/trabajo/alumnos/{c}', 'max_mb': MAX_ARCHIVO_MB,
                    'archivos': [{'nombre': f.name, 'bytes': f.stat().st_size,
                                  'fecha': f.stat().st_mtime} for f in lista[:200]]})


@app.route('/api/archivos', methods=['POST'])
def api_subir():
    c, d = carpeta_alumno()
    guardados = []
    for f in request.files.getlist('archivo'):
        nombre = nombre_archivo_seguro(f.filename)
        if nombre:
            f.save(d / nombre)
            (d / nombre).chmod(0o755 if nombre.endswith(('.py', '.sh')) else 0o644)
            guardados.append(nombre)
    if not guardados:
        return jsonify({'ok': False, 'error': 'No se recibió ningún archivo'}), 400
    return jsonify({'ok': True, 'guardados': guardados, 'ruta': f'/trabajo/alumnos/{c}'})


@app.route('/api/archivos/<nombre>')
def api_descargar(nombre):
    _, d = carpeta_alumno()
    nombre = nombre_archivo_seguro(nombre)
    if not nombre or not (d / nombre).is_file():
        abort(404)
    return send_from_directory(d, nombre, as_attachment=True)


@app.errorhandler(413)
def muy_grande(_):
    return jsonify({'ok': False, 'error': f'El archivo pasa de {MAX_ARCHIVO_MB} MB'}), 413


@app.route('/api/eco_video')
def api_eco_video():
    """Video MJPEG de cualquier tópico de imagen (p. ej. /alumnos/<nombre>/camara_procesada)."""
    token, topico = request.args.get('token', ''), request.args.get('topico', '')
    if not sala.valido(token):
        abort(401)
    e = eco.obtener(topico)
    if e is None or e['tipo'] != 'sensor_msgs/msg/Image':
        abort(404)

    def generar():
        ultimo, t_ult = -1, 0.0
        while True:
            e = eco.obtener(topico)
            if e is None:
                break
            jpg = None
            if e['n'] != ultimo and e['msg'] is not None:
                ultimo, t_ult = e['n'], time.monotonic()
                jpg = imagen_a_jpeg(e['msg']) or SIN_SENAL
            elif time.monotonic() - t_ult > 5:          # el alumno detuvo su programa
                jpg, t_ult = SIN_SENAL, time.monotonic()
            if jpg:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                       str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
            time.sleep(1.0 / FPS_MAX)

    return Response(generar(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no'})


@app.route('/api/dbc')
def api_dbc():
    if not RUTA_DBC.exists():
        abort(404)
    return send_file(RUTA_DBC, as_attachment=True, download_name='lab_ads.dbc', mimetype='text/plain')


@app.route('/api/nube/<sensor>')
def api_nube(sensor):
    return Response(puente.nube(sensor), mimetype='application/octet-stream',
                    headers={'Cache-Control': 'no-store'})


@app.route('/stream/<camara>')
def stream(camara):
    f = puente.flujos.get(camara)
    if f is None:
        abort(404)

    def generar():
        with f.cond:
            f.espectadores += 1
        try:
            ultimo = -1
            while True:
                with f.cond:
                    f.cond.wait_for(lambda: f.id != ultimo, timeout=2.0)
                    jpg, ultimo = f.jpeg, f.id
                jpg = jpg if jpg is not None and ultimo > 0 else SIN_SENAL
                yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                       str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
        finally:
            with f.cond:
                f.espectadores -= 1

    return Response(generar(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-store', 'X-Accel-Buffering': 'no'})


if __name__ == '__main__':
    if CLAVE_PROFESOR == 'profesor':
        print('[web] AVISO: se usa la clave por defecto del profesor; cámbiala en .env', flush=True)
    print(f'[web] Visualizador en http://0.0.0.0:5000 (vehículo: {AUTO})', flush=True)
    app.run(host='0.0.0.0', port=5000, threaded=True)
