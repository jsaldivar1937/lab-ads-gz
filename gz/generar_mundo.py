#!/usr/bin/env python3
"""Genera la ciudad del laboratorio para Gazebo Harmonic.

Salidas (en la carpeta de datos, por defecto /data):
  ciudad.sdf    mundo con calles, marcas viales, banquetas, edificios, señales,
                estacionamiento, autos de tráfico y el vehículo con sus sensores
  ciudad.json   geometría para el núcleo: rutas, obstáculos, marcas viales
  mapa.json     carriles para el mapa de la web
  puente.yaml   configuración de ros_gz_bridge (tópicos Gazebo ↔ ROS 2)

Ciudad ligera a propósito: Gazebo dibuja por software dentro de Docker en
Windows, así que se usan cajas con colores en lugar de mallas pesadas.

Etiquetas de la cámara de segmentación:
  1 calle · 2 banqueta · 3 edificio · 4 marca vial · 5 señal · 6 vehículo
  7 pasto · 8 estacionamiento
"""
import json
import math
import random
import sys
from pathlib import Path

import yaml

CFG = Path(sys.argv[1] if len(sys.argv) > 1 else '/lab/config/lab.yaml')
SALIDA = Path(sys.argv[2] if len(sys.argv) > 2 else '/data')

cfg = yaml.safe_load(CFG.read_text(encoding='utf-8'))
SIM = cfg.get('simulador', {})
CIU = cfg.get('ciudad', {})
MUNDO = SIM.get('mundo', 'ciudad')
P = float(CIU.get('separacion_calles', 60))
W = float(CIU.get('ancho_calle', 9.0))
CARRIL = 1.75                       # distancia del eje de la calle al centro del carril
COORDS = [-P, 0.0, P]
AUTO = cfg['equipos'][0]['nombre']
rng = random.Random(int(CIU.get('semilla', 7)))

COLORES = {
    'asfalto': (0.18, 0.18, 0.2), 'banqueta': (0.62, 0.62, 0.6), 'pasto': (0.32, 0.5, 0.26),
    'blanco': (0.95, 0.95, 0.95), 'amarillo': (0.95, 0.78, 0.1), 'rojo': (0.8, 0.08, 0.08),
    'gris': (0.45, 0.45, 0.47), 'lote': (0.24, 0.24, 0.26),
}
EDIFICIOS = [(0.72, 0.62, 0.5), (0.55, 0.6, 0.68), (0.78, 0.74, 0.66), (0.5, 0.45, 0.42),
             (0.66, 0.5, 0.42), (0.6, 0.66, 0.6)]
AUTOS = [(0.75, 0.1, 0.1), (0.1, 0.3, 0.7), (0.9, 0.9, 0.9), (0.15, 0.15, 0.15),
         (0.2, 0.55, 0.3), (0.7, 0.7, 0.72)]

obstaculos = []      # cajas 2D para ultrasonido y radar emulados
marcas = []          # segmentos de marcas viales para detectar invasión de carril
carriles = []        # polilíneas para el mapa web


# ============================================================ utilidades SDF
def material(rgb):
    r, g, b = rgb
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular></material>')


def etiqueta(n):
    return (f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
            f'<label>{n}</label></plugin>') if n else ''


def caja(nombre, x, y, z, sx, sy, sz, rgb, yaw=0.0, col=True, et=None):
    pose = f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>'
    geo = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    s = f'<visual name="{nombre}_v">{pose}{geo}{material(rgb)}{etiqueta(et)}</visual>'
    if col:
        s += f'<collision name="{nombre}_c">{pose}{geo}</collision>'
    return s


def cilindro(nombre, x, y, z, r, h, rgb, col=True, et=None, rpy=(0, 0, 0)):
    pose = f'<pose>{x:.3f} {y:.3f} {z:.3f} {rpy[0]} {rpy[1]} {rpy[2]}</pose>'
    geo = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
    s = f'<visual name="{nombre}_v">{pose}{geo}{material(rgb)}{etiqueta(et)}</visual>'
    if col:
        s += f'<collision name="{nombre}_c">{pose}{geo}</collision>'
    return s


def modelo_estatico(nombre, contenido):
    return (f'<model name="{nombre}"><static>true</static>'
            f'<link name="link">{contenido}</link></model>')


def obstaculo(x, y, sx, sy, h, tipo, yaw=0.0):
    obstaculos.append({'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4),
                       'sx': round(sx, 3), 'sy': round(sy, 3), 'h': h, 'tipo': tipo})


# ============================================================ calles y marcas
def calles():
    s = ''
    largo = 2 * P + W
    for i, c in enumerate(COORDS):
        s += caja(f'calle_x{i}', 0, c, 0.01, largo, W, 0.02, COLORES['asfalto'], et=1)
        s += caja(f'calle_y{i}', c, 0, 0.0105, W, largo, 0.021, COLORES['asfalto'], et=1)
        for sgn in (-1, 1):
            carriles.append([[-P - W / 2, c + sgn * CARRIL], [P + W / 2, c + sgn * CARRIL]])
            carriles.append([[c + sgn * CARRIL, -P - W / 2], [c + sgn * CARRIL, P + W / 2]])
    # marcas entre intersecciones
    k = 0
    for c in COORDS:
        for a, b in zip(COORDS[:-1], COORDS[1:]):
            ini, fin = a + W / 2, b - W / 2
            largo_t, medio = fin - ini, (ini + fin) / 2
            for eje in ('x', 'y'):
                def seg(off, ancho, rgb, tipo, nombre):
                    if eje == 'x':
                        marcas.append([ini, c + off, fin, c + off, tipo])
                        return caja(nombre, medio, c + off, 0.024, largo_t, ancho, 0.004, rgb, col=False, et=4)
                    marcas.append([c + off, ini, c + off, fin, tipo])
                    return caja(nombre, c + off, medio, 0.024, ancho, largo_t, 0.004, rgb, col=False, et=4)
                k += 1
                s += seg(0.0, 0.15, COLORES['amarillo'], 'amarilla_continua', f'centro{k}')
                s += seg(W / 2 - 0.35, 0.12, COLORES['blanco'], 'blanca_continua', f'borde_a{k}')
                s += seg(-(W / 2 - 0.35), 0.12, COLORES['blanco'], 'blanca_continua', f'borde_b{k}')
                # líneas de alto en ambos extremos, sobre el carril que llega
                if eje == 'x':
                    s += caja(f'alto_a{k}', fin - 0.6, c - CARRIL, 0.024, 0.4, 3.3, 0.004, COLORES['blanco'], col=False, et=4)
                    s += caja(f'alto_b{k}', ini + 0.6, c + CARRIL, 0.024, 0.4, 3.3, 0.004, COLORES['blanco'], col=False, et=4)
                else:
                    s += caja(f'alto_a{k}', c + CARRIL, fin - 0.6, 0.024, 3.3, 0.4, 0.004, COLORES['blanco'], col=False, et=4)
                    s += caja(f'alto_b{k}', c - CARRIL, ini + 0.6, 0.024, 3.3, 0.4, 0.004, COLORES['blanco'], col=False, et=4)
    return modelo_estatico('calles', s)


# ============================================================ manzanas
def manzanas():
    s_banq, s_edif, s_lote, extra = '', '', '', []
    k = 0
    for i in range(len(COORDS) - 1):
        for j in range(len(COORDS) - 1):
            x0, x1 = COORDS[i] + W / 2, COORDS[i + 1] - W / 2
            y0, y1 = COORDS[j] + W / 2, COORDS[j + 1] - W / 2
            cx, cy, bx, by = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
            if CIU.get('estacionamiento', True) and (i, j) == (1, 1):
                s_lote += caja('lote', cx, cy, 0.01, bx, by, 0.02, COLORES['lote'], et=8)
                extra.append(estacionamiento(x0, x1, y0, y1))
                continue
            s_banq += caja(f'manzana{i}{j}', cx, cy, 0.075, bx, by, 0.15, COLORES['banqueta'], et=2)
            # 2×2 edificios por manzana
            m, hueco = 4.0, 6.0
            ex, ey = (bx - 2 * m - hueco) / 2, (by - 2 * m - hueco) / 2
            for a in range(2):
                for b in range(2):
                    k += 1
                    alto = rng.uniform(8, 28)
                    px = x0 + m + ex / 2 + a * (ex + hueco)
                    py = y0 + m + ey / 2 + b * (ey + hueco)
                    s_edif += caja(f'edif{k}', px, py, 0.15 + alto / 2, ex, ey, alto,
                                   rng.choice(EDIFICIOS), et=3)
                    obstaculo(px, py, ex, ey, alto, 'edificio')
            # la banqueta también es obstáculo bajo para el ultrasonido (borde)
    return (modelo_estatico('manzanas', s_banq + s_lote) + modelo_estatico('edificios', s_edif)
            + ''.join(extra))


def auto_visual(nombre, color, col=True):
    """Carrocería sencilla: cuerpo, cabina, 4 ruedas."""
    s = caja(f'{nombre}_cuerpo', 0, 0, 0.65, 4.4, 1.8, 0.7, color, col=col, et=6)
    s += caja(f'{nombre}_cabina', -0.2, 0, 1.22, 2.3, 1.6, 0.5, (0.12, 0.14, 0.18), col=False, et=6)
    for ix, px in enumerate((1.35, -1.35)):
        for iy, py in enumerate((0.82, -0.82)):
            s += cilindro(f'{nombre}_r{ix}{iy}', px, py, 0.33, 0.33, 0.24, (0.05, 0.05, 0.05),
                          col=False, et=6, rpy=(1.5708, 0, 0))
    return s


def estacionamiento(x0, x1, y0, y1):
    """Fila de cajones al norte del lote con autos estacionados."""
    s, s_autos = '', ''
    ancho, fondo = 2.8, 5.5
    n = int((x1 - x0 - 6) // ancho)
    ini = x0 + 3
    yl = y1 - 1.0 - fondo / 2
    for k in range(n + 1):
        x = ini + k * ancho
        s += caja(f'cajon{k}', x, yl, 0.024, 0.12, fondo, 0.004, COLORES['blanco'], col=False, et=4)
    s += caja('cajon_fondo', (ini + ini + n * ancho) / 2, y1 - 1.0, 0.024, n * ancho, 0.12, 0.004,
              COLORES['blanco'], col=False, et=4)
    ocupados = sorted(rng.sample(range(n), max(1, int(n * 0.55))))
    for k in ocupados:
        x = ini + (k + 0.5) * ancho
        s_autos += (f'<model name="estacionado{k}"><static>true</static><pose>{x:.2f} {yl:.2f} 0 0 0 1.5708</pose>'
                    f'<link name="link">{auto_visual(f"e{k}", rng.choice(AUTOS))}</link></model>')
        obstaculo(x, yl, 1.8, 4.4, 1.5, 'auto_estacionado', yaw=0.0)
    return modelo_estatico('estacionamiento', s) + s_autos


# ============================================================ señales
def senales():
    s = ''
    k = 0
    # altos en las 4 entradas al cruce central
    for (x, y) in ((-W / 2 - 1.5, -W / 2 - 1.5), (W / 2 + 1.5, W / 2 + 1.5),
                   (W / 2 + 1.5, -W / 2 - 1.5), (-W / 2 - 1.5, W / 2 + 1.5)):
        k += 1
        s += cilindro(f'poste{k}', x, y, 1.25, 0.05, 2.2, COLORES['gris'], et=5)
        s += cilindro(f'alto{k}', x, y, 2.45, 0.38, 0.04, COLORES['rojo'], et=5, rpy=(0, 1.5708, 0))
        obstaculo(x, y, 0.15, 0.15, 2.5, 'senal')
    # límites de velocidad sobre el anillo
    for (x, y) in ((-P / 2, -P - W / 2 - 1.2), (P + W / 2 + 1.2, -P / 2)):
        k += 1
        s += cilindro(f'poste{k}', x, y, 1.25, 0.05, 2.2, COLORES['gris'], et=5)
        s += caja(f'limite{k}', x, y, 2.5, 0.05, 0.6, 0.75, COLORES['blanco'], et=5)
        obstaculo(x, y, 0.15, 0.15, 2.5, 'senal')
    return modelo_estatico('senales', s)


# ============================================================ rutas
def ruta(esquinas, paso=2.0):
    """Recorre las esquinas (cruces) por el carril derecho (circulación por la derecha)."""
    pts = []
    n = len(esquinas)
    for a in range(n):
        (x0, y0), (x1, y1) = esquinas[a], esquinas[(a + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        rx, ry = uy, -ux                 # derecha de la dirección de avance
        m = int(L // paso)
        for t in range(m):
            d = t * paso
            pts.append([round(x0 + ux * d + rx * CARRIL, 2), round(y0 + uy * d + ry * CARRIL, 2)])
    return pts


def rutas():
    c = COORDS
    r = {
        'anillo_antihorario': ruta([(c[0], c[0]), (c[2], c[0]), (c[2], c[2]), (c[0], c[2])]),
        'anillo_horario': ruta([(c[0], c[0]), (c[0], c[2]), (c[2], c[2]), (c[2], c[0])]),
        'centro_oeste': ruta([(c[0], c[1]), (c[1], c[1]), (c[1], c[2]), (c[0], c[2])]),
        'centro_sur': ruta([(c[1], c[0]), (c[2], c[0]), (c[2], c[1]), (c[1], c[1])][::-1]),
        'manzana_so': ruta([(c[0], c[0]), (c[1], c[0]), (c[1], c[1]), (c[0], c[1])]),
    }
    return r


# ============================================================ vehículos
def npc(nombre, x, y, yaw, color):
    return (f'<model name="{nombre}"><pose>{x:.2f} {y:.2f} 0.02 0 0 {yaw:.4f}</pose>'
            f'<link name="link"><inertial><mass>1200</mass><inertia><ixx>400</ixx><iyy>1600</iyy>'
            f'<izz>1800</izz></inertia></inertial>'
            f'<collision name="cuerpo"><pose>0 0 0.8 0 0 0</pose><geometry><box><size>4.4 1.8 1.0</size>'
            f'</box></geometry></collision>{ruedas_colision()}{auto_visual(nombre, color, col=False)}</link>'
            f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>'
            f'{POSE_PUB}</model>')


POSE_PUB = ('<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">'
            '<publish_link_pose>false</publish_link_pose><publish_sensor_pose>false</publish_sensor_pose>'
            '<publish_collision_pose>false</publish_collision_pose><publish_visual_pose>false</publish_visual_pose>'
            '<publish_nested_model_pose>false</publish_nested_model_pose><publish_model_pose>true</publish_model_pose>'
            '<use_pose_vector_msg>false</use_pose_vector_msg><static_publisher>false</static_publisher>'
            '<update_frequency>50</update_frequency></plugin>')


def ruedas_colision():
    s = ''
    for ix, px in enumerate((1.35, -1.35)):
        for iy, py in enumerate((0.82, -0.82)):
            s += (f'<collision name="rueda_{ix}{iy}"><pose>{px} {py} 0.33 0 0 0</pose><geometry><sphere>'
                  f'<radius>0.33</radius></sphere></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2>'
                  f'</ode></friction></surface></collision>')
    return s


def rad(g):
    return math.radians(float(g))


def sensores_sdf():
    s = ''
    for sen in cfg['sensores']:
        if not sen.get('activo', True):
            continue
        t, n = sen['tipo'], sen['nombre']
        a = sen.get('attrs', {}) or {}
        x, y, z = sen.get('pos', [0, 0, 0])
        r, p, yw = (rad(v) for v in sen.get('rot', [0, 0, 0]))
        pose = f'<pose>{x} {y} {z} {r} {p} {yw}</pose>'
        hz = sen.get('hz', 10)
        comun = f'{pose}<always_on>1</always_on><update_rate>{hz}</update_rate><visualize>false</visualize>'
        if t in ('sensor.camera.rgb', 'sensor.camera.depth', 'sensor.camera.semantic_segmentation'):
            w, h, fov = a.get('image_size_x', 640), a.get('image_size_y', 360), rad(a.get('fov', 90))
            if t == 'sensor.camera.rgb':
                tipo, fmt, top, extra = 'camera', 'R8G8B8', f'/gz/{AUTO}/{n}/image', ''
            elif t == 'sensor.camera.depth':
                tipo, fmt, top, extra = 'depth_camera', 'R_FLOAT32', f'/gz/{AUTO}/{n}/image', ''
            else:
                tipo, fmt, top = 'segmentation', 'R8G8B8', f'/gz/{AUTO}/{n}'
                extra = '<segmentation_type>semantic</segmentation_type>'
            s += (f'<sensor name="{n}" type="{tipo}">{comun}<topic>{top}</topic>'
                  f'<gz_frame_id>{AUTO}/{n}</gz_frame_id><camera>{extra}<horizontal_fov>{fov:.4f}</horizontal_fov>'
                  f'<image><width>{w}</width><height>{h}</height><format>{fmt}</format></image>'
                  f'<clip><near>0.1</near><far>150</far></clip></camera></sensor>')
        elif t == 'sensor.lidar.ray_cast':
            s += (f'<sensor name="{n}" type="gpu_lidar">{comun}<topic>/gz/{AUTO}/{n}</topic>'
                  f'<gz_frame_id>{AUTO}/{n}</gz_frame_id><lidar><scan>'
                  f'<horizontal><samples>{a.get("horizontal_samples", 720)}</samples><resolution>1</resolution>'
                  f'<min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>'
                  f'<vertical><samples>{a.get("channels", 16)}</samples><resolution>1</resolution>'
                  f'<min_angle>{rad(a.get("lower_fov", -20)):.4f}</min_angle>'
                  f'<max_angle>{rad(a.get("upper_fov", 10)):.4f}</max_angle></vertical></scan>'
                  f'<range><min>0.5</min><max>{a.get("range", 60)}</max><resolution>0.01</resolution></range>'
                  f'<noise><type>gaussian</type><mean>0</mean><stddev>{a.get("noise_stddev", 0.01)}</stddev>'
                  f'</noise></lidar></sensor>')
        elif t == 'sensor.other.imu':
            def ruido(sd):
                return ''.join(f'<{e}><noise type="gaussian"><mean>0</mean><stddev>{sd}</stddev></noise></{e}>'
                               for e in ('x', 'y', 'z'))
            s += (f'<sensor name="{n}" type="imu">{comun}<topic>/gz/{AUTO}/{n}</topic>'
                  f'<gz_frame_id>{AUTO}/{n}</gz_frame_id><imu>'
                  f'<angular_velocity>{ruido(a.get("noise_gyro", 0.005))}</angular_velocity>'
                  f'<linear_acceleration>{ruido(a.get("noise_accel", 0.05))}</linear_acceleration></imu></sensor>')
        elif t == 'sensor.other.gnss':
            # Gazebo aplica el ruido horizontal en grados: se convierte de metros a grados
            s += (f'<sensor name="{n}" type="navsat">{comun}<topic>/gz/{AUTO}/{n}</topic>'
                  f'<gz_frame_id>{AUTO}/{n}</gz_frame_id><navsat><position_sensing>'
                  f'<horizontal><noise type="gaussian"><mean>0</mean><stddev>{float(a.get("noise_horizontal", 1.5)) / 111320.0:.8f}'
                  f'</stddev></noise></horizontal><vertical><noise type="gaussian"><mean>0</mean>'
                  f'<stddev>{a.get("noise_vertical", 3.0)}</stddev></noise></vertical>'
                  f'</position_sensing></navsat></sensor>')
        elif t == 'sensor.other.collision':
            s += (f'<sensor name="{n}" type="contact"><always_on>1</always_on><update_rate>20</update_rate>'
                  f'<topic>/gz/{AUTO}/contacto</topic><contact><collision>chasis</collision>'
                  f'<topic>/gz/{AUTO}/contacto</topic></contact></sensor>')
    return s


def ego(x, y, yaw):
    color = tuple(int(v) / 255 for v in cfg['equipos'][0].get('color', '245,165,36').split(','))
    return (f'<model name="{AUTO}"><pose>{x:.2f} {y:.2f} 0.02 0 0 {yaw:.4f}</pose>'
            f'<link name="base_link"><inertial><mass>1500</mass><inertia><ixx>500</ixx><iyy>2000</iyy>'
            f'<izz>2200</izz></inertia></inertial>'
            f'<collision name="chasis"><pose>0 0 0.8 0 0 0</pose><geometry><box><size>4.5 1.85 1.0</size>'
            f'</box></geometry></collision>{ruedas_colision()}{auto_visual("ego", color, col=False)}'
            f'{sensores_sdf()}</link>'
            f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>'
            f'{POSE_PUB}</model>')


# ============================================================ mundo
def mundo(rs):
    npcs, s_npc = [], ''
    n_npc = int(cfg.get('trafico', {}).get('vehiculos', 5))
    nombres_ruta = ['anillo_horario', 'centro_oeste', 'centro_sur', 'manzana_so', 'anillo_antihorario']
    for k in range(n_npc):
        nombre_r = nombres_ruta[k % len(nombres_ruta)]
        pts = rs[nombre_r]
        idx = (len(pts) // 3 + k * 17) % len(pts) if nombre_r == 'anillo_antihorario' else (k * 23) % len(pts)
        (x, y), (x2, y2) = pts[idx], pts[(idx + 1) % len(pts)]
        yaw = math.atan2(y2 - y, x2 - x)
        s_npc += npc(f'npc_{k}', x, y, yaw, AUTOS[k % len(AUTOS)])
        npcs.append({'nombre': f'npc_{k}', 'ruta': nombre_r, 'inicio': idx})

    pts = rs['anillo_antihorario']
    (x, y), (x2, y2) = pts[8], pts[9]
    ego_pose = {'x': x, 'y': y, 'yaw': math.atan2(y2 - y, x2 - x), 'ruta': 'anillo_antihorario'}

    lat, lon, ele = SIM.get('latitud', 22.7709), SIM.get('longitud', -102.5833), SIM.get('elevacion', 2440)
    sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
<world name="{MUNDO}">
  <physics name="paso" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>{lat}</latitude_deg><longitude_deg>{lon}</longitude_deg>
    <elevation>{ele}</elevation><heading_deg>0</heading_deg>
  </spherical_coordinates>
  <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.62 0.75 0.9 1</background>
    <shadows>false</shadows><grid>false</grid></scene>
  <light type="directional" name="sol"><cast_shadows>false</cast_shadows><pose>0 0 50 0 0 0</pose>
    <diffuse>0.9 0.9 0.88 1</diffuse><specular>0.2 0.2 0.2 1</specular>
    <direction>-0.4 0.3 -0.85</direction></light>
  <model name="suelo"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
      {material(COLORES['pasto'])}{etiqueta(7)}</visual></link></model>
  {calles()}
  {manzanas()}
  {senales()}
  {s_npc}
  {ego(ego_pose['x'], ego_pose['y'], ego_pose['yaw'])}
</world>
</sdf>
'''
    return sdf, npcs, ego_pose


def puente(npcs):
    def e(ros, gz, rt, gt, d, lazy=False):
        return {'ros_topic_name': ros, 'gz_topic_name': gz, 'ros_type_name': rt, 'gz_type_name': gt,
                'direction': d, 'lazy': lazy}
    b = [e('/clock', f'/world/{MUNDO}/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'),
         e(f'/gz/{AUTO}/pose', f'/model/{AUTO}/pose', 'geometry_msgs/msg/TransformStamped', 'gz.msgs.Pose',
           'GZ_TO_ROS'),
         e(f'/gz/{AUTO}/cmd_vel', f'/model/{AUTO}/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist',
           'ROS_TO_GZ')]
    for n in npcs:
        b.append(e(f'/gz/{n["nombre"]}/cmd_vel', f'/model/{n["nombre"]}/cmd_vel', 'geometry_msgs/msg/Twist',
                   'gz.msgs.Twist', 'ROS_TO_GZ'))
        b.append(e(f'/gz/{n["nombre"]}/pose', f'/model/{n["nombre"]}/pose', 'geometry_msgs/msg/TransformStamped',
                   'gz.msgs.Pose', 'GZ_TO_ROS'))
    for sen in cfg['sensores']:
        if not sen.get('activo', True):
            continue
        t, n = sen['tipo'], sen['nombre']
        lazy = bool(sen.get('bajo_demanda'))
        if t in ('sensor.camera.rgb', 'sensor.camera.depth'):
            b.append(e(f'/gz/{AUTO}/{n}/image', f'/gz/{AUTO}/{n}/image', 'sensor_msgs/msg/Image',
                       'gz.msgs.Image', 'GZ_TO_ROS', lazy))
        elif t == 'sensor.camera.semantic_segmentation':
            for sub in ('colored_map', 'labels_map'):
                b.append(e(f'/gz/{AUTO}/{n}/{sub}', f'/gz/{AUTO}/{n}/{sub}', 'sensor_msgs/msg/Image',
                           'gz.msgs.Image', 'GZ_TO_ROS', lazy))
        elif t == 'sensor.lidar.ray_cast':
            b.append(e(f'/gz/{AUTO}/{n}/points', f'/gz/{AUTO}/{n}/points', 'sensor_msgs/msg/PointCloud2',
                       'gz.msgs.PointCloudPacked', 'GZ_TO_ROS'))
        elif t == 'sensor.other.imu':
            b.append(e(f'/gz/{AUTO}/{n}', f'/gz/{AUTO}/{n}', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS'))
        elif t == 'sensor.other.gnss':
            b.append(e(f'/gz/{AUTO}/{n}', f'/gz/{AUTO}/{n}', 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat',
                       'GZ_TO_ROS'))
        elif t == 'sensor.other.collision':
            b.append(e(f'/gz/{AUTO}/contacto', f'/gz/{AUTO}/contacto', 'ros_gz_interfaces/msg/Contacts',
                       'gz.msgs.Contacts', 'GZ_TO_ROS'))
    return b


def main():
    SALIDA.mkdir(parents=True, exist_ok=True)
    rs = rutas()
    sdf, npcs, ego_pose = mundo(rs)
    (SALIDA / 'ciudad.sdf').write_text(sdf, encoding='utf-8')
    (SALIDA / 'ciudad.json').write_text(json.dumps({
        'mundo': MUNDO, 'rutas': rs, 'npcs': npcs, 'ego': ego_pose,
        'obstaculos': obstaculos, 'marcas': marcas, 'ancho_calle': W, 'coords': COORDS}), encoding='utf-8')
    (SALIDA / 'mapa.json').write_text(json.dumps({
        'mapa': f'{MUNDO} (Gazebo)', 'carriles': carriles,
        'equipos': [{'nombre': AUTO, 'spawn': [ego_pose['x'], ego_pose['y']]}]}), encoding='utf-8')
    (SALIDA / 'puente.yaml').write_text(yaml.safe_dump(puente(npcs), sort_keys=False), encoding='utf-8')
    print(f'[mundo] {MUNDO}: {len(obstaculos)} obstáculos, {len(marcas)} marcas, {len(npcs)} autos de tráfico')


if __name__ == '__main__':
    main()
