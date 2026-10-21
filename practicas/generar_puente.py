#!/usr/bin/env python3
"""Lista de tópicos que se copian (solo lectura) al dominio de prácticas."""
import os
import sys
from pathlib import Path

import yaml



def topicos_practicas():
    """Devuelve (dominio_destino, {tópico: (tipo, persistente)})."""
    cfg = yaml.safe_load(Path(os.environ.get('LAB_CONFIG', '/lab/config/lab.yaml')).read_text(encoding='utf-8'))
    AUTO = cfg['equipos'][0]['nombre']
    destino = int((cfg.get('practicas') or {}).get('dominio', 8))
    origen = int(os.environ.get('ROS_DOMAIN_ID_LAB', 7))

    topicos = {
        '/clock': 'rosgraph_msgs/msg/Clock',
        '/tf': 'tf2_msgs/msg/TFMessage',
        f'/{AUTO}/ground_truth/odom': 'nav_msgs/msg/Odometry',
        f'/{AUTO}/vehiculo/estado': 'std_msgs/msg/String',
        f'/{AUTO}/eventos': 'std_msgs/msg/String',
        f'/{AUTO}/encoders/joint_states': 'sensor_msgs/msg/JointState',
        f'/{AUTO}/encoders/ticks': 'std_msgs/msg/Int32MultiArray',
        f'/{AUTO}/can/rx': 'can_msgs/msg/Frame',
        f'/{AUTO}/ecu/estado': 'std_msgs/msg/String',
        '/lab/estado': 'std_msgs/msg/String',
    }
    for s in cfg['sensores']:
        if not s.get('activo', True):
            continue
        n, t = s['nombre'], s['tipo']
        if t.startswith('sensor.camera'):
            topicos[f'/{AUTO}/{n}/image'] = 'sensor_msgs/msg/Image'
            topicos[f'/{AUTO}/{n}/camera_info'] = 'sensor_msgs/msg/CameraInfo'
        elif t in ('sensor.lidar.ray_cast', 'sensor.other.radar'):
            topicos[f'/{AUTO}/{n}/points'] = 'sensor_msgs/msg/PointCloud2'
        elif t == 'sensor.other.imu':
            topicos[f'/{AUTO}/{n}/data'] = 'sensor_msgs/msg/Imu'
        elif t == 'sensor.other.gnss':
            topicos[f'/{AUTO}/{n}/fix'] = 'sensor_msgs/msg/NavSatFix'
        elif t == 'sensor.other.obstacle':
            for k in (s.get('arreglo') or {}):
                topicos[f'/{AUTO}/{n}_{k}/range'] = 'sensor_msgs/msg/Range'

    salida = {k: (v, False) for k, v in topicos.items()}
    salida['/tf_static'] = ('tf2_msgs/msg/TFMessage', True)
    return destino, salida


if __name__ == '__main__':
    destino, t = topicos_practicas()
    for k, (tipo, _) in t.items():
        print(f'{k:45s} {tipo}')
    print(f'{len(t)} tópicos → dominio {destino}')
