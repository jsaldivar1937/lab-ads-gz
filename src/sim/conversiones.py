"""Conversiones entre CARLA (mano izquierda) y ROS 2 (mano derecha, REP-103).

CARLA:  x adelante, y derecha,   z arriba, ángulos en grados.
ROS:    x adelante, y izquierda, z arriba, ángulos en radianes.
"""
import array
import math

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2, PointField


def a_stamp(segundos: float) -> Time:
    ns = int(round(segundos * 1e9))
    t = Time()
    t.sec = ns // 1_000_000_000
    t.nanosec = ns % 1_000_000_000
    return t


def rpy_a_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def loc_carla_a_ros(loc):
    return loc.x, -loc.y, loc.z


def rot_carla_a_ros(rot):
    """Devuelve (roll, pitch, yaw) en radianes, convención ROS."""
    return math.radians(rot.roll), -math.radians(rot.pitch), -math.radians(rot.yaw)


def transform_ros_a_carla(pos, rot_deg):
    """pos=[x,y,z] (m) y rot=[roll,pitch,yaw] (grados) en ROS → carla.Transform."""
    import carla
    x, y, z = pos
    r, p, yw = rot_deg
    return carla.Transform(carla.Location(x=x, y=-y, z=z),
                           carla.Rotation(roll=r, pitch=-p, yaw=-yw))


def vel_mundo_a_cuerpo(vx, vy, yaw):
    """Velocidad del marco mundo (ROS) al marco del vehículo."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def nube_puntos(header, datos: np.ndarray, campos) -> PointCloud2:
    """Crea un PointCloud2 a partir de un arreglo float32 (N, k)."""
    datos = np.ascontiguousarray(datos, dtype=np.float32)
    k = len(campos)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = datos.shape[0]
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(campos)]
    msg.is_bigendian = False
    msg.point_step = 4 * k
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = a_bytes_ros(datos)
    return msg


def a_bytes_ros(arr: np.ndarray):
    """uint8[] rápido para mensajes ROS 2 (evita la validación elemento por elemento)."""
    return array.array('B', np.ascontiguousarray(arr).tobytes())
