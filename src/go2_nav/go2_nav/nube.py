"""Utilidades de nubes y poses: PointCloud2 <-> numpy sin dependencias raras (rapido)."""
import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

_DT = {PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64, PointField.INT16: np.int16,
       PointField.UINT16: np.uint16, PointField.INT32: np.int32, PointField.UINT32: np.uint32,
       PointField.INT8: np.int8, PointField.UINT8: np.uint8}


def leer_xyz(msg):
    """PointCloud2 -> Nx3 float32 (descarta NaN)."""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), np.float32)
    campos = {f.name: f for f in msg.fields}
    if not all(k in campos for k in ("x", "y", "z")):
        return np.zeros((0, 3), np.float32)
    dt = np.dtype({"names": ["x", "y", "z"],
                   "formats": [_DT[campos[k].datatype] for k in ("x", "y", "z")],
                   "offsets": [campos[k].offset for k in ("x", "y", "z")],
                   "itemsize": msg.point_step})
    if msg.is_bigendian:
        dt = dt.newbyteorder(">")
    a = np.frombuffer(bytes(msg.data) if not isinstance(msg.data, (bytes, bytearray, memoryview)) else msg.data,
                      dtype=dt, count=n)
    xyz = np.empty((n, 3), np.float32)
    xyz[:, 0], xyz[:, 1], xyz[:, 2] = a["x"], a["y"], a["z"]
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok]


def crear_nube(xyz, frame, stamp):
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    m = PointCloud2()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.height = 1
    m.width = len(xyz)
    m.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * len(xyz)
    m.is_dense = True
    m.data = xyz.tobytes()
    return m


def yaw_de_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rpy_de_quat(q):
    """(yaw, pitch, roll) de un cuaternion ROS."""
    yaw = yaw_de_quat(q)
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    return yaw, pitch, roll


def quat_de_yaw(yaw):
    """(x, y, z, w) de un giro plano."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def a_cuerpo(xyz, x, y, yaw):
    """Puntos (odom) -> coordenadas del perro (fx adelante, fy izquierda)."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx = xyz[:, 0] - x
    dy = xyz[:, 1] - y
    return c * dx + s * dy, -s * dx + c * dy


def voxel_unico(xyz, vox):
    """Un punto por voxel (el primero). Para bajar el tamano de nubes de salida."""
    if len(xyz) == 0:
        return xyz
    k = np.floor(xyz / vox).astype(np.int64)
    k = (k[:, 0] << 42) ^ (k[:, 1] << 21) ^ k[:, 2]
    _, i = np.unique(k, return_index=True)
    return xyz[i]
