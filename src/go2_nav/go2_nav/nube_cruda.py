"""go2_nube: la nube CRUDA del L1 a 'odom', hecha en ROS 2 (no en el perro).

Por que: 'go2 escala' midio que la nube procesada por el perro (/utlidar/cloud_deskewed) NO queda
fija en 'odom': una pared se acerca en esa nube lo mismo que el perro avanza (la pared "camina" con
el perro). Todo lo que se arma con ella sale corrido: paredes que nunca aparecen, mapas dobles.

Aca se usa lo que sale del sensor tal cual:
  /utlidar/cloud      marco del L1 (utlidar_lidar), reloj del perro, campo 'time' por punto
  /go2/odom_perro     pose del perro (driver: fuente y escala de 'go2 escala'), MISMO reloj -> por punto
  extrinseca del L1 en el cuerpo (URDF de Unitree; la inclinacion la confirmo 'go2 diagnostico'):
    xyz = (0.28945, 0, -0.046825)  rpy = (0, 2.8782, 0)   -- boca abajo, ~15 grados al suelo
Cada punto: cuerpo = R_ext * p + t_ext ; odom = pose(t_punto) * cuerpo  (corrige el movimiento
durante el barrido). Se juntan las ultimas 'acumular' vueltas (ya en odom) -> /go2/nube_odom.

Salida: /go2/nube_odom (PointCloud2, odom, hora de la laptop) -> la usa el limpiador.
        /go2/nube_estado (JSON 1 Hz): Hz, puntos, % de puntos dentro del cuerpo (chequeo de extrinseca)
"""
import collections
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .nube import _DT, crear_nube


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_quats(q):
    """(n, 4) x y z w -> (n, 3, 3)."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def leer_campos(msg, nombres):
    n = msg.width * msg.height
    campos = {f.name: f for f in msg.fields}
    nombres = [k for k in nombres if k in campos]
    dt = np.dtype({"names": nombres, "formats": [_DT[campos[k].datatype] for k in nombres],
                   "offsets": [campos[k].offset for k in nombres], "itemsize": msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dt, count=n), nombres


class NubeCruda(Node):
    def __init__(self):
        super().__init__("go2_nube")
        P = self.declare_parameter
        self.topico = P("topico", "/utlidar/cloud").value
        xyz = list(P("extrinseca_xyz", [0.28945, 0.0, -0.046825]).value)
        rpy = list(P("extrinseca_rpy", [0.0, 2.8782, 0.0]).value)
        self.acumular = int(P("acumular", 3).value)
        self.r_min = P("rango_min", 0.05).value
        self.r_max = P("rango_max", 20.0).value
        self.R_ext = rot_rpy(*rpy)
        self.t_ext = np.array(xyz, np.float64)
        self.odom = collections.deque(maxlen=600)          # (t_robot, x, y, z, qx, qy, qz, qw)
        self.vueltas = collections.deque(maxlen=max(1, self.acumular))
        self.pub = self.create_publisher(PointCloud2, "/go2/nube_odom", qos_profile_sensor_data)
        self.pub_est = self.create_publisher(String, "/go2/nube_estado", 1)
        # la odometria que publica el driver (fuente y escala elegidas con 'go2 escala'), con la hora
        # del PERRO: la misma que la nube -> se interpola por punto
        self.create_subscription(Odometry, P("topico_odom", "/go2/odom_perro").value, self.cb_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.topico, self.cb_nube, qos_profile_sensor_data)
        self.est = {"n": 0, "puntos": 0, "en_cuerpo_pct": 0.0, "sin_odom": 0, "ms": 0.0, "sin_tiempo": False,
                    "relojes_distintos": False}
        self.create_timer(1.0, self.publicar_estado)
        self.get_logger().info("go2_nube: %s (crudo) -> /go2/nube_odom con la odometria del perro, %d vueltas"
                               % (self.topico, self.acumular))

    def cb_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if self.odom and t <= self.odom[-1][0]:
            return
        self.odom.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def cb_nube(self, m):
        t0 = time.perf_counter()
        if len(self.odom) < 2:
            self.est["sin_odom"] += 1
            return
        d, nombres = leer_campos(m, ["x", "y", "z", "time"])
        p = np.stack([d["x"], d["y"], d["z"]], axis=1).astype(np.float64)
        ok = np.isfinite(p).all(axis=1)
        r = np.linalg.norm(p, axis=1)
        ok &= (r > self.r_min) & (r < self.r_max)
        p = p[ok]
        if len(p) == 0:
            return
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if "time" in nombres:
            tp = ts + d["time"][ok].astype(np.float64)
        else:
            tp = np.full(len(p), ts)
            self.est["sin_tiempo"] = True
        O = np.array(self.odom)
        if tp.max() < O[0, 0] - 1.0 or tp.min() > O[-1, 0] + 1.0:
            # la nube y la odometria NO vienen con el mismo reloj: se usa la odometria mas nueva
            # (sin corregir el movimiento dentro del barrido) y se avisa
            tp = tp - ts + O[-1, 0] - 0.07
            self.est["relojes_distintos"] = True
        if tp.min() < O[0, 0] - 0.05:
            self.est["sin_odom"] += 1                        # nube mas vieja que la odometria guardada
            return
        tp = np.clip(tp, O[0, 0], O[-1, 0])                  # (a lo sumo unos ms despues de la ultima)
        pos = np.stack([np.interp(tp, O[:, 0], O[:, k]) for k in (1, 2, 3)], axis=1)
        q = np.stack([np.interp(tp, O[:, 0], O[:, k]) for k in (4, 5, 6, 7)], axis=1)
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        cuerpo = p @ self.R_ext.T + self.t_ext
        en_cuerpo = ((np.abs(cuerpo[:, 0]) < 0.45) & (np.abs(cuerpo[:, 1]) < 0.25)
                     & (cuerpo[:, 2] > -0.45) & (cuerpo[:, 2] < 0.15))
        R = rot_quats(q)
        w = np.einsum("nij,nj->ni", R, cuerpo) + pos
        self.vueltas.append(w.astype(np.float32))
        todo = np.concatenate(list(self.vueltas))
        self.pub.publish(crear_nube(todo, "odom", self.get_clock().now().to_msg()))
        self.est["n"] += 1
        self.est["puntos"] = len(todo)
        self.est["en_cuerpo_pct"] = round(100.0 * en_cuerpo.mean(), 1)
        self.est["ms"] = round((time.perf_counter() - t0) * 1000, 1)

    def publicar_estado(self):
        self.pub_est.publish(String(data=json.dumps(self.est)))
        self.est["n"] = 0
        self.est["sin_odom"] = 0


def main():
    rclpy.init()
    n = NubeCruda()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
