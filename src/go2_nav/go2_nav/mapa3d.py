"""go2_mapa3d: mapa 3D DENSO y a color (por altura) de todo lo que vio el lidar.

  entrada: /go2/nube_sin_fantasmas (la nube del limpiador, sin el cuerpo ni los rebotes; marco odom)
  salida:  /go2/mapa_3d  PointCloud2 'map' (xyz + rgb), latched: RViz lo muestra al abrir
  servicio /go2/mapa3d (Comando): estado | borrar | guardar RUTA.pcd
  'go2 guardar_mapa casa' guarda tambien ~/.go2_nav/mapas/casa.pcd; con 'mapa:=casa' se carga y se
  sigue completando.

Voxeles de 5 cm; un voxel se dibuja si aparecio en >= 3 nubes (lo que pasa una vez no queda).
Solo usa puntos a menos de 'alcance' m del perro (lejos el L1 es ralo y ruidoso).
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from go2_nav_interfaces.srv import Comando

from .nube import leer_xyz
from .nucleo.mapa3d import Voxeles, colores, guardar_pcd, leer_pcd, rgb_float

MAPAS = os.path.expanduser("~/.go2_nav/mapas")


def matriz(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


def nube_color(xyz, rgb, stamp):
    m = PointCloud2()
    m.header.frame_id = "map"
    m.header.stamp = stamp
    m.height, m.width = 1, len(xyz)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(("x", "y", "z", "rgb"))]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 16, 16 * len(xyz), True
    d = np.empty((len(xyz), 4), np.float32)
    d[:, :3] = xyz
    d[:, 3] = rgb_float(rgb)
    m.data = d.tobytes()
    return m


class Mapa3D(Node):
    def __init__(self):
        super().__init__("go2_mapa3d")
        self.res = self.declare_parameter("resolucion", 0.05).value
        self.min_vistas = self.declare_parameter("min_vistas", 3).value
        self.alcance = self.declare_parameter("alcance", 8.0).value
        self.hz = self.declare_parameter("nubes_por_s", 5.0).value
        self.cada = self.declare_parameter("publicar_cada_s", 2.0).value
        mapa = self.declare_parameter("mapa", "").value.strip()
        self.vox = Voxeles(self.res)
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PointCloud2, "/go2/mapa_3d", latched)
        self.create_subscription(PointCloud2, "/go2/nube_sin_fantasmas", self.cb, qos_profile_sensor_data)
        self.create_service(Comando, "/go2/mapa3d", self.srv)
        self.t_ult = 0.0
        self.t_pub = 0.0
        self.nubes = 0
        self.sin_tf = 0
        if mapa:
            ruta = os.path.join(MAPAS, mapa + ".pcd")
            try:
                self.vox.cargar_puntos(leer_pcd(ruta))
                self.get_logger().info("mapa 3D '%s' cargado: %d voxeles" % (mapa, len(self.vox.k)))
            except (OSError, ValueError, KeyError) as e:
                self.get_logger().warn("no pude cargar %s (%s): empiezo vacio" % (ruta, e))
        self.create_timer(0.5, self.lazo)
        self.get_logger().info("go2_mapa3d listo: /go2/mapa_3d (voxel %.0f cm, >= %d vistas)"
                               % (self.res * 100, self.min_vistas))

    def cb(self, m):
        ahora = time.monotonic()
        if ahora - self.t_ult < 1.0 / max(self.hz, 0.1):
            return
        try:
            tf = self.tf.lookup_transform("map", m.header.frame_id or "odom", Time())
            tb = self.tf.lookup_transform("map", "base_link", Time())
        except Exception:
            self.sin_tf += 1
            return
        self.t_ult = ahora
        xyz = leer_xyz(m)
        if not len(xyz):
            return
        R, t = matriz(tf)
        p = xyz.astype(np.float64) @ R.T + t
        b = tb.transform.translation
        d2 = (p[:, 0] - b.x) ** 2 + (p[:, 1] - b.y) ** 2
        self.vox.agregar(p[d2 < self.alcance ** 2])
        self.nubes += 1

    def lazo(self):
        self.vox.fusionar()
        if self.vox.cambio and time.monotonic() - self.t_pub > self.cada:
            self.publicar()

    def publicar(self):
        xyz = self.vox.puntos(self.min_vistas)
        self.pub.publish(nube_color(xyz, colores(xyz[:, 2]), self.get_clock().now().to_msg()))
        self.vox.cambio = False
        self.t_pub = time.monotonic()

    def srv(self, req, res):
        o = req.orden
        if o == "borrar":
            self.vox.borrar()
            self.publicar()
            res.ok, res.mensaje = True, "mapa 3D borrado"
        elif o == "guardar":
            self.vox.fusionar()
            ruta = req.args[0] if req.args else os.path.join(MAPAS, "mapa3d.pcd")
            xyz = self.vox.puntos(self.min_vistas)
            if not len(xyz):
                res.ok, res.mensaje = False, "el mapa 3D esta vacio (llega /go2/nube_sin_fantasmas? hay TF map?)"
                return res
            os.makedirs(os.path.dirname(ruta), exist_ok=True)
            guardar_pcd(ruta, xyz)
            res.ok, res.mensaje = True, "mapa 3D: %d puntos -> %s" % (len(xyz), ruta)
        else:
            n = int((self.vox.c >= self.min_vistas).sum())
            res.ok = True
            res.mensaje = ("mapa 3D: %d voxeles dibujados (%d vistos), %d nubes usadas, %d sin TF"
                           % (n, len(self.vox.k), self.nubes, self.sin_tf))
            res.json = json.dumps({"voxeles": n, "total": len(self.vox.k), "nubes": self.nubes})
        return res


def main():
    rclpy.init()
    n = Mapa3D()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
