"""go2_slam3d: SLAM 3D del Go2 (reemplaza a slam_toolbox; ver nucleo/slam3d.py).

  entradas: /go2/nube_sin_fantasmas (nube del L1 sin el cuerpo, marco odom)  -> alinear con el mapa 3D
            /go2/obstaculos + /go2/suelo (del limpiador, marco odom)         -> mapa 2D
            /go2/pausar_slam (escalera: el mapa 2D no se toca), /initialpose ("2D Pose Estimate")
  salidas:  TF map->odom (20 Hz), /map (OccupancyGrid, latched, 1 por piso), /go2/mapa_3d (color = altura),
            /go2/slam_estado (JSON 1 Hz)
  servicio: /go2/mapa3d (Comando): estado | borrar | guardar RUTA.pcd
            guardar escribe RUTA.pcd (3D), RUTA.slam3d.npz (para navegar despues) y RUTA.pgm/.yaml (2D)
  mapa:=NOMBRE  carga ~/.go2_nav/mapas/NOMBRE.slam3d.npz: se ubica en el, no lo cambia.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from go2_nav_interfaces.srv import Comando

from .mapa3d import nube_color
from .nube import leer_xyz, quat_de_yaw, yaw_de_quat
from .nucleo.mapa3d import colores, guardar_pcd
from .nucleo.slam3d import (Registro, Rejilla, aplicar, ang, componer, guardar_pgm, inversa, limite_correccion,
                            submuestrear)

MAPAS = os.path.expanduser("~/.go2_nav/mapas")


def tf_a_T(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return (t.x, t.y, t.z, yaw_de_quat(q))


class Slam3D(Node):
    def __init__(self):
        super().__init__("go2_slam3d")
        P = self.declare_parameter
        self.alcance = P("alcance", 8.0).value              # m: lo que entra al mapa
        self.alcance_match = P("alcance_match", 12.0).value
        self.hz = P("nubes_por_s", 6.0).value
        self.min_inliers = P("min_inliers", 80).value
        self.min_frac = P("min_frac", 0.35).value
        self.salto_max = P("salto_max", 0.5).value           # m que puede corregir de una nube a otra
        self.giro_max = math.radians(P("giro_max_grados", 10.0).value)
        self.dz_piso = P("dz_piso", 0.8).value
        self.min_vistas = P("min_vistas_3d", 2).value
        mapa = P("mapa", "").value.strip()

        self.reg = Registro()
        self.T = (0.0, 0.0, 0.0, 0.0)                         # map -> odom
        self.pisos = [None]                                    # z del cuerpo en cada piso (el 1o: al arrancar)
        self.rej = {0: Rejilla()}
        self.piso = 0
        self.arranque = 5                                     # nubes que entran sin alinear (mapa vacio)
        self.reubicar = 0                                     # nubes con busqueda ancha (mapa guardado / 2D Pose)
        self.pausa = False
        self.ult_ok = 0.0
        self.seguidas_mal = 0
        self.est = {"aceptadas": 0, "descartadas": 0, "rms": None, "inliers": 0, "ms": 0.0, "motivo": ""}
        self.base_ok = None                                   # odom del perro en la ultima nube aceptada
        self.t_nube = 0.0
        self.suelo = None
        self.cambio_2d = True
        self.t_2d = 0.0
        self.t_3d = 0.0
        self.fijo = False

        if mapa:
            self.cargar(os.path.join(MAPAS, mapa))

        self.tfb = Buffer(cache_time=Duration(seconds=10.0))
        TransformListener(self.tfb, self)
        self.tfo = TransformBroadcaster(self)
        lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", lat)
        self.pub_3d = self.create_publisher(PointCloud2, "/go2/mapa_3d", lat)
        self.pub_est = self.create_publisher(String, "/go2/slam_estado", 1)
        sd = qos_profile_sensor_data
        self.create_subscription(PointCloud2, "/go2/nube_sin_fantasmas", self.cb_nube, sd)
        self.create_subscription(PointCloud2, "/go2/obstaculos", self.cb_obst, sd)
        self.create_subscription(PointCloud2, "/go2/suelo", lambda m: setattr(self, "suelo", m), sd)
        self.create_subscription(Bool, "/go2/pausar_slam", lambda m: setattr(self, "pausa", m.data), 1)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_pose_inicial, 1)
        self.create_service(Comando, "/go2/mapa3d", self.srv)
        self.create_timer(0.05, self.publicar_tf)
        self.create_timer(0.5, self.lazo)
        self.create_timer(1.0, self.publicar_estado)
        self.get_logger().info("go2_slam3d listo (%s): odometria del perro + alineacion 3D contra el mapa"
                               % ("mapa guardado '%s', se ubica" % mapa if self.fijo else "mapeando"))

    # ---------------------------------------------------------------- utilidades
    def _tf(self, padre, hijo):
        try:
            return tf_a_T(self.tfb.lookup_transform(padre, hijo, Time()))
        except Exception:
            return None

    def publicar_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.T[:3]
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = quat_de_yaw(self.T[3])
        self.tfo.sendTransform(t)

    # ---------------------------------------------------------------- alineacion 3D
    def cb_nube(self, m):
        ahora = time.monotonic()
        if ahora - self.t_nube < 1.0 / max(self.hz, 0.5):
            return
        lid = self._tf("odom", "lidar")
        base = self._tf("odom", "base_link")
        if lid is None or base is None:
            return
        self.t_nube = ahora
        t0 = time.perf_counter()
        p = leer_xyz(m).astype(np.float64)
        if len(p) < 50:
            return
        d = np.hypot(p[:, 0] - lid[0], p[:, 1] - lid[1])
        T0 = self.T
        if self.arranque > 0 and not self.fijo:
            ok, info = True, {"inliers": len(p), "frac": 1.0, "rms": 0.0}
            self.arranque -= 1
        else:
            pos = aplicar(T0, np.array([[base[0], base[1], base[2]]]))[0]
            self.reg.rearmar(pos)
            ancho = self.reubicar > 0
            Tn, info = self.reg.alinear(submuestrear(p[d < self.alcance_match], 0.10), T0,
                                        d_max=1.0 if ancho else 0.5, iters=15 if ancho else 10)
            # cuanto mueve AL PERRO la nueva correccion, contra lo que la odometria puede derivar
            # desde la ultima nube aceptada (nucleo/slam3d.limite_correccion)
            b = np.array([[base[0], base[1], base[2]]])
            pa, pb = aplicar(T0, b)[0], aplicar(Tn, b)[0]
            salto = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            giro = abs(ang(Tn[3] - T0[3]))
            if ancho or self.base_ok is None:
                lxy, lgiro = self.salto_max * 3.0, self.giro_max * 3.0
            else:
                lxy, lgiro = limite_correccion(math.hypot(base[0] - self.base_ok[0], base[1] - self.base_ok[1]),
                                               ang(base[3] - self.base_ok[3]))
                lxy, lgiro = min(lxy, self.salto_max), min(lgiro, self.giro_max)
            ok = (info["inliers"] >= self.min_inliers and info["frac"] >= self.min_frac
                  and salto <= lxy and giro <= lgiro and abs(pb[2] - pa[2]) < 0.2)
            if not ok and not info.get("motivo"):
                if info["inliers"] < self.min_inliers or info["frac"] < self.min_frac:
                    info["motivo"] = "calzan %d puntos (%.0f %%)" % (info["inliers"], 100 * info["frac"])
                else:
                    info["motivo"] = ("pedia mover al perro %.2f m / %.1f grados (limite %.2f m / %.1f): "
                                      "descartada" % (salto, math.degrees(giro), lxy, math.degrees(lgiro)))
            if self.reubicar > 0:
                self.reubicar -= 1
            if ok:
                self.T = Tn
        if ok:
            self.ult_ok = ahora
            self.base_ok = base
            self.seguidas_mal = 0
            self.est["aceptadas"] += 1
            if not self.fijo:
                self.reg.insertar(aplicar(self.T, p[d < self.alcance]))
            self.ver_piso(base)
        else:
            self.seguidas_mal += 1
            self.est["descartadas"] += 1
            if self.seguidas_mal in (15, 60, 300):
                self.get_logger().warn(
                    "no calzo con el mapa hace %d nubes (%s): sigo con la odometria del perro"
                    % (self.seguidas_mal, info.get("motivo", "")))
        self.est.update(rms=round(info.get("rms", 0.0), 3), inliers=info.get("inliers", 0),
                        ms=round((time.perf_counter() - t0) * 1000, 1),
                        motivo="" if ok else info.get("motivo", ""), planos=info.get("planos", 0))

    def ver_piso(self, base):
        """Otro piso (despues de una escalera): otro mapa 2D. El 3D es uno solo para toda la casa."""
        if self.pausa:
            return
        z = aplicar(self.T, np.array([[base[0], base[1], base[2]]]))[0][2]
        if self.pisos[0] is None:
            self.pisos[0] = float(z)
            return
        if abs(z - self.pisos[self.piso]) < self.dz_piso:
            return
        k = next((i for i, zp in enumerate(self.pisos) if zp is not None and abs(zp - z) < self.dz_piso), None)
        if k is None:
            self.pisos.append(float(z))
            k = len(self.pisos) - 1
            self.rej[k] = Rejilla()
        self.get_logger().info("piso %d -> %d (z %.2f m)" % (self.piso, k, z))
        self.piso = k
        self.cambio_2d = True

    def cb_pose_inicial(self, m):
        """'2D Pose Estimate' en RViz: el perro esta AQUI en el mapa."""
        base = self._tf("odom", "base_link")
        if base is None:
            return
        q = m.pose.pose
        dest = (q.position.x, q.position.y, aplicar(self.T, np.array([list(base[:3])]))[0][2],
                yaw_de_quat(q.orientation))
        # T * base = dest  ->  T = dest * base^-1
        T = componer(dest, inversa(base))
        self.T = (T[0], T[1], self.T[2], T[3])
        self.reubicar = 10
        self.get_logger().info("pose inicial: x=%.2f y=%.2f giro=%.0f grados (busco 10 nubes alrededor)"
                               % (dest[0], dest[1], math.degrees(dest[3])))

    # ---------------------------------------------------------------- mapa 2D
    def cb_obst(self, m):
        if self.fijo or self.pausa or time.monotonic() - self.ult_ok > 1.0:
            return
        if self.arranque > 0:
            return
        lid = self._tf("odom", "lidar")
        if lid is None:
            return
        o = leer_xyz(m).astype(np.float64)
        s = leer_xyz(self.suelo).astype(np.float64) if self.suelo is not None else np.zeros((0, 3))
        sen = aplicar(self.T, np.array([list(lid[:3])]))[0]
        o = aplicar(self.T, o) if len(o) else o
        s = aplicar(self.T, s) if len(s) else s
        self.rej[self.piso].actualizar(sen[:2], o[:, :2] if len(o) else np.zeros((0, 2)),
                                       s[:, :2] if len(s) else None, alcance=self.alcance)
        self.cambio_2d = True

    def publicar_map(self):
        occ, x0, y0 = self.rej[self.piso].ocupacion()
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = float(self.rej[self.piso].res)
        g.info.width, g.info.height = int(occ.shape[1]), int(occ.shape[0])
        g.info.origin.position.x, g.info.origin.position.y = float(x0), float(y0)
        g.info.origin.orientation.w = 1.0
        g.data = occ.ravel().tolist()
        self.pub_map.publish(g)
        self.cambio_2d = False
        self.t_2d = time.monotonic()

    def publicar_3d(self):
        self.reg.vox.fusionar()
        xyz = self.reg.vox.puntos(1 if self.fijo else self.min_vistas)
        self.pub_3d.publish(nube_color(xyz, colores(xyz[:, 2]), self.get_clock().now().to_msg()))
        self.reg.vox.cambio = False
        self.t_3d = time.monotonic()

    def lazo(self):
        self.reg.vox.fusionar()
        ahora = time.monotonic()
        if self.cambio_2d and ahora - self.t_2d > 1.0:
            self.publicar_map()
        if self.reg.vox.cambio and ahora - self.t_3d > 2.0:
            self.publicar_3d()

    def publicar_estado(self):
        e = dict(self.est)
        e.update(piso=self.piso, pisos=len(self.pisos), voxeles=int(len(self.reg.vox.k)),
                 mapa_match=self.reg.puntos_match(), fijo=self.fijo,
                 sin_calzar_s=round(time.monotonic() - self.ult_ok, 1) if self.ult_ok else None,
                 correccion=[round(v, 3) for v in self.T])
        self.pub_est.publish(String(data=json.dumps(e)))
        self.est["aceptadas"] = self.est["descartadas"] = 0

    # ---------------------------------------------------------------- guardar / cargar
    def guardar(self, ruta):
        base = ruta[:-4] if ruta.endswith(".pcd") else ruta
        os.makedirs(os.path.dirname(base) or ".", exist_ok=True)
        self.reg.vox.fusionar()
        xyz = self.reg.vox.puntos(self.min_vistas)
        if not len(xyz):
            return False, "el mapa esta vacio (llega /go2/nube_sin_fantasmas? el limpiador corre?)"
        guardar_pcd(base + ".pcd", xyz)
        rej = {}
        for k, r in self.rej.items():
            e = r.estado()
            rej.update({"L_%d" % k: e["L"], "vista_%d" % k: e["vista"],
                        "o_%d" % k: np.array([e["x0"], e["y0"], e["res"]])})
        np.savez_compressed(base + ".slam3d.npz", k=self.reg.vox.k, c=self.reg.vox.c, res=self.reg.vox.res,
                            pisos=np.array([0.0 if v is None else v for v in self.pisos]), **rej)
        occ, x0, y0 = self.rej[0].ocupacion()
        guardar_pgm(base, occ, x0, y0, self.rej[0].res)
        return True, "mapa 3D (%d puntos) + 2D guardados: %s.pcd / .pgm / .slam3d.npz" % (len(xyz), base)

    def cargar(self, base):
        ruta = base + ".slam3d.npz"
        try:
            z = np.load(ruta)
            self.reg.vox.res = float(z["res"])
            self.reg.vox.k, self.reg.vox.c = z["k"].astype(np.int64), z["c"].astype(np.int32)
            self.reg.vox.cambio = True
            self.pisos = [float(v) for v in z["pisos"]]
            self.rej = {}
            for k in range(len(self.pisos)):
                if "L_%d" % k in z:
                    o = z["o_%d" % k]
                    self.rej[k] = Rejilla.desde(z["L_%d" % k], z["vista_%d" % k], o[0], o[1], o[2])
                else:
                    self.rej[k] = Rejilla()
            self.reg.fijo = self.fijo = True
            self.reubicar = 10
            self.get_logger().info("mapa '%s': %d voxeles, %d piso(s). Arranca donde EMPEZASTE a mapear "
                                   "o marca donde esta con '2D Pose Estimate'" % (base, len(self.reg.vox.k), len(self.pisos)))
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().error("no pude cargar %s (%s): MAPEO DE CERO" % (ruta, e))

    def srv(self, req, res):
        o = req.orden
        if o == "guardar":
            res.ok, res.mensaje = self.guardar(req.args[0] if req.args else os.path.join(MAPAS, "mapa3d.pcd"))
        elif o == "borrar":
            if self.fijo:
                res.ok, res.mensaje = False, "es un mapa guardado: no lo borro (lanza sin mapa:= para mapear)"
                return res
            self.reg = Registro()
            self.rej = {k: Rejilla() for k in self.rej}
            self.arranque = 5
            self.cambio_2d = True
            self.publicar_3d()
            res.ok, res.mensaje = True, "mapa 2D y 3D borrados: empiezo de cero desde aqui"
        else:
            e = dict(self.est)
            res.ok = True
            res.mensaje = ("SLAM 3D: %d voxeles, piso %d de %d, correccion x=%.2f y=%.2f giro=%.1f grados, "
                           "ultima alineacion buena %s (inliers %s, planos %s, rms %s m, %s ms)%s"
                           % (len(self.reg.vox.k), self.piso, len(self.pisos), self.T[0], self.T[1],
                              math.degrees(self.T[3]),
                              "hace %.1f s" % (time.monotonic() - self.ult_ok) if self.ult_ok else "nunca",
                              e["inliers"], e.get("planos"), e["rms"], e["ms"],
                              ("\n  ultima descartada: " + e["motivo"]) if e.get("motivo") else ""))
            res.json = json.dumps(e)
        return res


def main():
    rclpy.init()
    n = Slam3D()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
