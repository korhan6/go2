"""go2_limpiador: convierte la nube CRUDA del L1 en datos limpios para ROS 2 / Nav2.

Entrada: /utlidar/cloud_deskewed (marco odom del perro, ~15 Hz, ~10k puntos)
Salidas (todas con la hora de la laptop):
  /go2/obstaculos   PointCloud2 odom: obstaculos reales (+ puntos virtuales en bordes/caidas)
  /go2/suelo        PointCloud2 odom: suelo visto (para "limpiar" el mapa de costos)
  /go2/fantasmas    PointCloud2 odom: lo que se ELIMINO (cuerpo, error del L1, ruido) -> rojo en RViz
  /scan             LaserScan base_link: para slam_toolbox
  /go2/limpiador_estado  String JSON: Hz, ms, suelo, holguras

Lo que limpia (medido en el Go2 real del usuario):
  1. el propio cuerpo (caja del perro, patas, cabeza);
  2. el ERROR DE ALTURA DEL L1 DE CERCA: el suelo sale +12 cm a 15 cm del lidar, +7 cm a 35 cm,
     0 a ~0.7 m. Sin corregir, el suelo junto a la cabeza parece un obstaculo de 10 cm;
  3. puntos fantasma propios aprendidos con 'go2 calibrar' (columnas fijas al cuerpo);
  4. ruido suelto: un obstaculo tiene que aparecer en 2 de las ultimas 3 nubes;
  5. reflejos del piso brillante (salen DEBAJO del suelo): no son caidas salvo que formen una
     superficie plana sin suelo encima (escalon, hueco real).
"""
import collections
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from .nube import a_cuerpo, crear_nube, leer_xyz, voxel_unico, yaw_de_quat

LARGO2, ANCHO2 = 0.36, 0.17      # medio largo / medio ancho del cuerpo del Go2 (0.70 x 0.31 m)


# ---------------------------------------------------------------- mascara aprendida
class Mascara(object):
    """Columnas (4 cm, marco del cuerpo) donde el L1 ve 'algo' siempre, gire hacia donde gire."""
    CELDA, FX, FY = 0.04, 0.96, 0.76
    H_APRENDE, H_FILTRA, H_MAX = 0.10, 0.06, 1.0

    def __init__(self, m=None):
        self.forma = (int(round(2 * self.FX / self.CELDA)), int(round(2 * self.FY / self.CELDA)))
        self.m = m

    def indices(self, fx, fy, h, h0):
        i = np.floor((fx + self.FX) / self.CELDA).astype(np.int64)
        j = np.floor((fy + self.FY) / self.CELDA).astype(np.int64)
        ok = (i >= 0) & (i < self.forma[0]) & (j >= 0) & (j < self.forma[1]) & (h >= h0) & (h < self.H_MAX)
        return i, j, ok

    def quitar(self, fx, fy, h):
        """bool por punto: True = es propio (quitar)."""
        q = np.zeros(len(fx), bool)
        if self.m is None:
            return q
        i, j, ok = self.indices(fx, fy, h, self.H_FILTRA)
        q[ok] = self.m[i[ok], j[ok]]
        return q

    def guardar(self, ruta):
        os.makedirs(os.path.dirname(ruta), exist_ok=True)
        np.savez_compressed(ruta, m=self.m, celda=self.CELDA, fx=self.FX, fy=self.FY, v=3)

    @classmethod
    def cargar(cls, ruta):
        try:
            d = np.load(ruta)
            if int(d["v"]) != 3:
                return cls()
            return cls(d["m"].astype(bool))
        except Exception:
            return cls()


class Calibrador(object):
    UMBRAL, R_MAX, MAX_CELDAS = 0.15, 0.80, 110

    def __init__(self):
        self.mk = Mascara()
        self.cuenta = np.zeros(self.mk.forma, np.int32)
        self.por_sector = np.zeros((12,) + self.mk.forma, bool)
        self.sectores = np.zeros(12, np.int32)
        self.n = 0
        self.giro = 0.0
        self._yaw = None

    def sumar(self, fx, fy, h, yaw):
        i, j, ok = self.mk.indices(fx, fy, h, self.mk.H_APRENDE)
        hit = np.zeros(self.mk.forma, bool)
        hit[i[ok], j[ok]] = True
        self.cuenta += hit
        self.n += 1
        sec = int(((yaw % (2 * math.pi)) / (2 * math.pi)) * 12) % 12
        self.sectores[sec] += 1
        self.por_sector[sec] |= hit
        if self._yaw is not None:
            self.giro += abs((yaw - self._yaw + math.pi) % (2 * math.pi) - math.pi)
        self._yaw = yaw

    def listo(self):
        return self.n >= 40 and self.giro >= 2 * math.pi * 1.2 and bool(np.all(self.sectores >= 2))

    def base(self):
        nx, ny = self.mk.forma
        ci = (np.arange(nx) + 0.5) * self.mk.CELDA - self.mk.FX
        cj = (np.arange(ny) + 0.5) * self.mk.CELDA - self.mk.FY
        rho = np.hypot(*np.meshgrid(ci, cj, indexing="ij"))
        # un fantasma sale con CUALQUIER rumbo; un objeto real solo cuando el perro mira hacia el
        return ((self.cuenta >= max(5, self.UMBRAL * self.n)) & (rho <= self.R_MAX)
                & (self.por_sector.sum(axis=0) >= 9))

    def mascara(self):
        from scipy import ndimage
        return Mascara(ndimage.binary_dilation(self.base(), np.ones((3, 3), bool)))


class FantasmasEnLinea(object):
    """Aprende los fantasmas del propio perro SIEMPRE, sin girar a proposito.

    Un fantasma va pegado al cuerpo: sale en la MISMA celda (marco del perro) mire hacia donde
    mire. Un objeto real esta quieto en el mundo: en una celda del cuerpo solo aparece cuando el
    perro mira hacia un lado concreto. Por eso: celda vista en >= 7 de 12 rumbos distintos, con
    frecuencia, cerca del cuerpo = fantasma. Caminar recto junto a una pared NO la marca (un solo
    rumbo)."""
    SECT, DECAE, TASA, MIN_SECT, MIN_VIS = 12, 0.997, 0.30, 7, 6

    def __init__(self):
        self.mk = Mascara()
        nx, ny = self.mk.forma
        self.hits = np.zeros((self.SECT, nx, ny), np.float32)
        self.vis = np.zeros(self.SECT, np.float32)
        ci = (np.arange(nx) + 0.5) * self.mk.CELDA - self.mk.FX
        cj = (np.arange(ny) + 0.5) * self.mk.CELDA - self.mk.FY
        self.cerca = np.hypot(*np.meshgrid(ci, cj, indexing="ij")) <= 0.80
        self.m = np.zeros((nx, ny), bool)
        self.n = 0

    def sumar(self, fx, fy, h, yaw):
        i, j, ok = self.mk.indices(fx, fy, h, self.mk.H_APRENDE)
        hit = np.zeros(self.mk.forma, bool)
        hit[i[ok], j[ok]] = True
        s = int(((yaw % (2 * math.pi)) / (2 * math.pi)) * self.SECT) % self.SECT
        self.hits *= self.DECAE
        self.vis *= self.DECAE
        self.hits[s] += hit
        self.vis[s] += 1.0
        self.n += 1
        if self.n % 15 == 0:
            ok_s = self.vis >= self.MIN_VIS
            if ok_s.sum() < self.MIN_SECT:
                return
            tasa = self.hits[ok_s] / self.vis[ok_s][:, None, None]
            m = ((tasa >= self.TASA).sum(axis=0) >= self.MIN_SECT) & self.cerca
            from scipy import ndimage
            self.m = ndimage.binary_dilation(m, np.ones((3, 3), bool)) if m.any() else m

    def quitar(self, fx, fy, h):
        q = np.zeros(len(fx), bool)
        if not self.m.any():
            return q
        i, j, ok = self.mk.indices(fx, fy, h, self.mk.H_FILTRA)
        q[ok] = self.m[i[ok], j[ok]]
        return q


# ---------------------------------------------------------------- nodo
class Limpiador(Node):
    def __init__(self):
        super().__init__("go2_limpiador")
        p = self.declare_parameter
        self.topico = p("topico_nube", "/utlidar/cloud_deskewed").value
        self.lidar_fx = p("lidar_fx", 0.30).value
        sesgo = p("sesgo_cerca", [0.14, 0.19]).value
        self.sesgo_a, self.sesgo_b = float(sesgo[0]), float(sesgo[1])
        self.obst_min = p("obst_min", 0.10).value          # mas bajo se pisa (cables, umbrales)
        self.obst_max = p("obst_max", 0.55).value          # mas alto pasa por debajo
        self.caida_min = p("caida_min", 0.12).value
        self.altura_base = p("altura_base", 0.33).value     # base_link sobre el suelo (si no se ve)
        self.persistencia = p("persistencia", True).value
        self.rango_scan = p("rango_scan", 10.0).value
        self.ruta_mascara = os.path.expanduser(p("archivo_mascara", "~/.go2_nav/propio.npz").value)

        self.mascara = Mascara.cargar(self.ruta_mascara)
        self.en_linea = FantasmasEnLinea()
        # memoria de obstaculos REALES (vistos fuera de la zona ciega), marco odom
        self.mem_k = np.zeros(0, np.int64)
        self.mem_xyz = np.zeros((0, 3), np.float32)
        self.mem_t = np.zeros(0, np.float64)
        self.pose = None            # (x, y, z, yaw) del perro en odom
        self.suelo = None
        self.prev_keys = collections.deque(maxlen=2)
        self.hist_scan = collections.deque()
        self.calib = None
        self.t_calib = 0.0
        self.lock = threading.Lock()
        self.stats = {"n": 0, "ms": 0.0, "t0": time.monotonic()}
        self.ultimo = {}
        self.estado_calib = None
        self.giro_hist = collections.deque(maxlen=25)
        self.giro_peligro = False

        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, "/odom", self.cb_odom, sd)   # la del driver (1 sola copia desde el perro)
        self.create_subscription(PointCloud2, self.topico, self.cb_nube, sd)
        self.pub_obs = self.create_publisher(PointCloud2, "/go2/obstaculos", sd)
        self.pub_suelo = self.create_publisher(PointCloud2, "/go2/suelo", sd)
        self.pub_fant = self.create_publisher(PointCloud2, "/go2/fantasmas", sd)
        # la nube del L1 tal cual, pero SIN el cuerpo ni los fantasmas (para go2_escaleras)
        self.pub_sin = self.create_publisher(PointCloud2, "/go2/nube_sin_fantasmas", sd)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", sd)
        self.pub_est = self.create_publisher(String, "/go2/limpiador_estado", 1)
        self.pub_vivo = self.create_publisher(Header, "/go2/lidar_vivo", 1)   # latido: llego una nube
        self.pub_giro = self.create_publisher(Twist, "cmd_vel_prioridad", 10)
        self.create_service(Trigger, "/go2/calibrar", self.srv_calibrar)
        self.create_service(Trigger, "/go2/calibrar_parar", self.srv_calibrar_parar)
        # en la escalera el perro va inclinado: el SLAM no debe recibir esos scans
        self.pausa_slam = False
        self.create_subscription(Bool, "/go2/pausar_slam", lambda m: setattr(self, "pausa_slam", m.data), 1)
        self.escalera_activa = False
        self.create_subscription(String, "/go2/escalera_estado", self.cb_escalera, 1)
        self.create_timer(1.0, self.publicar_estado)
        self.create_timer(0.1, self.lazo_calibracion)
        self.get_logger().info("go2_limpiador listo (mascara propia: %s)" %
                               ("cargada" if self.mascara.m is not None else "sin calibrar"))

    # ------------------------------------------------------------ entradas
    def cb_escalera(self, m):
        try:
            e = json.loads(m.data)
            self.escalera_activa = bool(e.get("activa") or e.get("en_escalera") or e.get("detenido"))
        except ValueError:
            pass

    def cb_odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y, p.z, yaw_de_quat(m.pose.pose.orientation))

    def cb_nube(self, msg):
        if self.pose is None:
            return
        t0 = time.perf_counter()
        stamp = self.get_clock().now().to_msg()
        xyz = leer_xyz(msg)
        x, y, z, yaw = self.pose
        if msg.header.frame_id not in ("odom", "map", "world", ""):
            # nube en el marco del cuerpo: a odom
            c, s = math.cos(yaw), math.sin(yaw)
            xyz = np.c_[x + c * xyz[:, 0] - s * xyz[:, 1], y + s * xyz[:, 0] + c * xyz[:, 1],
                        z + xyz[:, 2]].astype(np.float32)
        if len(xyz) == 0:
            return
        self.procesar(xyz, x, y, z, yaw, stamp)
        self.pub_vivo.publish(Header(stamp=stamp, frame_id="odom"))
        dt = (time.perf_counter() - t0) * 1000
        self.stats["n"] += 1
        self.stats["ms"] = max(self.stats["ms"], dt)

    # ------------------------------------------------------------ el filtro
    def procesar(self, xyz, x, y, z, yaw, stamp):
        fx, fy = a_cuerpo(xyz, x, y, yaw)
        xyz0 = xyz
        # 1) error de altura del L1 de cerca
        if self.sesgo_a > 0:
            r = np.hypot(fx - self.lidar_fx, fy)
            xyz = xyz.copy()
            xyz[:, 2] -= np.maximum(0.0, self.sesgo_a - self.sesgo_b * r).astype(np.float32)
        # 2) suelo
        suelo = self.estimar_suelo(xyz, fx, fy, z)
        h = xyz[:, 2] - suelo
        # 3) cuerpo propio
        alto_cuerpo = z - suelo
        propio = ((np.abs(fx) < LARGO2 + 0.12) & (np.abs(fy) < ANCHO2 + 0.13)
                  & (h > 0.03) & (h < alto_cuerpo + 0.35))
        if self.calib is not None:
            self.calib.sumar(fx[~propio], fy[~propio], h[~propio], yaw)
            self.vigilar_giro(fx, fy, h, propio)
        if not self.pausa_slam:           # en la escalera (inclinado) no aprende
            self.en_linea.sumar(fx[~propio], fy[~propio], h[~propio], yaw)
        caja = propio.copy()
        propio |= self.mascara.quitar(fx, fy, h)
        propio |= self.en_linea.quitar(fx, fy, h)
        # tomando una escalera la mascara podria borrar el escalon de delante: solo la caja
        solo_caja = self.pausa_slam or self.escalera_activa
        self.pub_sin.publish(crear_nube(xyz0[~(caja if solo_caja else propio)], "odom", stamp))
        # 4) clases
        es_suelo = (np.abs(h) < 0.06) & ~propio
        cand = (h >= self.obst_min) & (h <= self.obst_max) & ~propio
        # 5) persistencia: un obstaculo real aparece en 2 de las ultimas 3 nubes
        V = 0.12
        k = np.floor(xyz[cand] / V).astype(np.int64)
        keys = (k[:, 0] << 40) ^ (k[:, 1] << 20) ^ k[:, 2]
        if self.persistencia and len(self.prev_keys) == 2:
            ok = np.isin(keys, self.prev_keys[0]) | np.isin(keys, self.prev_keys[1])
        else:
            ok = np.ones(len(keys), bool)
        self.prev_keys.append(np.unique(keys))
        idx = np.nonzero(cand)[0]
        obst = xyz[idx[ok]]
        ruido = xyz[idx[~ok]]
        # 6) caidas: celdas sin suelo con una SUPERFICIE plana por debajo (escalon, hueco)
        virtuales = self.caidas(xyz, fx, fy, h, suelo, x, y)
        # 7) memoria: lo real que entra a la zona ciega (pegado al cuerpo / mascara) se recuerda
        recordados = self.memoria(obst, suelo, x, y, yaw)
        if len(recordados):
            obst = np.concatenate([obst, recordados])
        # salidas
        fant = np.concatenate([xyz[propio], ruido]) if len(ruido) else xyz[propio]
        obs_out = np.concatenate([obst, virtuales]) if len(virtuales) else obst
        self.pub_obs.publish(crear_nube(obs_out, "odom", stamp))
        self.pub_suelo.publish(crear_nube(voxel_unico(xyz[es_suelo], 0.08), "odom", stamp))
        self.pub_fant.publish(crear_nube(voxel_unico(fant, 0.03), "odom", stamp))
        if not self.pausa_slam:
            self.publicar_scan(obst, x, y, yaw, stamp)
        self.ultimo = self.holguras(obst, virtuales, x, y, yaw)
        self.ultimo.update({"suelo_bajo_cuerpo": round(alto_cuerpo, 3), "puntos": int(len(xyz)),
                            "obstaculos": int(len(obst)), "fantasmas": int(len(fant)),
                            "caidas": int(len(virtuales) // 4)})

    def memoria(self, obst, suelo, x, y, yaw):
        """La zona donde salen los fantasmas (pegada al cuerpo) se filtra: ahi el perro no puede
        distinguir. Pero un objeto REAL se ve antes, desde lejos, fuera de esa zona; los fantasmas
        nunca salen fuera de ella. Asi que: todo obstaculo visto fuera de la zona ciega se recuerda
        (voxel de 8 cm en odom) y, si despues queda dentro de la zona ciega, se sigue publicando
        (hasta 20 s). Lo que queda a la vista y ya no se ve, se olvida en 1.5 s."""
        ahora = time.monotonic()
        V = 0.08
        if len(obst):
            # NUEVOS recuerdos solo de lo visto a >= 0.35 m del borde del cuerpo (donde nunca hay
            # fantasmas); lo de mas cerca solo refresca recuerdos que ya existian
            ox, oy = a_cuerpo(obst, x, y, yaw)
            lejos = np.maximum(np.abs(ox) - LARGO2, np.abs(oy) - ANCHO2) >= 0.35
            k = np.floor(obst / V).astype(np.int64)
            kk = (k[:, 0] << 42) ^ ((k[:, 1] & 0x1FFFFF) << 21) ^ (k[:, 2] & 0x1FFFFF)
            sirve = lejos | np.isin(kk, self.mem_k)
            obst, k = obst[sirve], k[sirve]
        if len(obst):
            keys = (k[:, 0] << 42) ^ ((k[:, 1] & 0x1FFFFF) << 21) ^ (k[:, 2] & 0x1FFFFF)
            keys, idx = np.unique(keys, return_index=True)
            nuevos_xyz = (k[idx] + 0.5).astype(np.float32) * V
        else:
            keys, nuevos_xyz = np.zeros(0, np.int64), np.zeros((0, 3), np.float32)
        viejo = ~np.isin(self.mem_k, keys)
        mk = np.concatenate([self.mem_k[viejo], keys])
        mx = np.concatenate([self.mem_xyz[viejo], nuevos_xyz])
        mt = np.concatenate([self.mem_t[viejo], np.full(len(keys), ahora)])
        if len(mk) == 0:
            self.mem_k, self.mem_xyz, self.mem_t = mk, mx, mt
            return np.zeros((0, 3), np.float32)
        fx, fy = a_cuerpo(mx, x, y, yaw)
        h = mx[:, 2] - suelo
        ciego = ((np.abs(fx) < LARGO2 + 0.12) & (np.abs(fy) < ANCHO2 + 0.13)) \
            | self.mascara.quitar(fx, fy, h) | self.en_linea.quitar(fx, fy, h)
        edad = ahora - mt
        cerca = np.hypot(fx, fy) < 2.5
        # algo "dentro" del cuerpo no puede ser real (el perro estaria parado encima): fuera
        dentro = (np.abs(fx) < LARGO2) & (np.abs(fy) < ANCHO2)
        guardar = cerca & ~dentro & ((ciego & (edad < 20.0)) | (~ciego & (edad < 1.5)))
        self.mem_k, self.mem_xyz, self.mem_t = mk[guardar], mx[guardar], mt[guardar]
        # solo lo que esta en la zona ciega y no se acaba de ver (lo visible ya va en la nube)
        out = guardar & ciego & (edad > 0.0)
        return mx[out]

    def estimar_suelo(self, xyz, fx, fy, z):
        prev = self.suelo if self.suelo is not None else z - self.altura_base
        anillo = ((np.abs(fx) < 1.6) & (np.abs(fy) < 1.2)
                  & ~((np.abs(fx) < LARGO2 + 0.15) & (np.abs(fy) < ANCHO2 + 0.15)))
        hz = xyz[anillo, 2]
        hz = hz[np.abs(hz - prev) < (0.15 if self.suelo is None else 0.08)]
        if len(hz) > 40:
            hz = np.sort(hz)
            # la ventana de 3 cm mas poblada = la superficie del suelo (no reflejos ni objetos)
            fin = np.searchsorted(hz, hz + 0.03)
            dens = fin - np.arange(len(hz))
            b = int(np.argmax(dens))
            nuevo = float(np.median(hz[b:fin[b]]))
            self.suelo = nuevo if self.suelo is None else 0.7 * self.suelo + 0.3 * nuevo
        elif self.suelo is None:
            self.suelo = prev
        # el suelo no puede quedar a menos de 20 cm ni a mas de 45 cm del cuerpo
        self.suelo = min(max(self.suelo, z - 0.45), z - 0.20)
        return self.suelo

    def caidas(self, xyz, fx, fy, h, suelo, x, y):
        R, C = 2.5, 0.10
        sel = (np.abs(fx) < R) & (np.abs(fy) < R)
        if not np.any(sel):
            return np.zeros((0, 3), np.float32)
        px, py, ph = xyz[sel, 0], xyz[sel, 1], h[sel]
        i = np.floor((px - x + R) / C).astype(np.int64)
        j = np.floor((py - y + R) / C).astype(np.int64)
        n = int(2 * R / C)
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        i, j, ph = i[ok], j[ok], ph[ok]
        cel = i * n + j
        hay_suelo = np.zeros(n * n, bool)
        hay_suelo[cel[np.abs(ph) < 0.06]] = True
        bajo = ph < -self.caida_min
        if not np.any(bajo):
            return np.zeros((0, 3), np.float32)
        cb, hb = cel[bajo], ph[bajo]
        cnt = np.bincount(cb, minlength=n * n)
        # superficie: poca dispersion de alturas en la celda
        s1 = np.bincount(cb, weights=hb, minlength=n * n)
        s2 = np.bincount(cb, weights=hb * hb, minlength=n * n)
        with np.errstate(invalid="ignore", divide="ignore"):
            var = s2 / np.maximum(cnt, 1) - (s1 / np.maximum(cnt, 1)) ** 2
        caida = (cnt >= 6) & ~hay_suelo & (var < 0.03 ** 2)
        if not np.any(caida):
            return np.zeros((0, 3), np.float32)
        ii, jj = np.divmod(np.nonzero(caida)[0], n)
        cx = x - R + (ii + 0.5) * C
        cy = y - R + (jj + 0.5) * C
        out = []
        for dx, dy in ((-0.025, -0.025), (0.025, -0.025), (-0.025, 0.025), (0.025, 0.025)):
            out.append(np.c_[cx + dx, cy + dy, np.full(len(cx), suelo + 0.25)])
        return np.concatenate(out).astype(np.float32)

    def publicar_scan(self, obst, x, y, yaw, stamp):
        ahora = time.monotonic()
        self.hist_scan.append((ahora, obst))
        while self.hist_scan and ahora - self.hist_scan[0][0] > 0.35:
            self.hist_scan.popleft()
        pts = np.concatenate([o for _, o in self.hist_scan]) if self.hist_scan else obst
        s = LaserScan()
        s.header.frame_id = "base_link"
        s.header.stamp = stamp
        N = 720
        s.angle_min, s.angle_max = -math.pi, math.pi - 2 * math.pi / N
        s.angle_increment = 2 * math.pi / N
        s.scan_time, s.time_increment = 0.066, 0.0
        s.range_min, s.range_max = 0.20, float(self.rango_scan)
        rangos = np.full(N, np.inf, np.float32)
        if len(pts):
            fx, fy = a_cuerpo(pts, x, y, yaw)
            r = np.hypot(fx, fy)
            a = np.floor((np.arctan2(fy, fx) + math.pi) / s.angle_increment).astype(np.int64) % N
            ok = (r > s.range_min) & (r < s.range_max)
            np.minimum.at(rangos, a[ok], r[ok].astype(np.float32))
        s.ranges = rangos.tolist()
        self.pub_scan.publish(s)

    def holguras(self, obst, virt, x, y, yaw):
        """Espacio libre medido desde el BORDE del cuerpo (para 'go2 ver')."""
        pts = np.concatenate([obst, virt]) if len(virt) else obst
        if len(pts) == 0:
            return {"adelante": None, "izq": None, "der": None, "atras": None}
        fx, fy = a_cuerpo(pts, x, y, yaw)

        def d(sel, v):
            return round(float(np.partition(v[sel], min(2, int(sel.sum()) - 1))[min(2, int(sel.sum()) - 1)]), 2) \
                if np.count_nonzero(sel) >= 3 else None
        lat = np.abs(fy) < ANCHO2 + 0.05
        lon = np.abs(fx) < LARGO2
        return {"adelante": d(lat & (fx > LARGO2), fx - LARGO2), "atras": d(lat & (fx < -LARGO2), -fx - LARGO2),
                "izq": d(lon & (fy > ANCHO2), fy - ANCHO2), "der": d(lon & (fy < -ANCHO2), -fy - ANCHO2)}

    # ------------------------------------------------------------ calibracion
    def vigilar_giro(self, fx, fy, h, propio):
        """Durante el giro de calibracion: hay algo REAL donde barren las esquinas (~0.40 m)?
        Un fantasma sale en la misma celda del cuerpo en TODOS los rumbos; un objeto real pasa por
        una celda solo cuando el perro mira hacia el. Despues de recorrer >= 4 rumbos (120 grados):
        celdas del barrido con puntos ahora pero vistas en <= 2 rumbos = objeto real -> parar."""
        c = self.calib
        if c is None or int((c.sectores > 0).sum()) < 4:
            return
        rho = np.hypot(fx, fy)
        sel = (rho < 0.46) & (h > 0.06) & (h < 0.60) & ~propio
        if np.count_nonzero(sel) < 3:
            return
        i, j, ok = c.mk.indices(fx[sel], fy[sel], h[sel], 0.0)
        n_sect = c.por_sector.sum(axis=0)
        nuevos = n_sect[i[ok], j[ok]] <= 2
        if int(nuevos.sum()) >= 4:
            self.giro_peligro = True

    def srv_calibrar(self, req, res):
        if self.calib is not None:
            res.success, res.message = False, "ya estoy calibrando"
            return res
        self.giro_hist = collections.deque(maxlen=25)
        self.giro_peligro = False
        self.giro_ref = None
        self.calib = Calibrador()
        self.t_calib = time.monotonic()
        res.success, res.message = True, ("calibrando: giro ~2 vueltas en el sitio (necesita espacio "
                                          "para girar). Mira 'go2 ver'")
        return res

    def srv_calibrar_parar(self, req, res):
        if self.calib is not None:
            self.calib = None
            self.pub_giro.publish(Twist())
            self.estado_calib = "cancelada"
        res.success, res.message = True, "ok"
        return res

    def lazo_calibracion(self):
        c = self.calib
        if c is None:
            return
        listo = c.listo()
        # si no gira (algo lo traba), no insistir
        ahora = time.monotonic()
        if self.giro_ref is None:
            self.giro_ref = (ahora, c.giro)
        elif ahora - self.giro_ref[0] > 3.0:
            if c.giro - self.giro_ref[1] < 0.25 and not listo:
                self.giro_peligro = True
            self.giro_ref = (ahora, c.giro)
        if not listo and self.giro_peligro:
            # algo se acerco mientras giraba: paro (los fantasmas se siguen aprendiendo en linea)
            self.pub_giro.publish(Twist())
            self.calib = None
            self.estado_calib = "interrumpida: algo cerca"
            self.get_logger().warn("calibracion interrumpida: algo se acerco al girar")
            return
        if not listo and time.monotonic() - self.t_calib < 120.0:
            t = Twist()
            t.angular.z = 0.6
            self.pub_giro.publish(t)
            self.estado_calib = "calibrando: %d nubes, %.0f de 430 grados" % (c.n, math.degrees(c.giro))
            return
        self.pub_giro.publish(Twist())
        self.calib = None
        if not listo:
            self.get_logger().warn("calibracion INCOMPLETA: %.0f grados en 120 s" % math.degrees(c.giro))
            self.estado_calib = "incompleta"
            return
        nb = int(c.base().sum())
        if nb > c.MAX_CELDAS:
            self.get_logger().warn("calibracion RECHAZADA: algo real muy cerca (%d celdas)" % nb)
            self.estado_calib = "rechazada: algo cerca"
            return
        self.mascara = c.mascara()
        self.mascara.guardar(self.ruta_mascara)
        self.get_logger().info("calibracion lista: %d celdas propias -> %s" % (nb, self.ruta_mascara))
        self.estado_calib = "lista (%d celdas)" % nb

    def publicar_estado(self):
        s = self.stats
        dt = time.monotonic() - s["t0"]
        e = dict(self.ultimo)
        e.update({"fantasmas_aprendidos": int(self.en_linea.m.sum()),
                  "nubes_hz": round(s["n"] / dt, 1) if dt > 0 else 0, "ms_max": round(s["ms"], 1),
                  "calibrado": self.mascara.m is not None, "calibracion": self.estado_calib})
        self.stats = {"n": 0, "ms": 0.0, "t0": time.monotonic()}
        self.pub_est.publish(String(data=json.dumps(e)))


def main():
    rclpy.init()
    n = Limpiador()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
