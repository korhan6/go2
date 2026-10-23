"""SLAM 3D del Go2 (logica pura, numpy + scipy): la odometria del perro + correccion contra un mapa 3D.

Por que: el L1 da ~1000 puntos utiles por vuelta. slam_toolbox (2D) solo ve un "scan" ralo armado
de esa nube: empareja mal, el mapa sale con paredes dobles y rayos sueltos. Aca se usa TODA la nube
3D (suelo, paredes, muebles, techo) contra un mapa 3D que se va armando, como FAST-LIO/GLIM:

  1. la odometria del perro (/utlidar/robot_odom, lidar-inercial de Unitree) da el movimiento
     corto -- es buena en metros, deriva en decenas de metros y al girar;
  2. cada nube (ya en 'odom', sin el cuerpo) se alinea con el mapa 3D: punto-a-plano (5 vecinos),
     4 grados de libertad (x, y, z, giro; inclinacion y balanceo los da bien la IMU), pesos de
     Huber (lo que no calza -- una persona, una puerta abierta -- casi no cuenta);
  3. la correccion resultante es el TF map->odom. Si la alineacion es mala (pocos puntos que calzan,
     salto grande) se DESCARTA: se sigue con la odometria y esa nube no entra al mapa (no lo ensucia);
  4. el mapa 2D para Nav2 (ocupacion, log-odds) se arma de los obstaculos/suelo ya alineados:
     cada rayo del lidar al obstaculo libera las celdas del camino; pared = vista muchas veces.
     Sin lecturas "a maximo alcance" -> sin los rayos sueltos de slam_toolbox. Uno por piso.

Volver a pasar por un lugar ya mapeado lo re-alinea con lo que ya estaba: la deriva NO se acumula
(en una casa no hace falta cierre de lazo).
"""
import math

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:                      # pragma: no cover
    cKDTree = None

from .mapa3d import Voxeles, centros


# ---------------------------------------------------------------- transformaciones (x, y, z, yaw)
def aplicar(T, p):
    """T = (x, y, z, yaw) aplicado a Nx3."""
    c, s = math.cos(T[3]), math.sin(T[3])
    q = np.empty_like(p, dtype=np.float64)
    q[:, 0] = c * p[:, 0] - s * p[:, 1] + T[0]
    q[:, 1] = s * p[:, 0] + c * p[:, 1] + T[1]
    q[:, 2] = p[:, 2] + T[2]
    return q


def componer(A, B):
    """A * B (primero B, despues A)."""
    c, s = math.cos(A[3]), math.sin(A[3])
    return (A[0] + c * B[0] - s * B[1], A[1] + s * B[0] + c * B[1], A[2] + B[2], ang(A[3] + B[3]))


def inversa(T):
    c, s = math.cos(T[3]), math.sin(T[3])
    return (-(c * T[0] + s * T[1]), -(-s * T[0] + c * T[1]), -T[2], ang(-T[3]))


def ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def submuestrear(p, res):
    """Un punto por voxel (el primero): reparte el peso de la nube en el espacio."""
    if len(p) == 0:
        return p
    k = np.floor(p / res).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return p[np.sort(i)]


# ---------------------------------------------------------------- alineacion
def limite_correccion(dist, giro, rel=0.06):
    """Cuanto puede cambiar la correccion desde la ultima nube aceptada, segun lo que el perro se
    movio desde entonces: la odometria del perro no deriva mas de ~6 % de lo recorrido/girado.
    Una alineacion que pide mas NO es deriva: es el mapa confundiendo la nube consigo misma
    (pasa con lidares ralos: la nube 'se pega' a su propio patron y arrastra al perro hacia atras).
    -> (m, rad)"""
    return 0.02 + rel * dist, math.radians(0.5) + rel * abs(giro) + math.radians(1.0) * dist


class Registro(object):
    """Mapa 3D (voxeles) + alineacion de cada nube contra el."""

    def __init__(self, res_mapa=0.05, res_match=0.10, radio_local=14.0, min_vistas_match=1):
        self.vox = Voxeles(res_mapa)
        self.res_match = res_match
        self.radio = radio_local
        self.min_vistas = min_vistas_match
        self.arbol = None
        self.pts = None
        self.centro_arbol = None
        self.arbol_n = 0
        self.fijo = False                 # mapa guardado: se usa, no se cambia

    # -- mapa local para emparejar (se rearma al moverse 1 m o cuando el mapa crecio)
    def puntos_match(self):
        return len(self.pts) if self.pts is not None else 0

    def rearmar(self, centro, forzar=False):
        if cKDTree is None:
            raise RuntimeError("falta scipy (sudo apt install python3-scipy)")
        if not forzar and self.centro_arbol is not None:
            lejos = math.hypot(centro[0] - self.centro_arbol[0], centro[1] - self.centro_arbol[1]) > 1.0
            crecio = len(self.vox.k) > 1.15 * self.arbol_n + 500
            if not (lejos or crecio):
                return False
        self.vox.fusionar()
        k = self.vox.k[self.vox.c >= (1 if self.fijo else self.min_vistas)]
        p = centros(k, self.vox.res).astype(np.float64)
        d2 = (p[:, 0] - centro[0]) ** 2 + (p[:, 1] - centro[1]) ** 2
        p = p[d2 < self.radio ** 2]
        p = submuestrear(p, self.res_match)
        self.pts = p
        self.arbol = cKDTree(p) if len(p) >= 20 else None
        self.centro_arbol = (centro[0], centro[1])
        self.arbol_n = len(self.vox.k)
        return True

    def _planos(self, p, radio, K=24):
        """Plano local del mapa alrededor de cada punto: vecinos en una ESFERA de 'radio' (no los k
        mas cercanos: con el L1, que barre en anillos, los k mas cercanos caen en la misma linea
        y no definen plano). -> (validos, centro, normal, distancia al mas cercano)."""
        K = min(K, len(self.pts))
        d, idx = self.arbol.query(p, k=K, distance_upper_bound=radio)
        vec = np.isfinite(d)
        nv = vec.sum(axis=1)
        idx = np.where(vec, idx, idx[:, :1] * 0)
        Q = self.pts[np.minimum(idx, len(self.pts) - 1)]
        m = vec[:, :, None]
        nvs = np.maximum(nv, 1)[:, None]
        c = (Q * m).sum(axis=1) / nvs
        A = (Q - c[:, None, :]) * m
        w_, v_ = np.linalg.eigh(np.einsum("nki,nkj->nij", A, A) / nvs[:, :, None])
        ok = ((nv >= 6) & (w_[:, 1] > 0.05 ** 2) & (w_[:, 0] < 0.1 * w_[:, 1])
              & (w_[:, 0] < 0.035 ** 2))
        return ok, c, v_[:, :, 0], d[:, 0]

    def alinear(self, scan, T0, d_max=0.5, iters=12, huber=0.05, prior_xy=2500.0, prior_yaw=40000.0):
        """scan: Nx3 en 'odom' (submuestreado). T0: correccion inicial (map->odom).
        Devuelve (T, info). info: inliers, frac, rms, planos, motivo (si no se pudo).

        Punto-a-plano con planos del mapa de 30-50 cm (juntan varios anillos del L1). La
        correccion de una nube a la siguiente casi no cambia: el 'prior' (2 cm, 0.3 grados) la
        frena en las direcciones que la nube no ve (un pasillo, solo el suelo) en vez de dejarla
        girar sola."""
        info = {"inliers": 0, "validos": 0, "frac": 0.0, "rms": 9.9, "iters": 0, "planos": 0, "motivo": ""}
        if self.arbol is None:
            info["motivo"] = "mapa local vacio"
            return T0, info
        if len(scan) < 30:
            info["motivo"] = "nube con %d puntos" % len(scan)
            return T0, info
        T = np.array(T0, dtype=np.float64)
        T0a = np.array(T0, dtype=np.float64)
        Hp = np.diag([prior_xy, prior_xy, prior_xy, prior_yaw])
        dm = d_max
        for it in range(iters):
            p = aplicar(T, scan)
            radio = 0.5 if it < 3 else 0.35
            ok, c, nrm, d0 = self._planos(p, radio)
            ok &= d0 < dm
            info["planos"] = int(ok.sum())
            if ok.sum() < 30:
                info["motivo"] = "solo %d planos del mapa cerca de la nube" % ok.sum()
                return tuple(T0), info
            p, c, nrm = p[ok], c[ok], nrm[ok]
            r = np.einsum("ni,ni->n", p - c, nrm)
            a = np.abs(r)
            w = np.where(a < huber, 1.0, huber / np.maximum(a, 1e-9))
            w[a > 0.3] = 0.0
            J = np.empty((len(p), 4))
            J[:, :3] = nrm
            J[:, 3] = nrm[:, 0] * -(p[:, 1] - T[1]) + nrm[:, 1] * (p[:, 0] - T[0])
            Jw = J * w[:, None]
            H = (J.T @ Jw) / (0.03 ** 2)
            g = (Jw.T @ r) / (0.03 ** 2)
            e = T - T0a
            e[3] = ang(e[3])
            dx = np.linalg.solve(H + Hp, -(g + Hp @ e))
            T += dx
            T[3] = ang(T[3])
            info["iters"] = it + 1
            if it >= 2:
                dm = max(0.15, dm * 0.7)
            if abs(dx[0]) + abs(dx[1]) + abs(dx[2]) < 5e-4 and abs(dx[3]) < 1e-4:
                break
        p = aplicar(T, scan)
        ok, c, nrm, d0 = self._planos(p, 0.35)
        ok &= d0 < 0.3
        r = np.abs(np.einsum("ni,ni->n", p[ok] - c[ok], nrm[ok]))
        ins = r < 0.08
        info["inliers"] = int(ins.sum())
        info["validos"] = int(ok.sum())
        info["frac"] = float(ins.sum()) / max(1, int(ok.sum()))
        info["rms"] = float(np.sqrt(np.mean(r[ins] ** 2))) if ins.any() else 9.9
        if info["inliers"] < 30:
            info["motivo"] = "solo %d puntos calzan" % info["inliers"]
        return tuple(T), info

    def insertar(self, puntos_mapa):
        if not self.fijo:
            self.vox.agregar(puntos_mapa)


# ---------------------------------------------------------------- mapa 2D (ocupacion)
L_HIT, L_MISS, L_MISS_SUELO, L_MIN, L_MAX = 0.85, -0.40, -0.15, -2.0, 3.5
# ocupada = vista en >= 2 nubes (con 1 sola, un rebote suelto era "pared": el cuarto se llenaba de
# puntitos negros, Nav2 los inflaba y no encontraba ruta)
L_OCUP, L_LIBRE = 1.6, -0.35


class Rejilla(object):
    """Ocupacion 2D log-odds que crece sola. Cada celda: a lo sumo un 'visto' por nube."""

    def __init__(self, res=0.05, lado_inicial=20.0, centro=(0.0, 0.0)):
        self.res = res
        n = int(lado_inicial / res)
        self.L = np.zeros((n, n), np.float32)
        self.vista = np.zeros((n, n), bool)
        self.x0 = centro[0] - lado_inicial / 2
        self.y0 = centro[1] - lado_inicial / 2

    def _cubrir(self, xy):
        """Agranda la rejilla (con 5 m de margen) si algun punto cae fuera."""
        if len(xy) == 0:
            return
        r = self.res
        h, w = self.L.shape
        mx0, my0 = xy[:, 0].min() - 5.0, xy[:, 1].min() - 5.0
        mx1, my1 = xy[:, 0].max() + 5.0, xy[:, 1].max() + 5.0
        izq = max(0, int(math.ceil((self.x0 - mx0) / r))) if mx0 < self.x0 + 1.0 else 0
        abajo = max(0, int(math.ceil((self.y0 - my0) / r))) if my0 < self.y0 + 1.0 else 0
        der = max(0, int(math.ceil((mx1 - (self.x0 + w * r)) / r))) if mx1 > self.x0 + w * r - 1.0 else 0
        arriba = max(0, int(math.ceil((my1 - (self.y0 + h * r)) / r))) if my1 > self.y0 + h * r - 1.0 else 0
        if izq or abajo or der or arriba:
            self.L = np.pad(self.L, ((abajo, arriba), (izq, der)))
            self.vista = np.pad(self.vista, ((abajo, arriba), (izq, der)))
            self.x0 -= izq * r
            self.y0 -= abajo * r

    def _celdas(self, xy):
        i = np.floor((xy[:, 1] - self.y0) / self.res).astype(np.int64)
        j = np.floor((xy[:, 0] - self.x0) / self.res).astype(np.int64)
        h, w = self.L.shape
        ok = (i >= 0) & (i < h) & (j >= 0) & (j < w)
        return i[ok] * w + j[ok]

    def _rayos(self, s, fin, recorte):
        if len(fin) == 0:
            return np.zeros(0, np.int64)
        v = fin - s
        d = np.hypot(v[:, 0], v[:, 1])
        n = np.maximum(1, (d / (self.res * 0.9)).astype(np.int64))
        tot = int(n.sum())
        rep = np.repeat(np.arange(len(fin)), n)
        k = np.arange(tot) - np.repeat(np.cumsum(n) - n, n)
        t = k / np.repeat(n, n)
        dd = np.repeat(d, n)
        ok = t * dd <= dd - recorte
        return self._celdas(s + v[rep[ok]] * t[ok, None])

    def actualizar(self, sensor_xy, golpes_xy, libres_xy=None, alcance=8.0):
        """golpes: obstaculos (fin de rayo). libres: suelo visto (libre sin rayo)."""
        s = np.asarray(sensor_xy, np.float64)
        g = np.asarray(golpes_xy, np.float64).reshape(-1, 2)
        if len(g):
            dg = np.hypot(g[:, 0] - s[0], g[:, 1] - s[1])
            g = g[(dg < alcance) & (dg > 0.05)]
        lib = np.zeros((0, 2)) if libres_xy is None else np.asarray(libres_xy, np.float64).reshape(-1, 2)
        if len(lib):
            dl = np.hypot(lib[:, 0] - s[0], lib[:, 1] - s[1])
            lib = lib[dl < alcance]
        self._cubrir(np.vstack([g, lib, s[None, :]]))
        Lf = self.L.reshape(-1)
        Vf = self.vista.reshape(-1)
        # rayos: a los obstaculos (hasta una celda y media antes) y al SUELO visto (hasta el punto):
        # el suelo llena el abanico entre rayo y rayo (con el L1 ralo, solo con los obstaculos el
        # mapa salia como una estrella de rayos)
        # (los rayos al suelo pesan menos: pasan por ENCIMA de lo bajo -- una caja -- y no lo deben borrar)
        golpe = np.unique(self._celdas(g)) if len(g) else np.zeros(0, np.int64)
        li_o = np.setdiff1d(np.unique(self._rayos(s, g, self.res * 1.5)), golpe, assume_unique=True)
        li_s = np.setdiff1d(np.unique(self._rayos(s, lib, 0.0)), np.concatenate([golpe, li_o]))
        Lf[li_o] += L_MISS
        Lf[li_s] += L_MISS_SUELO
        Vf[li_o] = True
        Vf[li_s] = True
        if len(golpe):
            Lf[golpe] += L_HIT
            Vf[golpe] = True
        np.clip(Lf, L_MIN, L_MAX, out=Lf)

    def ocupacion(self, recortar=True):
        """-> (int8 HxW con -1/0/100, x0, y0). Recorta a lo visto (+1 m)."""
        d = np.full(self.L.shape, -1, np.int8)
        d[self.vista & (self.L <= L_LIBRE)] = 0
        d[self.vista & (self.L >= L_OCUP)] = 100
        x0, y0 = self.x0, self.y0
        if recortar and self.vista.any():
            ii, jj = np.nonzero(self.vista)
            m = int(1.0 / self.res)
            i0, i1 = max(0, ii.min() - m), min(d.shape[0], ii.max() + m + 1)
            j0, j1 = max(0, jj.min() - m), min(d.shape[1], jj.max() + m + 1)
            d = d[i0:i1, j0:j1]
            x0, y0 = x0 + j0 * self.res, y0 + i0 * self.res
        return d, x0, y0

    def estado(self):
        return {"L": self.L, "vista": self.vista, "x0": self.x0, "y0": self.y0, "res": self.res}

    @classmethod
    def desde(cls, L, vista, x0, y0, res):
        r = cls(res, 1.0)
        r.L, r.vista, r.x0, r.y0 = L.astype(np.float32), vista.astype(bool), float(x0), float(y0)
        return r


def guardar_pgm(ruta_base, ocup, x0, y0, res):
    """Mapa 2D como el de map_server: NOMBRE.pgm + NOMBRE.yaml (blanco libre, negro ocupado)."""
    img = np.full(ocup.shape, 205, np.uint8)
    img[ocup == 0] = 254
    img[ocup == 100] = 0
    img = img[::-1]                                    # pgm: fila 0 arriba
    with open(ruta_base + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    with open(ruta_base + ".yaml", "w") as f:
        f.write("image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
                % (ruta_base.split("/")[-1], res, x0, y0))
