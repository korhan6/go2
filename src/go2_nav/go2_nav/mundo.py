"""Mundo simulado del Go2 (copiado de go2nav/sim.py) para probar go2_nav en ROS 2 sin el perro.

El mundo es un mapa de alturas (5 cm) + paredes. El "lidar" devuelve puntos del suelo y de
las paredes visibles (con oclusion 2.5D) en el marco odom, como la nube del Go2.
El perro simulado solo puede subir/bajar escalones con la marcha de escaleras puesta.
"""
import math
import time

import numpy as np



class Robot(object):
    """Base minima (el simulador no necesita mas)."""
    pass

RES = 0.05


class Mundo(object):
    def __init__(self, ancho, alto):
        self.W, self.H = int(ancho / RES), int(alto / RES)
        self.suelo = np.zeros((self.H, self.W), np.float32)    # altura del piso
        self.pared = np.zeros((self.H, self.W), bool)
        self.mesas = []
        self.borde()

    def mesa(self, x0, y0, x1, y1, z=0.72):
        """Tablero de una mesa (superficie colgada a z m, con aire debajo): el lidar ve su cara de
        abajo. El mapa de alturas no puede tener nada colgado, asi que va aparte."""
        self.mesas.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), z))

    def borde(self):
        self.pared[:2, :] = self.pared[-2:, :] = True
        self.pared[:, :2] = self.pared[:, -2:] = True

    def c(self, x, y):
        return int(y / RES), int(x / RES)

    def rect_pared(self, x0, y0, x1, y1):
        i0, j0 = self.c(x0, y0)
        i1, j1 = self.c(x1, y1)
        self.pared[min(i0, i1):max(i0, i1) + 1, min(j0, j1):max(j0, j1) + 1] = True

    def rect_libre(self, x0, y0, x1, y1):
        i0, j0 = self.c(x0, y0)
        i1, j1 = self.c(x1, y1)
        self.pared[min(i0, i1):max(i0, i1) + 1, min(j0, j1):max(j0, j1) + 1] = False

    def rect_altura(self, x0, y0, x1, y1, z):
        i0, j0 = self.c(x0, y0)
        i1, j1 = self.c(x1, y1)
        self.suelo[min(i0, i1):max(i0, i1), min(j0, j1):max(j0, j1)] = z

    def escalera_x(self, x0, y0, y1, n, huella, contra, z0=0.0):
        """Escalera que sube hacia +x desde x0."""
        for k in range(n):
            self.rect_altura(x0 + k * huella, y0, x0 + (k + 1) * huella, y1, z0 + (k + 1) * contra)

    def preparar(self):
        from scipy import ndimage
        self.zmin3 = ndimage.minimum_filter(self.suelo, size=3)
        self.zmax3 = ndimage.maximum_filter(self.suelo, size=3)

    def altura(self, x, y):
        i, j = self.c(x, y)
        i = min(max(i, 0), self.H - 1)
        j = min(max(j, 0), self.W - 1)
        return float(self.suelo[i, j])

    def es_pared(self, x, y):
        i, j = self.c(x, y)
        if not (0 <= i < self.H and 0 <= j < self.W):
            return True
        return bool(self.pared[i, j])


def casa_demo():
    """Dos cuartos + pasillo con puerta, una mesa, y una escalera a un entrepiso (1.53 m)."""
    m = Mundo(14.0, 10.0)
    # pared que divide (con puerta de 0.9 m)
    m.rect_pared(6.0, 0.0, 6.1, 10.0)
    m.rect_libre(6.0, 4.0, 6.1, 4.9)
    # mueble en el cuarto 1
    m.rect_pared(2.0, 6.5, 3.2, 7.3)
    # caja en el cuarto 2
    m.rect_pared(9.0, 2.0, 9.6, 2.5)
    # entrepiso a 1.53 m en x 11.2..14, y 5..9 ; escalera que sube hacia +x desde x=8.6
    m.escalera_x(8.68, 6.0, 7.2, 9, 0.28, 0.17)
    m.rect_altura(11.2, 5.0, 14.0, 9.0, 1.53)
    # baranda del entrepiso (no dejar caer por los lados)
    m.rect_pared(11.2, 5.0, 14.0, 5.05)
    m.rect_pared(11.2, 4.95, 11.25, 6.0)
    m.rect_pared(11.2, 7.2, 11.25, 9.0)
    # paredes a los costados de la escalera
    m.rect_pared(8.6, 5.9, 11.2, 5.95)
    m.rect_pared(8.6, 7.25, 11.2, 7.3)
    return m


def mundo_puerta():
    """Como la casa del usuario (foto del 19-oct): cuarto con el perro mirando una puerta de
    72 cm (la hoja abierta hacia adentro, pegada al marco izquierdo); detras, un pasillo; a la
    izquierda del pasillo, una escalera que BAJA 1.19 m (7 escalones de 17 cm); al fondo, un bano."""
    m = Mundo(10.0, 7.0)
    m.rect_pared(0.0, 4.0, 10.0, 4.1)            # pared del cuarto (arriba)
    m.rect_libre(4.6, 4.0, 5.32, 4.1)            # la puerta: 72 cm
    m.rect_pared(4.55, 3.2, 4.6, 4.0)            # la hoja de la puerta, abierta hacia el cuarto
    m.rect_pared(3.0, 0.0, 3.1, 4.0)             # pared izquierda del cuarto
    m.rect_pared(8.0, 0.0, 8.1, 4.0)             # pared derecha del cuarto
    m.rect_pared(3.2, 1.4, 4.0, 2.4)             # mesa
    # escalera que baja hacia -x desde el borde x = 4.2 (a la izquierda saliendo por la puerta)
    m.escalera_x(2.52, 4.1, 5.0, 6, 0.28, 0.17, z0=-1.19)
    m.rect_altura(0.1, 4.1, 2.52, 5.0, -1.19)    # piso de abajo
    m.rect_pared(0.0, 5.0, 4.2, 5.05)            # baranda / pared del lado de la escalera
    m.rect_pared(0.0, 5.05, 4.2, 7.0)
    m.rect_pared(4.2, 5.3, 10.0, 5.35)           # pared del fondo del pasillo
    m.rect_libre(5.6, 5.3, 6.3, 5.35)            # puerta del bano (70 cm)
    m.rect_pared(7.2, 5.35, 7.25, 7.0)           # el bano: 3 x 1.6 m
    return m, (5.0, 2.2, math.pi / 2)


def escalera_pasillo(pared_abajo=None):
    """Como la escalera de la casa del usuario (fotos del 19/20-oct): 9 escalones de 17 cm entre dos
    paredes (85 cm de ancho), con el pasillo de abajo igual de angosto. Sube hacia +x desde x = 3.0;
    arriba, un pasillo ancho a 1.53 m desde x = 5.43. pared_abajo = x de una pared detras del pie de la escalera
    (para ver que la bajada en reversa para a tiempo)."""
    m = Mundo(8.0, 3.0)
    arriba = 3.0 + 9 * 0.27
    m.rect_altura(arriba, 0.0, 8.0, 3.0, 1.53)                 # arriba: un pasillo ancho
    m.rect_altura(0.5, 0.0, arriba, 1.0, 1.53)                 # a los costados del hueco de la escalera,
    m.rect_altura(0.5, 1.85, arriba, 3.0, 1.53)                # el piso de arriba (paredes hasta el techo)
    m.escalera_x(3.0, 1.0, 1.85, 9, 0.27, 0.17)
    m.rect_pared(0.5, 0.95, arriba, 1.0)
    m.rect_pared(0.5, 1.85, arriba, 1.9)
    if pared_abajo is not None:
        m.rect_pared(pared_abajo - 0.05, 1.0, pared_abajo, 1.85)
    return m


def cuarto_simple():
    m = Mundo(8.0, 6.0)
    m.rect_pared(3.0, 2.0, 3.6, 2.6)
    return m


class RobotSim(Robot):
    def __init__(self, mundo, x=1.5, y=1.5, yaw=0.3, tiempo_real=False, deriva=0.0, odom_origen=None,
                 ruido_real=False, l1_real=False, piso_brillante=False):
        self.m = mundo
        mundo.preparar()
        self.x, self.y, self.yaw = x, y, yaw
        self.z = mundo.altura(x, y) + 0.30
        self.pitch = self.roll = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.v = [0.0, 0.0, 0.0]
        self.t = 0.0
        self.real = tiempo_real
        self.t_lidar = 0.0
        self.nubes = []
        self.escaleras = False
        self.choques = 0
        self.donde_choques = []
        self.deriva = deriva          # rad/m de deriva de rumbo en la odometria
        self.odo_yaw_err = 0.0
        # la odometria arranca en cero donde se prende el perro (como el Go2 real)
        self.org = odom_origen if odom_origen is not None else (0.0, 0.0, 0.0)
        self.rng = np.random.default_rng(1)
        self.t_ult_cmd = 0.0
        self._t0 = time.monotonic()
        self.historial = []
        self.personas = []           # (x0, y0, x1, y1, v): van y vienen
        self.ruido_real = ruido_real   # como el L1 real: ve sus patas/cabeza y "rebotes" en piso brillante
        # geometria del L1 de verdad (URDF del Go2): 4.7 cm DEBAJO del centro del cuerpo (a ~25 cm del
        # suelo), boca abajo e inclinado 15 grados: delante ve hasta 15 grados por encima del horizonte,
        # detras del hocico no ve (el cuerpo lo tapa) y una huella mas alta que el no la ve (solo su cara)
        self.l1_real = l1_real
        # piso de ceramica brillante (como la casa, foto del 20-oct): de lejos (rasante) el piso casi
        # no devuelve, y todo lo que hay arriba se ve ESPEJADO debajo del piso (mismo x, y)
        self.piso_brillante = piso_brillante
        self._marcadas = None

    # ---- tiempo
    def now(self):
        return (time.monotonic() - self._t0) if self.real else self.t

    def dormir(self, s):
        if self.real:
            time.sleep(s)
            self._fisica_hasta(self.now())
        else:
            self._fisica_hasta(self.t + s)

    def _fisica_hasta(self, t_fin):
        dt = 0.02
        t = self.t
        while t < t_fin - 1e-9:
            h = min(dt, t_fin - t)
            self._paso(h, t)
            t += h
            if t - self.t_lidar >= 0.1:
                self.t_lidar = t
                self.t = t
                self.nubes.append((t, self._lidar()))
                del self.nubes[:-6]
        self.t = t_fin

    def _mover_personas(self, t):
        if not self.personas:
            return
        if self._marcadas is not None:
            self.m.pared[self._marcadas] = False
        ii, jj = [], []
        for (x0, y0, x1, y1, v) in self.personas:
            L = math.hypot(x1 - x0, y1 - y0)
            f = (t * v / L) % 2.0
            f = f if f <= 1.0 else 2.0 - f
            px, py = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
            ci, cj = self.m.c(px, py)
            for a in range(-4, 5):
                for b in range(-4, 5):
                    if a * a + b * b <= 16 and not self.m.pared[ci + a, cj + b]:
                        ii.append(ci + a)
                        jj.append(cj + b)
        self._marcadas = (np.array(ii, int), np.array(jj, int))
        self.m.pared[self._marcadas] = True

    def _paso(self, h, t):
        self._mover_personas(t)
        if t - self.t_ult_cmd > 1.0:       # como el Go2: sin ordenes, se para
            self.cmd = (0.0, 0.0, 0.0)
        a = min(1.0, h / 0.15)
        for k in range(3):
            self.v[k] += a * (self.cmd[k] - self.v[k])
        vx, vy, wz = self.v
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        nx = self.x + (c * vx - s * vy) * h
        ny = self.y + (s * vx + c * vy) * h
        if self._libre(nx, ny, self.yaw + wz * h):
            self.x, self.y = nx, ny
            self.odo_yaw_err += self.deriva * math.hypot(vx, vy) * h
        elif abs(vx) + abs(vy) > 0.02:
            self.choques += 1
            if self.choques % 50 == 1:
                self.donde_choques.append((round(self.x, 2), round(self.y, 2), round(self.yaw, 2),
                                           round(vx, 2), round(vy, 2), round(self.t, 1), self.escaleras))
            self.v[0] = self.v[1] = 0.0
        ny_ = self.yaw + wz * h
        if self._libre(self.x, self.y, ny_):
            self.yaw = math.atan2(math.sin(ny_), math.cos(ny_))
        zs = self.m.altura(self.x, self.y)
        self.z += min(1.0, h / 0.3) * (zs + 0.30 - self.z)
        # pitch del cuerpo: altura de los pies de delante menos la de los de atras (a +-0.19 m)
        f = self.m.altura(self.x + 0.19 * math.cos(self.yaw), self.y + 0.19 * math.sin(self.yaw))
        b = self.m.altura(self.x - 0.19 * math.cos(self.yaw), self.y - 0.19 * math.sin(self.yaw))
        self.pitch = -math.atan2(f - b, 0.38)
        self.historial.append((self.x, self.y))

    def _huella(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        for fx in (-0.33, 0.0, 0.33):
            for fy in (-0.15, 0.15):
                yield x + c * fx - s * fy, y + s * fx + c * fy

    def _libre(self, x, y, yaw):
        z0 = self.m.altura(self.x, self.y)
        for px, py in self._huella(x, y, yaw):
            if self.m.es_pared(px, py):
                return False
            dz = abs(self.m.altura(px, py) - z0)
            if dz > 0.12 and not self.escaleras:
                return False        # sin marcha de escaleras no sube/baja escalones
            if dz > 0.45:
                return False
        return True

    def _lidar(self):
        """Puntos visibles en un radio de 5 m: suelo + paredes, con oclusion."""
        R = 4.0
        # el L1 va en la cabeza: 28 cm adelante y 12 cm arriba del centro del cuerpo
        if self.l1_real:
            # la cabeza sube/baja con el cuerpo inclinado (pitch < 0 = hocico arriba en este simulador)
            a = -self.pitch
            fx_s, z_s = 0.29 * math.cos(a) + 0.047 * math.sin(a), 0.29 * math.sin(a) - 0.047 * math.cos(a)
        else:
            fx_s, z_s = 0.28, 0.12
        sx = self.x + fx_s * math.cos(self.yaw)
        sy = self.y + fx_s * math.sin(self.yaw)
        sz = self.z + z_s
        j0, j1 = max(0, int((sx - R) / RES)), min(self.m.W, int((sx + R) / RES))
        i0, i1 = max(0, int((sy - R) / RES)), min(self.m.H, int((sy + R) / RES))
        ii, jj = np.mgrid[i0:i1, j0:j1]
        ii, jj = ii.ravel(), jj.ravel()
        px = (jj + 0.5) * RES
        py = (ii + 0.5) * RES
        d = np.hypot(px - sx, py - sy)
        sel = (d < R) & (d > (0.10 if self.l1_real else 0.30))      # el L1 ve desde ~5-10 cm
        ii, jj, px, py, d = ii[sel], jj[sel], px[sel], py[sel], d[sel]
        pared = self.m.pared[ii, jj]
        pz = self.m.suelo[ii, jj]
        # oclusion: muestreo a lo largo del rayo cada <= 4 cm (menos que la pared mas delgada, 5 cm).
        # Antes eran 24 muestras por rayo (cada 15 cm en un rayo de 4 m): el lidar simulado veia A
        # TRAVES de paredes y puertas, y esos rayos borraban paredes del mapa de costos
        n = int(np.ceil(R / 0.04))
        ts = np.linspace(0.08 / R, 1.0 - 0.03 / R, n)
        visible = np.ones(len(px), bool)
        for t in ts:
            qx = sx + t * (px - sx)
            qy = sy + t * (py - sy)
            qi = np.clip((qy / RES).astype(int), 0, self.m.H - 1)
            qj = np.clip((qx / RES).astype(int), 0, self.m.W - 1)
            z_rayo = sz + t * (pz + (0.5 if False else 0.0) - sz)
            bloq = self.m.pared[qi, qj] | (self.m.suelo[qi, qj] > z_rayo + 0.02)
            visible &= ~bloq
        pts = []
        sv = visible & ~pared
        pts.append(np.c_[px[sv], py[sv], pz[sv]])
        # paredes: columna de puntos (si la cara es visible: rayo hasta casi la pared)
        pv = pared.copy()
        for t in ts[:-3]:
            qx = sx + t * (px - sx)
            qy = sy + t * (py - sy)
            qi = np.clip((qy / RES).astype(int), 0, self.m.H - 1)
            qj = np.clip((qx / RES).astype(int), 0, self.m.W - 1)
            pv &= ~self.m.pared[qi, qj] | ((np.abs(qi - ii) <= 1) & (np.abs(qj - jj) <= 1))
        z_base = self.m.zmin3[ii[pv], jj[pv]]          # la pared llega hasta el piso mas bajo junto a ella
        z_top = self.m.suelo[ii[pv], jj[pv]] + 1.0
        for f in np.linspace(0.08, 1.0, 7):
            pts.append(np.c_[px[pv], py[pv], z_base + f * (z_top - z_base)])
        if self.l1_real:
            pts[0] = pts[0][pts[0][:, 2] < sz - 0.01]         # una huella mas alta que el L1 no se ve
            pts.extend(self._caras_l1(sx, sy, sz, ii, jj, px, py, pz))
            xyz = np.concatenate(pts).astype(np.float32)
            return self._terminar(xyz, sx, sy, sz)
        # caras verticales de los desniveles (escalones, borde del entrepiso)
        zl = self.m.zmin3[ii, jj]
        salto = sv & (pz - zl > 0.08)
        if salto.any():
            for f in np.arange(0.1, 1.0, 0.15):
                zz = zl[salto] + f * (pz[salto] - zl[salto])
                pts.append(np.c_[px[salto], py[salto], zz])
        # la misma cara vista desde abajo (pie del desnivel)
        zh = self.m.zmax3[ii, jj]
        pie = sv & (zh - pz > 0.08)
        if pie.any():
            for f in np.arange(0.1, 1.0, 0.15):
                zz = pz[pie] + f * np.minimum(zh[pie] - pz[pie], 1.0)
                pts.append(np.c_[px[pie], py[pie], zz])
        xyz = np.concatenate(pts).astype(np.float32)
        return self._terminar(xyz, sx, sy, sz)

    def _visible(self, sx, sy, sz, px, py, pz, margen):
        """Rayo del lidar al punto sin cortar el terreno ni paredes (hasta 'margen' m antes de el)."""
        d = np.hypot(px - sx, py - sy)
        fin = np.clip(1.0 - margen / np.maximum(d, 1e-6), 0.0, 1.0)
        vis = np.ones(len(px), bool)
        n = 100
        for k in range(n):
            t = fin * (k + 0.5) / n
            qx, qy = sx + t * (px - sx), sy + t * (py - sy)
            qi = np.clip((qy / RES).astype(int), 0, self.m.H - 1)
            qj = np.clip((qx / RES).astype(int), 0, self.m.W - 1)
            z_rayo = sz + t * (pz - sz)
            vis &= ~(self.m.pared[qi, qj] | (self.m.suelo[qi, qj] > z_rayo + 0.01))
        return vis

    def _caras_l1(self, sx, sy, sz, ii, jj, px, py, pz):
        """Caras verticales de los escalones, cada punto con su propia oclusion (la huella de encima
        puede no verse y la cara si)."""
        out = []
        zl, zh = self.m.zmin3[ii, jj], self.m.zmax3[ii, jj]
        for base, top, sel in ((zl, pz, pz - zl > 0.08), (pz, np.minimum(zh, pz + 1.0), zh - pz > 0.08)):
            if not sel.any():
                continue
            for f in np.arange(0.1, 1.0, 0.15):
                zz = base[sel] + f * (top[sel] - base[sel])
                v = self._visible(sx, sy, sz, px[sel], py[sel], zz, 0.04)
                out.append(np.c_[px[sel][v], py[sel][v], zz[v]])
        return out

    def _terminar(self, xyz, sx, sy, sz):
        if self.l1_real and len(xyz):
            # campo de vision del L1 (boca abajo, inclinado 15 grados) y el cuerpo tapando atras
            # (en el marco del cuerpo: inclinado en la escalera, el campo de vision se inclina con el)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            dx = c * (xyz[:, 0] - sx) + s * (xyz[:, 1] - sy)
            dy = -s * (xyz[:, 0] - sx) + c * (xyz[:, 1] - sy)
            dz = xyz[:, 2] - sz
            a = -self.pitch
            dx, dz = dx * math.cos(a) + dz * math.sin(a), -dx * math.sin(a) + dz * math.cos(a)
            ok = (dz <= 0.27 * dx) & ~((dx < 0.05) & (np.abs(dy) < 0.30))
            xyz = xyz[ok]
        if self.m.mesas:
            xyz = np.concatenate([xyz, self._mesas(sx, sy, sz)]).astype(np.float32)
        if self.piso_brillante and len(xyz):
            xyz = self._brillo(xyz, sx, sy, sz)
        xyz += self.rng.normal(0, 0.01, xyz.shape).astype(np.float32)
        if self.ruido_real:
            xyz = np.concatenate([xyz, self._ruido_real(xyz)]).astype(np.float32)
            # el L1 real mide el suelo mas alto cerca de el (ver terreno.corregir_cerca)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            dx, dy = xyz[:, 0] - self.x, xyz[:, 1] - self.y
            r = np.hypot(c * dx + s * dy - 0.30, -s * dx + c * dy)
            xyz[:, 2] += np.maximum(0.0, 0.13 - 0.18 * r).astype(np.float32)
        x, y = self._a_odom(xyz[:, 0], xyz[:, 1])
        xyz[:, 0], xyz[:, 1] = x, y
        return xyz

    def _mesas(self, sx, sy, sz):
        """Cara de abajo de los tableros visibles (dentro del campo de vision del L1, sin oclusion)."""
        out = []
        for x0, y0, x1, y1, z in self.m.mesas:
            gx, gy = np.meshgrid(np.arange(x0, x1, 0.04), np.arange(y0, y1, 0.04))
            p = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z + self.m.altura(x0, y0))]
            out.append(p)
        p = np.concatenate(out) if out else np.zeros((0, 3))
        if self.l1_real and len(p):
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            dx = c * (p[:, 0] - sx) + s * (p[:, 1] - sy)
            p = p[(p[:, 2] - sz <= 0.27 * dx) & (dx > 0.05)]
        return p

    def _brillo(self, xyz, sx, sy, sz):
        """Piso brillante: el suelo (al nivel del perro) devuelve cada vez menos con la distancia
        (casi nada pasado 1.6 m) y lo que hay arriba aparece espejado abajo, si el rayo rebota en
        un pedazo de ese mismo piso."""
        g = self.m.altura(self.x, self.y)
        r = np.hypot(xyz[:, 0] - sx, xyz[:, 1] - sy)
        h = xyz[:, 2] - g
        suelo = np.abs(h) < 0.03
        p_ve = np.clip(1.0 - (r - 0.8) / 0.8, 0.05, 1.0)
        queda = ~suelo | (self.rng.random(len(xyz)) < p_ve)
        arriba = (h > 0.03) & (self.rng.random(len(xyz)) < 0.6)
        esp = xyz[arriba].copy()
        hs = sz - g
        t = hs / np.maximum(hs + (esp[:, 2] - g), 1e-3)                # donde rebota el rayo en el piso
        rx, ry = sx + t * (esp[:, 0] - sx), sy + t * (esp[:, 1] - sy)
        ri = np.clip((ry / RES).astype(int), 0, self.m.H - 1)
        rj = np.clip((rx / RES).astype(int), 0, self.m.W - 1)
        en_piso = (np.abs(self.m.suelo[ri, rj] - g) < 0.02) & ~self.m.pared[ri, rj]
        esp = esp[en_piso]
        esp[:, 2] = g - (esp[:, 2] - g)
        return np.concatenate([xyz[queda], esp])

    def _ruido_real(self, xyz):
        """Lo que el L1 real devuelve ademas del mundo:
        - el propio perro: 4 patas (pies a +-0.19 m) y la cabeza;
        - rebotes en piso brillante: reflejo de paredes/muebles por DEBAJO del suelo;
        - puntos sueltos por debajo del suelo (ruido de largo alcance)."""
        r = self.rng
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        g = self.m.altura(self.x, self.y)
        extra = []
        for fx in (-0.19, 0.19):
            for fy in (-0.19, 0.19):
                n = 25
                bx = fx + r.normal(0, 0.02, n)
                by = fy + r.normal(0, 0.02, n)
                bz = g + r.uniform(0.02, 0.30, n)
                extra.append(np.c_[self.x + c * bx - s * by, self.y + s * bx + c * by, bz])
        n = 30
        bx = r.uniform(0.28, 0.40, n)
        by = r.uniform(-0.08, 0.08, n)
        extra.append(np.c_[self.x + c * bx - s * by, self.y + s * bx + c * by, g + r.uniform(0.30, 0.42, n)])
        # fantasmas pegados al cuerpo (como el Go2 real: izq/der ~0.12 m, delante ~0.2 m del hocico)
        if getattr(self, "fantasmas", False):
            for (cx, cy, sx, sy, n) in ((0.0, 0.29, 0.15, 0.02, 14), (0.0, -0.29, 0.15, 0.02, 14),
                                        (0.57, 0.0, 0.02, 0.10, 10)):
                if r.random() < 0.75:
                    bx = cx + r.uniform(-sx, sx, n)
                    by = cy + r.uniform(-sy, sy, n)
                    extra.append(np.c_[self.x + c * bx - s * by, self.y + s * bx + c * by,
                                       g + r.uniform(0.12, 0.35, n)])
        # reflejos: puntos altos (paredes) espejados bajo el piso, 40 % de ellos
        alto = xyz[:, 2] - g > 0.08
        sel = alto & (r.random(len(xyz)) < 0.4)
        refl = xyz[sel].copy()
        refl[:, 2] = g - (refl[:, 2] - g)
        extra.append(refl)
        # ruido suelto bajo el piso a 0.5-3 m
        n = 300
        ang = r.uniform(-math.pi, math.pi, n)
        d = r.uniform(0.5, 3.0, n)
        extra.append(np.c_[self.x + d * np.cos(ang), self.y + d * np.sin(ang), g - r.uniform(0.10, 0.35, n)])
        return np.concatenate(extra)

    def _a_odom(self, x, y):
        ox, oy, oyaw = self.org
        c, s = math.cos(-oyaw + self.odo_yaw_err), math.sin(-oyaw + self.odo_yaw_err)
        dx, dy = x - ox, y - oy
        return c * dx - s * dy, s * dx + c * dy

    # ---- interfaz Robot
    def pose(self):
        x, y = self._a_odom(self.x, self.y)
        yaw = self.yaw - self.org[2] + self.odo_yaw_err
        return (x, y, self.z, math.atan2(math.sin(yaw), math.cos(yaw)), self.pitch, self.roll, self.now())

    def nubes_nuevas(self):
        n, self.nubes = self.nubes, []
        return n

    def bateria(self):
        return 80

    def mover(self, vx, vy, wz):
        self.cmd = (float(vx), float(vy), float(wz))
        self.t_ult_cmd = self.t

    def parar(self):
        self.cmd = (0.0, 0.0, 0.0)
        self.t_ult_cmd = self.t

    def modo_escaleras(self, activo):
        self.escaleras = bool(activo)
        return 1.0

    def info(self):
        return {"firmware": "sim", "version": "sim", "lidar_hace_s": 0.1, "bateria": 80}
