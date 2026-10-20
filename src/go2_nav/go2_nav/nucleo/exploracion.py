"""Exploracion por fronteras: la rejilla de "suelo visto" y la eleccion de metas (puro numpy).

slam_toolbox solo marca libre hasta donde hay una pared: el suelo que el L1 SI ve pero sin
pared detras queda "desconocido" y la exploracion terminaba antes de tiempo. Por eso el gestor
lleva su propia rejilla de SUELO VISTO (de /go2/suelo) y de obstaculos, una por piso.

Rejilla: VN x VN celdas de VRES m centradas en el origen del mapa.
  visto  uint8: cuantas veces se vio suelo en la celda (>= 2 = suelo seguro)
  ocup   float: evidencia de obstaculo (sube con cada nube, decae con el tiempo: la gente se va)
"""
import collections
import math

import numpy as np

VRES, VN = 0.10, 1000           # 100 x 100 m, celdas de 10 cm
OCUPADO = 4.0                   # evidencia a partir de la cual una celda es obstaculo


def celda(x, y):
    return int(math.floor(y / VRES)) + VN // 2, int(math.floor(x / VRES)) + VN // 2


def mundo(i, j):
    return (j - VN // 2 + 0.5) * VRES, (i - VN // 2 + 0.5) * VRES


def celdas(x, y):
    """Vectorial: arrays de x, y -> (i, j, dentro)."""
    i = np.floor(y / VRES).astype(np.int64) + VN // 2
    j = np.floor(x / VRES).astype(np.int64) + VN // 2
    return i, j, (i >= 0) & (i < VN) & (j >= 0) & (j < VN)


class Vista(object):
    """Lo que el perro ya vio en UN piso."""

    def __init__(self):
        self.visto = np.zeros((VN, VN), np.uint8)
        self.ocup = np.zeros((VN, VN), np.float32)

    def marcar_suelo(self, i, j):
        v = self.visto[i, j]
        self.visto[i, j] = np.where(v < 250, v + 1, v)

    def marcar_obstaculos(self, i, j):
        np.add.at(self.ocup, (i, j), 1.0)
        self.visto[i, j] = np.maximum(self.visto[i, j], 1)

    def decaer(self, factor=0.7):
        self.ocup *= factor

    def pisado(self, pose, r=0.55):
        """Donde el perro ESTA parado es suelo, aunque el lidar no lo vea (zona ciega bajo el
        cuerpo): sin esto el perro quedaba en una 'isla sin ver', no encontraba camino a ninguna
        zona y daba el piso por terminado sin salir del cuarto."""
        i, j = celda(pose[0], pose[1])
        k = int(math.ceil(r / VRES))
        a0, b0 = max(0, i - k), max(0, j - k)
        sub = self.visto[a0:i + k + 1, b0:j + k + 1]
        yy, xx = np.mgrid[a0:a0 + sub.shape[0], b0:b0 + sub.shape[1]]
        disco = (yy - i) ** 2 + (xx - j) ** 2 <= k * k
        sub[disco] = np.maximum(sub[disco], 2)
        self.ocup[a0:i + k + 1, b0:j + k + 1][disco] = 0.0

    def zona_vista(self, x, y, r=0.3, max_sin_ver=None):
        """La zona de r m alrededor de (x, y) ya se vio (casi) toda."""
        i, j = celda(x, y)
        k = int(r / VRES)
        sub = self.visto[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1]
        oc = self.ocup[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1] >= OCUPADO
        sin_ver = np.count_nonzero((sub == 0) & ~oc)
        if max_sin_ver is not None:
            return sub.size > 0 and sin_ver <= max_sin_ver
        return sub.size > 0 and sin_ver < 0.10 * sub.size

    def area(self):
        return round(float(np.count_nonzero(self.visto >= 2)) * VRES ** 2, 1)

    def recorte(self, margen):
        """(slice_i, slice_j) de la zona vista con margen, o None si no se vio nada."""
        ii, jj = np.nonzero(self.visto)
        if len(ii) == 0:
            return None
        return (slice(max(0, ii.min() - margen), min(VN, ii.max() + margen + 1)),
                slice(max(0, jj.min() - margen), min(VN, jj.max() + margen + 1)))

    def rejilla_ocupacion(self, margen=10):
        """Para RViz: (grid int8 -1/0/100, x0, y0 del origen) o None."""
        sl = self.recorte(margen)
        if sl is None:
            return None
        v = self.visto[sl]
        oc = self.ocup[sl] >= OCUPADO
        g = np.full(v.shape, -1, np.int8)
        g[v >= 2] = 0
        g[oc] = 100
        x0, y0 = mundo(sl[0].start, sl[1].start)
        return g, x0 - VRES / 2, y0 - VRES / 2


def paredes_slam(mapa, largo_min=4, engordar=0):
    """Paredes del mapa de slam_toolbox (OccupancyGrid) como mascara bool (alto x ancho).

    1) une los puntos de una misma pared separados por 1 celda (una pared vista "punteada");
    2) quita los pedazos de menos de largo_min celdas (20 cm): un puntito de ruido, un rebote;
    3) engorda 'engordar' celdas (por defecto 0: el inflado de Nav2 ya da la distancia; engordar
       achicaba las puertas 10 cm y el planificador a veces no encontraba ruta por ellas).
    Lo desconocido NO es pared."""
    from scipy import ndimage
    g = np.array(mapa.data, np.int8).reshape(mapa.info.height, mapa.info.width)
    occ = g >= 50
    if not occ.any():
        return occ
    ocho = np.ones((3, 3), bool)
    unido = ndimage.binary_dilation(occ, ocho)
    lab, n = ndimage.label(unido, structure=ocho)
    # largo de cada pedazo = celdas ocupadas de verdad que tiene
    tam = np.bincount(lab[occ], minlength=n + 1)
    tam[0] = 0
    # grosor real (sin el engorde del paso 1) + los huecos de 1 celda rellenados
    pared = (tam >= largo_min)[lab] & (occ | _huecos_de_una(occ))
    if engordar > 0 and pared.any():
        pared = ndimage.binary_dilation(pared, ocho, iterations=engordar)
    return pared


def _huecos_de_una(occ):
    """Celdas libres con pared a ambos lados (horizontal, vertical o diagonal): hueco de 1 celda."""
    p = np.pad(occ, 1)
    c = p[1:-1, 1:-1]
    h = p[1:-1, :-2] & p[1:-1, 2:]
    v = p[:-2, 1:-1] & p[2:, 1:-1]
    d1 = p[:-2, :-2] & p[2:, 2:]
    d2 = p[:-2, 2:] & p[2:, :-2]
    return ~c & (h | v | d1 | d2)


def ocupado_segun_slam(mapa, sl):
    """Paredes del SLAM (paredes_slam) llevadas a la rejilla de exploracion, dentro del recorte sl."""
    out = np.zeros((sl[0].stop - sl[0].start, sl[1].stop - sl[1].start), bool)
    if mapa is None:
        return out
    ii, jj = np.nonzero(paredes_slam(mapa, largo_min=2, engordar=0))
    if len(ii) == 0:
        return out
    x = mapa.info.origin.position.x + (jj + 0.5) * mapa.info.resolution
    y = mapa.info.origin.position.y + (ii + 0.5) * mapa.info.resolution
    a = np.floor(y / VRES).astype(np.int64) + VN // 2 - sl[0].start
    b = np.floor(x / VRES).astype(np.int64) + VN // 2 - sl[1].start
    ok = (a >= 0) & (a < out.shape[0]) & (b >= 0) & (b < out.shape[1])
    out[a[ok], b[ok]] = True
    return out


def zonas(sl, puntos, radio):
    """Mascara del recorte con un cuadrado de 'radio' m alrededor de cada punto (x, y)."""
    out = np.zeros((sl[0].stop - sl[0].start, sl[1].stop - sl[1].start), bool)
    k = int(radio / VRES)
    for x, y in puntos:
        i, j = celda(x, y)
        a0, b0 = i - k - sl[0].start, j - k - sl[1].start
        out[max(0, a0):max(0, a0 + 2 * k + 1), max(0, b0):max(0, b0 + 2 * k + 1)] = True
    return out


def bfs(ok, si, sj):
    """Distancia por camino (m) desde (si, sj) por celdas 'ok' (8 vecinos). inf = sin camino."""
    H, W = ok.shape
    dist = np.full((H, W), np.inf)
    if not (0 <= si < H and 0 <= sj < W):
        return dist
    if not ok[si, sj]:
        R = 10
        ii, jj = np.nonzero(ok[max(0, si - R):si + R + 1, max(0, sj - R):sj + R + 1])
        if len(ii) == 0:
            return dist
        k = int(np.argmin((ii - min(si, R)) ** 2 + (jj - min(sj, R)) ** 2))
        si, sj = ii[k] + max(0, si - R), jj[k] + max(0, sj - R)
    dist[si, sj] = 0.0
    q = collections.deque([(si, sj)])
    vec = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
           (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
    while q:
        i, j = q.popleft()
        d0 = dist[i, j]
        for di, dj, c in vec:
            a, b = i + di, j + dj
            if 0 <= a < H and 0 <= b < W and ok[a, b] and dist[a, b] > d0 + c * VRES + 1e-9:
                dist[a, b] = d0 + c * VRES
                q.append((a, b))
    return dist


def fronteras(vista, pose, mapa_slam, excluir, vetadas, frontera_min, hueco_min, holgura_meta, d_min_meta):
    """Metas de exploracion ordenadas por costo (la primera es la mejor).

    vista        Vista del piso (ya con pisado() aplicado y copiada bajo el lock del gestor:
                 se le pasan los arrays, no se escribe)
    pose         (x, y, yaw) en el mapa
    mapa_slam    OccupancyGrid de slam_toolbox o None
    excluir      [(x, y)] zonas de escaleras: lo que hay mas alla se descubre CRUZANDOLAS
    vetadas      [(x, y)] metas que fallaron
    Devuelve (lista de dict(x, y, n, d, costo[, directa]), celdas de frontera [(x, y)]).
    directa = no hay camino por suelo visto: el camino cruza lo no visto (nunca paredes).
    """
    from scipy import ndimage
    sl = vista.recorte(8)
    if sl is None:
        return [], []
    visto = vista.visto[sl].copy()
    ocup = (vista.ocup[sl] >= OCUPADO) | ocupado_segun_slam(mapa_slam, sl)
    libre = (visto >= 2) & ~ocup
    desc = (visto == 0) & ~ocup
    # huecos sin ver mas chicos que hueco_min (detras de un mueble, sombra del lidar) NO son
    # zonas a explorar: no valen el viaje y hacian que nunca saliera del cuarto
    lab_d, _n = ndimage.label(desc)
    tam = np.bincount(lab_d.ravel())
    tam[0] = 0
    desc = (tam >= hueco_min / VRES ** 2)[lab_d] & desc
    dist = ndimage.distance_transform_edt(~ocup) * VRES
    pasable = libre & (dist >= 0.15)
    borde = np.zeros_like(desc)
    borde[1:] |= desc[:-1]
    borde[:-1] |= desc[1:]
    borde[:, 1:] |= desc[:, :-1]
    borde[:, :-1] |= desc[:, 1:]
    # (la holgura NO se exige a toda la frontera: en una puerta de 70 cm solo el centro tiene
    # 22 cm a cada lado y la frontera quedaba "chica" y descartada -> nunca salia del cuarto)
    fr = pasable & borde & ~zonas(sl, excluir, 0.9)
    si, sj = celda(pose[0], pose[1])
    dcam = bfs(pasable, si - sl[0].start, sj - sl[1].start)
    lab, _n = ndimage.label(fr, structure=np.ones((3, 3)))
    objetos = list(enumerate(ndimage.find_objects(lab), start=1))

    def vetada(x, y):
        return any(math.hypot(x - vx, y - vy) < 0.6 for vx, vy in vetadas)

    def giro(x, y):
        # preferir lo que tiene DELANTE: cada radian de giro "cuesta" como 0.4 m de camino
        # (asi sigue de frente hacia los cuartos en vez de irse de lado y volver)
        return abs(_ang(math.atan2(y - pose[1], x - pose[0]) - pose[2]))

    out = []
    for k, ob in objetos:
        a, b_ = np.nonzero(lab[ob] == k)
        if len(a) < frontera_min:
            continue
        a = a + ob[0].start
        b_ = b_ + ob[1].start
        d = dcam[a, b_]
        holg = dist[a, b_]
        # la meta va donde el perro CABE con holgura (en el centro de una puerta de 60-80 cm
        # hay 30-40 cm): rincones mas angostos no, ahi no puede ni girar sin tocar
        if float(holg.max()) < holgura_meta:
            continue
        ok = np.isfinite(d) & (holg >= holgura_meta - 1e-6)
        if not ok.any():
            continue
        lej = np.nonzero(ok & (d >= d_min_meta))[0]
        b = int(lej[np.argmin(d[lej])]) if len(lej) else int(np.nonzero(ok)[0][np.argmin(d[ok])])
        x, y = mundo(int(a[b] + sl[0].start), int(b_[b] + sl[1].start))
        if vetada(x, y):
            continue
        out.append({"x": x, "y": y, "n": int(len(a)), "d": float(d[b]),
                    "costo": float(d[b]) - 0.15 * math.sqrt(len(a)) + 0.4 * giro(x, y)})
    fi, fj = np.nonzero(fr)
    xs, ys = mundo(fi + sl[0].start, fj + sl[1].start)
    celdas_fr = list(zip(xs.tolist(), ys.tolist()))
    if not out:
        # ninguna zona con camino por suelo YA VISTO: voy a la mas cercana por un camino que puede
        # cruzar lo no visto pero NUNCA una pared conocida (antes era en linea recta, y Nav2 la
        # buscaba cruzando la pared en vez de salir por la puerta)
        dlibre = bfs(~ocup & (dist >= 0.15), si - sl[0].start, sj - sl[1].start)
        for k, ob in objetos:
            a, b_ = np.nonzero(lab[ob] == k)
            if len(a) < frontera_min:
                continue
            a = a + ob[0].start
            b_ = b_ + ob[1].start
            d = dlibre[a, b_].copy()
            d[dist[a, b_] < 0.15] = np.inf
            cand = np.nonzero(np.isfinite(d) & (d >= d_min_meta))[0]
            if not len(cand):
                continue
            b = int(cand[np.argmin(d[cand])])
            x, y = mundo(int(a[b] + sl[0].start), int(b_[b] + sl[1].start))
            if vetada(x, y):
                continue
            out.append({"x": x, "y": y, "n": int(len(a)), "d": float(d[b]), "directa": True,
                        "costo": float(d[b]) * 1.3 - 0.15 * math.sqrt(len(a)) + 0.4 * giro(x, y)})
    out.sort(key=lambda o: o["costo"])
    return out, celdas_fr


def _ang(a):
    return math.atan2(math.sin(a), math.cos(a))
