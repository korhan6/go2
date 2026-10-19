"""
terreno.py - Analiza el terreno cerca del perro con la nube 3D del lidar L1.

Lo usan:
  control.py   guardian anti-choques (obstaculos y caidas en las 4 direcciones)
  escaleras.py deteccion de escaleras de bajada (borde, ancho, angulo) y de subida

Convenciones
  La nube del Go2 esta en 'odom' (fija al mundo, z hacia arriba).
  fx = hacia adelante del perro, fy = hacia su izquierda (m, desde el centro del cuerpo).
  h  = altura sobre el suelo que pisa el perro (m).
"""
import math

import numpy as np

# Cuerpo del Go2 (de pie): 0.70 x 0.31 m.  Medio largo / medio ancho desde el centro.
LARGO2 = 0.36
ANCHO2 = 0.17
PROPIO = 0.06          # m alrededor del cuerpo que se ignoran (patas: los pies van a ~19 cm del centro)
ALTURA_BASE = 0.30     # base_link sobre el suelo (solo si todavia no se ve el suelo)

PENDIENTE = 0.27       # tan(15 grados): rampa maxima que se trata como suelo
OBST_MIN = 0.10        # m: mas bajo que esto se pisa (cables, umbrales de puerta)
OBST_MAX = 0.60        # m: mas alto que esto el perro pasa por debajo (mesas)
SUELO_TOL = 0.05       # m: +-5 cm del suelo = suelo
CAIDA_MIN = 0.10       # m por debajo del suelo = escalon o caida

# deteccion de escaleras (rejilla delante del perro: carriles laterales x franjas)
CARRIL = 0.10
LAT = 0.70
X0, X1, PASO = 0.15, 2.20, 0.05
HUECO = 5              # franjas seguidas sin suelo (25 cm) para llamarlo borde
RAMPA_MAX = 0.40       # m de caida por m de avance: mas suave que esto es rampa, no escalera
ANCHO_MIN = 0.45       # m: escalera mas angosta que esto no se baja sola


# ---------------------------------------------------------------- lectura
def leer_nube(msg):
    """PointCloud2 (DDS de Unitree o ROS) -> array Nx3 float32 con los puntos finitos."""
    n = int(msg.width) * int(msg.height)
    off = {f.name: int(f.offset) for f in msg.fields}
    paso = int(msg.point_step)
    if n == 0 or paso < 12 or not all(k in off for k in "xyz"):
        return np.zeros((0, 3), np.float32)
    datos = msg.data
    if isinstance(datos, (bytes, bytearray, memoryview)):
        raw = np.frombuffer(datos, np.uint8)
    else:                                   # cyclonedds entrega sequence<uint8> como lista
        raw = np.asarray(datos, dtype=np.uint8)
    if raw.size < n * paso:
        n = raw.size // paso
    raw = raw[: n * paso].reshape(n, paso)
    xyz = np.empty((n, 3), np.float32)
    for i, k in enumerate("xyz"):
        o = off[k]
        xyz[:, i] = raw[:, o:o + 4].copy().view("<f4").ravel()
    return xyz[np.isfinite(xyz).all(axis=1)]


def quitar_propio(xyz, x, y, z, yaw, suelo=None):
    """Quita el propio perro de una nube (cuerpo, 4 patas, cabeza). El L1 se ve a si mismo y,
    si esos puntos quedan en la memoria mientras el perro gira, rodean al perro de 'obstaculos'."""
    if len(xyz) == 0:
        return xyz
    piso = (z - ALTURA_BASE) if suelo is None else suelo
    fx, fy = a_cuerpo(xyz, x, y, yaw)
    yo = ((np.abs(fx) < LARGO2 + 0.12) & (np.abs(fy) < ANCHO2 + 0.13)
          & (xyz[:, 2] > piso + 0.03) & (xyz[:, 2] < z + 0.35))
    return xyz[~yo]


LIDAR_FX = 0.30        # m: el L1 va en la cara, ~30 cm delante del centro del cuerpo


def corregir_cerca(xyz, x, y, yaw, sesgo):
    """El L1 mide el suelo MAS ALTO cuanto mas cerca de el esta (medido en un Go2 real, piso
    brillante: +12 cm a 15 cm del lidar, +7 cm a 35 cm, 0 a ~0.7 m). Sin esto el suelo junto a la
    cabeza parece un obstaculo bajo: 'izq/der 0.12', 'adel 0.2', 'sin espacio para girar'.
    sesgo = (a, b): se resta max(0, a - b*r), r = distancia horizontal al lidar."""
    a, b = sesgo
    if len(xyz) == 0 or a <= 0.0:
        return xyz
    fx, fy = a_cuerpo(xyz, x, y, yaw)
    r = np.hypot(fx - LIDAR_FX, fy)
    s = np.maximum(0.0, a - b * r)
    if not np.any(s > 0):
        return xyz
    out = xyz.copy()
    out[:, 2] -= s.astype(out.dtype)
    return out


def recortar(xyz, x, y, radio):
    m = (np.abs(xyz[:, 0] - x) < radio) & (np.abs(xyz[:, 1] - y) < radio)
    return xyz[m]


def a_cuerpo(xyz, x, y, yaw):
    """Puntos (odom) -> coordenadas del perro (fx adelante, fy izquierda)."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx = xyz[:, 0] - x
    dy = xyz[:, 1] - y
    return c * dx + s * dy, -s * dx + c * dy


def suelo_abs(xyz, x, y, z, yaw, previo=None, medido=False):
    """Altura (odom) del suelo que pisa el perro: el nivel con mas puntos alrededor de las patas.

    Se usa el nivel mas poblado (no la mediana) para que un borde de escalera o una mesa
    cerca no muevan la estimacion. Con medido=True devuelve (altura, se_vio_el_suelo).
    """
    g, ok = (previo if previo is not None else z - ALTURA_BASE), False
    if len(xyz):
        fx, fy = a_cuerpo(xyz, x, y, yaw)
        dz = xyz[:, 2] - z
        # anillo alrededor del perro, SIN el propio cuerpo y las patas (si no, las patas
        # parecen "el suelo" a 15 cm y todo el piso de verdad se ve como una bajada)
        propio = (np.abs(fx) < 0.48) & (np.abs(fy) < 0.30)
        m = (np.abs(fx) < 1.1) & (np.abs(fy) < 0.8) & ~propio & (dz < -0.12) & (dz > -0.70)
        if np.count_nonzero(m) >= 10:
            d = dz[m]
            hist, bordes = np.histogram(d, bins=np.arange(-0.70, -0.10, 0.02))
            i = int(np.argmax(hist))
            c = 0.5 * (bordes[i] + bordes[i + 1])
            g, ok = float(z + np.median(d[np.abs(d - c) < 0.03])), True
    return (g, ok) if medido else g


# ---------------------------------------------------------------- guardian
DIRECCIONES = {"adelante": (1, 0), "atras": (-1, 0), "izq": (0, 1), "der": (0, -1)}


def _kmin(a, k=2):
    """k-esimo menor (robusto a un voxel de ruido). inf si hay menos de k."""
    if a.size < k:
        return math.inf
    return float(np.partition(a, k - 1)[k - 1])


def es_rampa(a, h, lo, paso=0.05, largo=0.5):
    """Lo que hay desde 'lo' hacia adelante es una rampa: superficie continua (sin hueco de
    sombra) que cambia de altura poco a poco. Un escalon salta >= 10 cm de golpe o deja sombra."""
    sel = (a >= lo - paso) & (a < lo + largo)
    if np.count_nonzero(sel) < 6:
        return False
    nb = int(round((largo + paso) / paso))
    b = ((a[sel] - (lo - paso)) / paso).astype(int).clip(0, nb - 1)
    alto = np.full(nb, -np.inf)
    np.maximum.at(alto, b, h[sel])
    ok = np.isfinite(alto)
    if np.count_nonzero(ok) < 0.8 * nb:
        return False
    v = alto[ok]
    return bool(np.all(np.abs(np.diff(v)) <= 0.06)            # voxeles de 5 cm: 1 nivel por franja
                and abs(v[-1] - v[0]) <= PENDIENTE * 1.2 * (largo + paso))


def holguras(fx, fy, h, largo=1.2, margen=0.06, obst_min=OBST_MIN, rampa=False):
    """Espacio libre alrededor del cuerpo.

    Devuelve dict direccion -> (d_obstaculo, d_caida, d_hueco), en m medidos desde el borde
    del cuerpo (inf = nada a la vista):
      d_caida  borde con puntos por debajo del suelo (escalon, escalera, hueco real)
      d_hueco  el suelo se acaba y no se ve nada despues (puede ser una escalera vista de
               lejos, cuyo fondo aun no se ve, o solo una alfombra oscura): ir lento
    y 'giro' -> distancia minima desde el centro a un obstaculo dentro del circulo que barre
    el perro al girar.
    """
    out = {}
    for nombre, (ux, uy) in DIRECCIONES.items():
        if ux:
            a = ux * fx - LARGO2
            lat, w = fy, ANCHO2 + margen
        else:
            a = uy * fy - ANCHO2
            lat, w = fx, LARGO2 + margen
        m = (a > PROPIO) & (a < largo) & (np.abs(lat) < w)
        # Rampas (v5.6): el suelo puede subir o bajar hasta PENDIENTE (15 grados) con la
        # distancia sin ser obstaculo ni caida. Antes una "subidita" era una pared delante.
        ap = np.clip(a, 0.0, None)
        tol = PENDIENTE * ap
        # obstaculo = algo VERTICAL: sobresale obst_min por encima de lo mas bajo de su franja
        # de 10 cm. Una rampa sube 2-3 cm por franja; una pared, caja o persona, mucho mas.
        fr = np.clip((ap / 0.1).astype(int), 0, int(largo / 0.1) + 1)
        base = np.full(int(largo / 0.1) + 2, np.inf)
        np.minimum.at(base, fr[m], h[m])
        vertical = (h - base[fr]) > obst_min * 0.8
        obst = (h > obst_min) & (h < OBST_MAX + tol) & vertical
        # suelo: plano, o subiendo en rampa (subir nunca hace caer). Bajando, en plano es estricto
        # (un escalon de 17 cm NO es suelo); si el perro ya esta en una rampa, se tolera la pendiente.
        extra = PENDIENTE * (ap + LARGO2)
        if rampa:
            piso = np.abs(h) < SUELO_TOL + extra
            bajo = h < -(CAIDA_MIN + extra)
        else:
            piso = (np.abs(h) < SUELO_TOL) | ((h > 0.0) & (h < SUELO_TOL + extra))
            bajo = h < -CAIDA_MIN
        d_ob = _kmin(a[m & obst])
        # caida: una franja de 10 cm sin suelo con puntos por debajo del suelo en ella o justo detras
        d_ca = math.inf
        ab = a[m & bajo]
        am, hm = a[m], h[m]
        if ab.size >= 3:
            ap = a[m & piso]
            hb = h[m & bajo]
            for k in range(7):
                lo = k * 0.1
                # sin suelo 20 cm seguidos (un parche ciego justo delante del hocico no es borde:
                # si el suelo sigue enseguida, no hay caida)
                if np.count_nonzero((ap >= lo) & (ap < lo + 0.2)) > 0:
                    continue
                sel = (ab >= lo) & (ab < lo + 0.3)
                if np.count_nonzero(sel) < 8:
                    continue
                # lo de abajo tiene que ser una SUPERFICIE (escalon, piso de abajo), no puntos
                # sueltos de ruido o rebotes: al menos 8 voxeles a la misma altura (+-3 cm)
                v = np.sort(hb[sel])
                superficie = bool(np.any(v[7:] - v[:-7] <= 0.06)) if v.size >= 8 else False
                if superficie and not es_rampa(am, hm, lo):
                    d_ca = lo
                    break
        # hueco: hubo suelo y luego 20 cm sin suelo ni obstaculos
        d_hu = math.inf
        ap = a[m & piso]
        if ap.size:
            ao_all = a[m & obst]
            hay = [np.count_nonzero((ap >= k * 0.1) & (ap < k * 0.1 + 0.1)) > 0 for k in range(10)]
            for k in range(1, 9):
                lo = k * 0.1
                if (hay[k - 1] and not hay[k] and not hay[k + 1]
                        and np.count_nonzero((ao_all >= lo) & (ao_all < lo + 0.2)) == 0
                        and not es_rampa(am, hm, lo)):
                    d_hu = lo
                    break
        out[nombre] = (d_ob, d_ca, d_hu)
    obst = (h > obst_min) & (h < OBST_MAX)
    r = np.hypot(fx, fy)
    dentro = (np.abs(fx) < LARGO2 + PROPIO) & (np.abs(fy) < ANCHO2 + PROPIO)
    # 4 puntos (no 2): un par de pixeles sueltos no impiden girar
    out["giro"] = _kmin(r[obst & ~dentro & (r < 0.50)], k=4)
    return out


def mitades_frente(fx, fy, h, largo=0.6, margen=0.06, obst_min=OBST_MIN):
    """Distancia (desde el hocico) al obstaculo mas cercano en la mitad izquierda y derecha del
    carril de delante. Sirve para ver un marco de puerta que solo tapa un lado."""
    obst = (h > obst_min) & (h < OBST_MAX)
    a = fx - LARGO2
    m = obst & (a > PROPIO) & (a < largo) & (np.abs(fy) < ANCHO2 + margen)
    return _kmin(a[m & (fy >= 0)]), _kmin(a[m & (fy < 0)])


# ---------------------------------------------------------------- escaleras
def _rejilla(fx, fy, h):
    """Rejilla carriles x franjas delante del perro con: hay suelo / hay algo bajo / hay obstaculo."""
    nl = int(round(2 * LAT / CARRIL))
    nb = int(round((X1 - X0) / PASO))
    sel = (fx >= X0) & (fx < X1) & (np.abs(fy) < LAT) & (h < OBST_MAX)
    fx, fy, h = fx[sel], fy[sel], h[sel]
    li = ((fy + LAT) / CARRIL).astype(int).clip(0, nl - 1)
    bi = ((fx - X0) / PASO).astype(int).clip(0, nb - 1)
    piso = np.zeros((nl, nb), bool)
    bajo = np.zeros((nl, nb), bool)
    alto = np.zeros((nl, nb), bool)
    hmin = np.full((nl, nb), np.nan)
    hmaxb = np.full((nl, nb), np.nan)
    m = np.abs(h) < SUELO_TOL
    piso[li[m], bi[m]] = True
    m = h < -CAIDA_MIN
    bajo[li[m], bi[m]] = True
    if m.any():
        tmp = np.full((nl, nb), np.inf)
        np.minimum.at(tmp, (li[m], bi[m]), h[m])
        hmin = np.where(np.isfinite(tmp), tmp, np.nan)
        tmp = np.full((nl, nb), -np.inf)
        np.maximum.at(tmp, (li[m], bi[m]), h[m])
        # solo si es una superficie: >= 2 voxeles a menos de 4 cm de ese nivel (no un punto suelto)
        cerca_max = m.copy()
        cerca_max[m] = h[m] >= tmp[li[m], bi[m]] - 0.04
        cnt = np.zeros((nl, nb), np.int32)
        np.add.at(cnt, (li[cerca_max], bi[cerca_max]), 1)
        hmaxb = np.where(np.isfinite(tmp) & (cnt >= 2), tmp, np.nan)
    m = h > OBST_MIN
    alto[li[m], bi[m]] = True
    centros = -LAT + (np.arange(nl) + 0.5) * CARRIL
    _rejilla.hmaxb = hmaxb
    return piso, bajo, alto, hmin, centros


def _carril(piso, bajo, alto):
    """Clasifica un carril: ('caida', franja_borde) | ('pared', i) | ('suelo', None) | ('nada', None)."""
    nb = len(piso)
    vio_suelo = 0
    i = 0
    while i < nb:
        if alto[i]:
            return ("pared", i)
        if piso[i]:
            vio_suelo += 1
            i += 1
            continue
        fin = min(nb, i + HUECO)
        if vio_suelo >= 2 and fin - i == HUECO and not piso[i:fin].any() and not alto[i:fin].any():
            if bajo[i:min(nb, i + 24)].any():          # se ve algo mas abajo en los siguientes 1.2 m
                return ("caida", i)
            # hueco sin nada debajo: borde dudoso (alfombra negra, vidrio, fuera de alcance)
            return ("nada", i)
        i += 1
    return ("suelo", None) if vio_suelo else ("nada", None)


def analizar_bajada(fx, fy, h):
    """Busca una escalera (o escalon) hacia abajo delante del perro.

    Devuelve (deteccion | None, estado_centro).
      deteccion: dict(bx, by, dist, ang, centro, ancho, prof, abierta)
        bx, by punto central del borde (m, coordenadas del perro)
        dist   distancia perpendicular del centro del perro al borde (m)
        ang    angulo de la normal del borde respecto al frente del perro (rad, + = izquierda)
        centro desplazamiento lateral del centro de la escalera (m, + = izquierda)
        ancho  ancho visto de la bajada (m); abierta = llega al limite de lo que se mira
        prof   lo mas bajo que se ve despues del borde (m, negativo)
      estado_centro: 'caida' | 'suelo' | 'pared' | 'nada' (lo que hay justo delante)
    """
    piso, bajo, alto, hmin, centros = _rejilla(fx, fy, h)
    estados = [_carril(piso[k], bajo[k], alto[k]) for k in range(len(centros))]
    medio = [k for k in range(len(centros)) if abs(centros[k]) <= 0.15 + 1e-6]
    tipos = [estados[k][0] for k in medio]
    estado_centro = max(set(tipos), key=tipos.count)
    if any(estados[k][0] != "caida" for k in medio):
        return None, estado_centro
    # ensancha desde el centro mientras los carriles tambien sean bajada
    k0, k1 = min(medio), max(medio)
    while k0 > 0 and estados[k0 - 1][0] == "caida":
        k0 -= 1
    while k1 < len(centros) - 1 and estados[k1 + 1][0] == "caida":
        k1 += 1
    ks = np.arange(k0, k1 + 1)
    lat = centros[ks]
    borde = np.array([X0 + estados[k][1] * PASO for k in ks])
    b, a = np.polyfit(lat, borde, 1)          # borde(lat) = a + b * lat
    resid = borde - (a + b * lat)
    if float(np.std(resid)) > 0.07 or abs(b) > 1.2:   # no es una linea recta: sombra, objetos
        return None, estado_centro
    prof = []
    empinados = 0
    hmaxb = _rejilla.hmaxb
    niveles = []
    for k in ks:
        i = estados[k][1]
        v = hmin[k, i:i + 24]
        d = (np.arange(v.size) + 1) * PASO
        ok = np.isfinite(v)
        if ok.any():
            prof.append(float(v[ok].min()))
            # una escalera cae empinado (~30 grados o mas, y con la sombra parece mas);
            # una rampa baja suave: menos de 0.40 m por metro no es escalera
            if np.any(ok & (v <= -0.12) & (-v / d >= RAMPA_MAX)):
                empinados += 1
        # SEGURIDAD: una escalera tiene un PRIMER ESCALON plano (huella) a 10-27 cm bajo el borde,
        # visible en al menos 2 franjas seguidas y a la misma altura en todos los carriles.
        # Un balcon, un entrepiso o un hueco solo muestran el fondo (muy abajo); el ruido o los
        # rebotes del lidar no forman una huella plana y pareja en varios carriles.
        w = hmaxb[k, i:i + 10]
        for q in range(len(w) - 1):
            if (np.isfinite(w[q]) and np.isfinite(w[q + 1]) and -0.27 <= w[q] <= -0.10
                    and abs(w[q] - w[q + 1]) < 0.04):
                niveles.append(0.5 * float(w[q] + w[q + 1]))
                break
    if len(prof) < 2 or empinados < max(2, len(ks) // 2):
        return None, estado_centro
    if len(niveles) < max(3, (len(ks) * 2) // 3) or float(np.std(niveles)) > 0.04:
        return None, "caida"                      # precipicio o dudoso: NO es escalera, nunca bajar
    norma = math.sqrt(1.0 + b * b)
    c = float(0.5 * (lat[0] + lat[-1]))
    det = {
        "bx": float(a + b * c),           # punto central del borde (coordenadas del perro)
        "by": c,
        "dist": float(a / norma),
        "ang": float(math.atan2(-b, 1.0)),
        "centro": c,
        "ancho": float(lat[-1] - lat[0] + CARRIL),
        "abierta": bool(k0 == 0 or k1 == len(centros) - 1),
        "prof": float(np.median(prof)),
    }
    return det, estado_centro


def analizar_subida(fx, fy, h):
    """Escalera hacia arriba delante (solo para avisar). Devuelve distancia (m) o None.

    Pide en los tres tercios del pasillo central: suelo, un primer nivel de 8-24 cm que se
    mantiene al menos 10 cm (huella) y al menos otro nivel 8-24 cm mas arriba.
    """
    sel = (np.abs(fy) < 0.20) & (fx > X0) & (fx < 2.0) & (h > -SUELO_TOL) & (h < 0.9)
    fx, fy, h = fx[sel], fy[sel], h[sel]
    if fx.size < 20:
        return None
    dists = []
    for lo in (-0.20, -0.0667, 0.0667):
        m = (fy >= lo) & (fy < lo + 0.1333)
        if np.count_nonzero(m) < 6:
            return None
        bi = ((fx[m] - X0) / PASO).astype(int)
        nb = int((2.0 - X0) / PASO) + 1
        top = np.full(nb, np.nan)
        np.fmax.at(top, bi, h[m])
        niveles, inicio = [], None
        for i in range(nb):
            v = top[i]
            if np.isnan(v):
                continue
            base = niveles[-1][0] if niveles else 0.0
            if abs(v - base) < 0.04:
                if niveles:
                    niveles[-1][1] += 1
                continue
            if 0.08 <= v - base <= 0.24:
                niveles.append([v, 1])
                if inicio is None:
                    inicio = X0 + i * PASO
            elif v > base:
                break
        # el primer escalon con huella visible (>= 10 cm); los siguientes basta con verlos
        if len(niveles) < 2 or niveles[0][1] < 2 or inicio is None:
            return None
        dists.append(inicio)
    if max(dists) - min(dists) > 0.15:
        return None
    return float(np.median(dists))
