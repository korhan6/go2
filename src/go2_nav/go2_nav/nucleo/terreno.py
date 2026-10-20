"""
terreno.py - Analiza el terreno cerca del perro con la nube 3D del lidar L1.

Lo usan:
  control.py   guardian anti-choques (obstaculos y caidas en las 4 direcciones)
  escaleras.py deteccion de escaleras de bajada (borde, ancho, angulo) y de subida
  limpiador.py medidas del cuerpo

Geometria que importa para las BAJADAS: el L1 va en la cabeza, a ~0.3 m del suelo. Un escalon
de 17 cm queda en su sombra durante 0.57 x (distancia al borde): con el hocico a 0.7 m del borde
no se ve NADA del primer escalon; a 0.3 m se ven ~10 cm de huella. Y pegado al borde ya no ve
el suelo de delante (hace falta verlo para saber donde esta el borde). Por eso la bajada solo
se confirma con el hocico a ~0.25-0.55 m del borde (medido con pruebas/prueba_nucleo.py), y el
perro se acerca despacio hasta ahi (los pies van 17 cm detras del hocico).

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
RES_ANCHO = 0.05       # m: el ancho de una escalera se mide en franjas de 10 cm (+-5 cm)




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
HUELLA_MIN, HUELLA_MAX = 0.09, 0.30    # m bajo el borde donde tiene que verse el PRIMER escalon
XMAX_SUBIDA = 2.2


class Rejilla(object):
    """Rejilla carriles (10 cm, de lado) x franjas (5 cm, hacia adelante) delante del perro.

    piso   hay suelo en la celda          bajo   hay algo por debajo del suelo
    alto   hay un obstaculo               hmin   lo mas bajo visto (bajo el suelo)
    hmaxb  nivel mas alto bajo el suelo, solo si es una superficie (>= 2 voxeles juntos)
    """

    def __init__(self, fx, fy, h):
        nl = int(round(2 * LAT / CARRIL))
        nb = int(round((X1 - X0) / PASO))
        sel = (fx >= X0) & (fx < X1) & (np.abs(fy) < LAT) & (h < OBST_MAX)
        fx, fy, h = fx[sel], fy[sel], h[sel]
        li = ((fy + LAT) / CARRIL).astype(int).clip(0, nl - 1)
        bi = ((fx - X0) / PASO).astype(int).clip(0, nb - 1)
        self.piso = np.zeros((nl, nb), bool)
        self.bajo = np.zeros((nl, nb), bool)
        self.alto = np.zeros((nl, nb), bool)
        self.hmin = np.full((nl, nb), np.nan)
        self.hmaxb = np.full((nl, nb), np.nan)
        m = np.abs(h) < SUELO_TOL
        self.piso[li[m], bi[m]] = True
        m = h < -CAIDA_MIN
        self.bajo[li[m], bi[m]] = True
        if m.any():
            tmp = np.full((nl, nb), np.inf)
            np.minimum.at(tmp, (li[m], bi[m]), h[m])
            self.hmin = np.where(np.isfinite(tmp), tmp, np.nan)
            tmp = np.full((nl, nb), -np.inf)
            np.maximum.at(tmp, (li[m], bi[m]), h[m])
            cerca_max = m.copy()
            cerca_max[m] = h[m] >= tmp[li[m], bi[m]] - 0.04
            cnt = np.zeros((nl, nb), np.int32)
            np.add.at(cnt, (li[cerca_max], bi[cerca_max]), 1)
            self.hmaxb = np.where(np.isfinite(tmp) & (cnt >= 2), tmp, np.nan)
        m = h > OBST_MIN
        self.alto[li[m], bi[m]] = True
        self.centros = -LAT + (np.arange(nl) + 0.5) * CARRIL

    def carril(self, k):
        """('caida', franja_borde) | ('pared', i) | ('suelo', None) | ('nada', None | i)."""
        piso, bajo, alto = self.piso[k], self.bajo[k], self.alto[k]
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
                if bajo[i:min(nb, i + 24)].any():      # se ve algo mas abajo en los siguientes 1.2 m
                    return ("caida", i)
                # hueco sin nada debajo: borde dudoso (alfombra negra, vidrio, fuera de alcance)
                return ("nada", i)
            i += 1
        return ("suelo", None) if vio_suelo else ("nada", None)

    def huella(self, k, i):
        """Nivel del primer escalon bajo el borde (franja i) del carril k, o None.

        SEGURIDAD: una escalera tiene un PRIMER ESCALON plano (huella) a 9-30 cm bajo el borde,
        visible en al menos 2 franjas seguidas. Un balcon, un entrepiso o un hueco solo muestran
        el fondo (muy abajo); el ruido o los rebotes del lidar no forman una huella plana y
        pareja en varios carriles."""
        w = self.hmaxb[k, i:i + 10]
        for q in range(len(w) - 1):
            if (np.isfinite(w[q]) and np.isfinite(w[q + 1]) and -HUELLA_MAX <= w[q] <= -HUELLA_MIN
                    and abs(w[q] - w[q + 1]) < 0.04):
                return 0.5 * float(w[q] + w[q + 1])
        return None


def analizar_bajada(fx, fy, h):
    """Busca una escalera (o escalon) hacia abajo delante del perro.

    Devuelve (deteccion | None, estado_centro, motivo).
      deteccion: dict(bx, by, dist, ang, centro, ancho, prof, abierta, huella)
        huella True  = se ve el primer escalon: ES una escalera de bajada
               False = caida empinada de borde recto sin huella vista: POSIBLE bajada (o balcon)
        bx, by punto central del borde (m, coordenadas del perro)
        dist   distancia perpendicular del centro del perro al borde (m)
        ang    angulo de la normal del borde respecto al frente del perro (rad, + = izquierda)
        centro desplazamiento lateral del centro de la escalera (m, + = izquierda)
        ancho  ancho visto de la bajada (m); abierta = llega al limite de lo que se mira
        prof   lo mas bajo que se ve despues del borde (m, negativo)
      estado_centro: 'caida' | 'suelo' | 'pared' | 'nada' (lo que hay justo delante)
      motivo: por que NO es (o si es) una escalera, en palabras (para 'go2 escaleras')
    """
    rj = Rejilla(fx, fy, h)
    estados = [rj.carril(k) for k in range(len(rj.centros))]
    medio = [k for k in range(len(rj.centros)) if abs(rj.centros[k]) <= 0.15 + 1e-6]
    tipos = [estados[k][0] for k in medio]
    estado_centro = max(set(tipos), key=tipos.count)
    if any(estados[k][0] != "caida" for k in medio):
        return None, estado_centro, {"suelo": "delante hay suelo", "pared": "algo delante",
                                     "nada": "no veo el suelo delante (o un hueco sin fondo)"}.get(
            estado_centro, "delante: " + estado_centro)
    # ensancha desde el centro mientras los carriles tambien sean bajada
    k0, k1 = min(medio), max(medio)
    while k0 > 0 and estados[k0 - 1][0] == "caida":
        k0 -= 1
    while k1 < len(rj.centros) - 1 and estados[k1 + 1][0] == "caida":
        k1 += 1
    ks = np.arange(k0, k1 + 1)
    lat = rj.centros[ks]
    borde = np.array([X0 + estados[k][1] * PASO for k in ks])
    b, a = np.polyfit(lat, borde, 1)          # borde(lat) = a + b * lat
    resid = borde - (a + b * lat)
    if float(np.std(resid)) > 0.07 or abs(b) > 1.2:   # no es una linea recta: sombra, objetos
        return None, estado_centro, "el borde de la caida no es recto (sombra u objetos)"
    prof, niveles = [], []
    empinados = 0
    for k in ks:
        i = estados[k][1]
        v = rj.hmin[k, i:i + 24]
        d = (np.arange(v.size) + 1) * PASO
        ok = np.isfinite(v)
        if ok.any():
            prof.append(float(v[ok].min()))
            # una escalera cae empinado (~30 grados o mas, y con la sombra parece mas);
            # una rampa baja suave: menos de 0.40 m por metro no es escalera
            if np.any(ok & (v <= -0.12) & (-v / d >= RAMPA_MAX)):
                empinados += 1
        n = rj.huella(k, i)
        if n is not None:
            niveles.append(n)
    if len(prof) < 2 or empinados < max(2, len(ks) // 2):
        return None, estado_centro, "baja suave: rampa o desnivel chico, no escalera"
    # SEGURIDAD: sin la huella del primer escalon NO es escalera (precipicio o dudoso). Igual se
    # devuelve la geometria (huella=False): es una POSIBLE bajada, que el gestor va a mirar de
    # cerca; la maniobra solo arranca con huella=True.
    huella = len(niveles) >= max(3, (len(ks) * 2) // 3) and float(np.std(niveles)) <= 0.04
    norma = math.sqrt(1.0 + b * b)
    c = float(0.5 * (lat[0] + lat[-1]))
    det = {
        "bx": float(a + b * c),           # punto central del borde (coordenadas del perro)
        "by": c,
        "dist": float(a / norma),
        "ang": float(math.atan2(-b, 1.0)),
        "centro": c,
        "ancho": float(lat[-1] - lat[0] + CARRIL),
        "abierta": bool(k0 == 0 or k1 == len(rj.centros) - 1),
        "prof": float(np.median(prof)),
        "huella": bool(huella),
    }
    if not huella:
        return det, "caida", ("caida a %.2f m pero NO veo el primer escalon (huella en %d de %d carriles): "
                              "balcon/desnivel, o el escalon queda en la sombra del lidar (acercarse)"
                              % (float(a) - LARGO2, len(niveles), len(ks)))
    return det, estado_centro, "escalera de bajada a %.2f m (ancho %.2f m)" % (det["dist"], det["ancho"])


def analizar_subida(fx, fy, h):
    """Escalera de subida delante: dict(bx, by, dist, ang, ancho, abierta) o None.

    En carriles de 10 cm (|lateral| < 0.45 m) busca: suelo, un nivel 8-26 cm mas alto que se
    mantiene (huella) y otro nivel encima. El inicio de cada carril se ajusta a una recta.
    """
    sel = (np.abs(fy) < 0.45) & (fx > X0) & (fx < XMAX_SUBIDA) & (h > -SUELO_TOL) & (h < 1.0)
    fx, fy, h = fx[sel], fy[sel], h[sel]
    if fx.size < 30:
        return None
    lats, inicios = [], []
    nb = int((XMAX_SUBIDA - X0) / PASO) + 1
    for lo in np.arange(-0.45, 0.45, 0.10):
        m = (fy >= lo) & (fy < lo + 0.10)
        if np.count_nonzero(m) < 6:
            continue
        bi = ((fx[m] - X0) / PASO).astype(int).clip(0, nb - 1)
        top = np.full(nb, np.nan)
        np.fmax.at(top, bi, h[m])
        niveles, inicio, piso = [], None, None
        for i in range(nb):
            v = top[i]
            if np.isnan(v):
                continue
            if piso is None:                 # el suelo delante (con su ruido de +-5 cm)
                if abs(v) > 0.08:
                    break
                piso = v
            base = niveles[-1][0] if niveles else piso
            if abs(v - base) <= 0.055:
                if niveles:
                    niveles[-1][1] += 1
                continue
            if 0.08 <= v - base <= 0.26:
                niveles.append([v, 1])
                if inicio is None:
                    inicio = X0 + i * PASO
            elif v > base:
                break
        if len(niveles) >= 2 and niveles[0][1] >= 2 and inicio is not None:
            lats.append(lo + 0.05)
            inicios.append(inicio)
    if len(lats) < 4:
        return None
    lats, inicios = np.array(lats), np.array(inicios)
    if np.count_nonzero(np.abs(lats) <= 0.2) < 3:
        return None
    b, a = np.polyfit(lats, inicios, 1)
    if float(np.std(inicios - (a + b * lats))) > 0.08 or abs(b) > 1.2:
        return None
    c = float(0.5 * (lats[0] + lats[-1]))
    norma = math.sqrt(1.0 + b * b)
    return {"bx": float(a + b * c), "by": c, "dist": float(a / norma),
            "ang": float(math.atan2(-b, 1.0)), "ancho": float(lats[-1] - lats[0] + 0.10),
            "abierta": bool(abs(lats[0]) > 0.39 or abs(lats[-1]) > 0.39)}


# ---------------------------------------------------------------- escaneo 2D para el SLAM
def rangos_scan(pts, x, y, yaw, n=360, r_min=0.20, r_max=6.0, hueco_max=3):
    """Obstaculos (odom) -> rangos de un LaserScan de n haces alrededor del perro (inf = nada).

    El L1 es un lidar 3D de pocos puntos: en la franja de alturas de los obstaculos, una pared
    a 3 m recibe pocos puntos por nube, y muchos haces quedaban vacios. slam_toolbox solo marca
    suelo libre a lo largo de los haces CON lectura: el mapa salia como una estrella de rayos
    sueltos con puntitos negros, en vez de un abanico lleno con paredes continuas. Por eso:
      - haces de 1 grado (a 4 m, 7 cm: como la celda del mapa), no de 0.5;
      - un haz vacio entre dos con lectura parecida (la misma pared, a <= hueco_max haces) se
        rellena interpolando: la pared queda continua y el suelo delante, libre;
      - nada mas alla de r_max (el SLAM debe tener max_laser_range >= r_max: si no, cada lectura
        entre los dos se vuelve un rayo libre de max_laser_range sin pared al final).
    """
    rangos = np.full(n, np.inf, np.float32)
    if len(pts):
        fx, fy = a_cuerpo(pts, x, y, yaw)
        r = np.hypot(fx, fy)
        a = np.floor((np.arctan2(fy, fx) + math.pi) / (2 * math.pi / n)).astype(np.int64) % n
        ok = (r > r_min) & (r < r_max)
        np.minimum.at(rangos, a[ok], r[ok].astype(np.float32))
    if hueco_max > 0:
        rangos = _rellenar_huecos(rangos, hueco_max)
    return rangos


def _rellenar_huecos(rangos, hueco_max):
    n = len(rangos)
    llenos = np.nonzero(np.isfinite(rangos))[0]
    if len(llenos) < 2:
        return rangos
    out = rangos.copy()
    sig = np.r_[llenos[1:], llenos[0] + n]          # el siguiente haz con lectura (circular)
    for i0, i1 in zip(llenos, sig):
        hueco = i1 - i0 - 1
        if hueco < 1 or hueco > hueco_max:
            continue
        r0, r1 = float(rangos[i0]), float(rangos[i1 % n])
        if abs(r0 - r1) > max(0.10, 0.04 * min(r0, r1)):
            continue                                 # bordes distintos: un hueco de verdad (puerta)
        for k in range(1, hueco + 1):
            out[(i0 + k) % n] = r0 + (r1 - r0) * k / (hueco + 1)
    return out


# ---------------------------------------------------------------- caidas para el mapa de costos
def caidas_rejilla(bajo, piso, x, y, R=2.5, C=0.10, min_pts=4, sombra_max=0.7, cerca_cuerpo=0.45):
    """Celdas de CAIDA (escalon hacia abajo, hueco) alrededor del perro, para marcarlas como
    obstaculo en el mapa de costos.

    bajo  Nx3 (x, y, h) puntos por debajo del suelo (h < 0), de las ultimas nubes (odom)
    piso  Mx2 (x, y) puntos de suelo de las ultimas nubes
    Una celda es caida si tiene >= min_pts puntos bajo el suelo que forman una SUPERFICIE (poca
    dispersion de altura: un escalon, el piso de abajo; no reflejos sueltos) y ningun punto de
    suelo. Ademas, la SOMBRA del borde: el L1 va a ~30 cm del suelo y no ve el primer escalon
    pegado al borde, asi que entre el ultimo suelo visto y la caida queda una franja sin nada.
    Antes esa franja quedaba "libre" y Nav2 llevaba al perro hasta el filo. Ahora se marca: desde
    cada celda de caida hacia el perro, las celdas SIN suelo (hasta sombra_max m) tambien son
    caida, hasta encontrar suelo. Nunca a menos de cerca_cuerpo m del centro del perro.
    Devuelve Kx2 centros de celda (odom).
    """
    n = int(round(2 * R / C))
    vacio = np.zeros((0, 2), np.float32)
    if len(bajo) == 0:
        return vacio

    def idx(px, py):
        i = np.floor((px - x + R) / C).astype(np.int64)
        j = np.floor((py - y + R) / C).astype(np.int64)
        return i, j, (i >= 0) & (i < n) & (j >= 0) & (j < n)

    hay_suelo = np.zeros((n, n), bool)
    if len(piso):
        i, j, ok = idx(piso[:, 0], piso[:, 1])
        hay_suelo[i[ok], j[ok]] = True
    i, j, ok = idx(bajo[:, 0], bajo[:, 1])
    cel = i[ok] * n + j[ok]
    hb = bajo[ok, 2].astype(np.float64)
    cnt = np.bincount(cel, minlength=n * n)
    s1 = np.bincount(cel, weights=hb, minlength=n * n)
    s2 = np.bincount(cel, weights=hb * hb, minlength=n * n)
    with np.errstate(invalid="ignore", divide="ignore"):
        var = s2 / np.maximum(cnt, 1) - (s1 / np.maximum(cnt, 1)) ** 2
    caida = ((cnt >= min_pts) & (var < 0.03 ** 2)).reshape(n, n) & ~hay_suelo
    if not caida.any():
        return vacio
    # sombra del borde: de cada celda de caida hacia el perro, mientras no haya suelo
    ii, jj = np.nonzero(caida)
    cx = (ii + 0.5) * C - R
    cy = (jj + 0.5) * C - R
    d = np.hypot(cx, cy)
    ux, uy = -cx / np.maximum(d, 1e-6), -cy / np.maximum(d, 1e-6)
    vivo = np.ones(len(ii), bool)
    sombra = np.zeros((n, n), bool)
    for s in np.arange(C / 2, sombra_max + 1e-6, C / 2):
        px, py = cx + ux * s, cy + uy * s
        vivo &= (d - s) > cerca_cuerpo
        a = np.floor((px + R) / C).astype(np.int64).clip(0, n - 1)
        b = np.floor((py + R) / C).astype(np.int64).clip(0, n - 1)
        vivo &= ~hay_suelo[a, b]
        if not vivo.any():
            break
        sombra[a[vivo], b[vivo]] = True
    ii, jj = np.nonzero(caida | sombra)
    return np.c_[x - R + (ii + 0.5) * C, y - R + (jj + 0.5) * C].astype(np.float32)
