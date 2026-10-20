"""Nube local con memoria corta + guardian anti-choques y anti-caidas.

Lo usa go2_escaleras: junta las ultimas nubes (voxeles de 5 cm) y mide el espacio libre
alrededor del cuerpo en cada paso de la maniobra.
"""
import math

import numpy as np

from . import terreno


def ang(a):
    """Angulo normalizado a (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Memoria(object):
    """Nube local: junta las nubes de los ultimos segundos (voxeles de 5 cm, radio 3.5 m)."""
    VOX, OFF = 0.05, 1 << 20

    def __init__(self, segundos=1.5, radio=3.5):
        self.seg, self.radio = segundos, radio
        self.k = np.zeros(0, np.int64)
        self.t = np.zeros(0, np.float64)
        self._c = None

    def sumar(self, xyz, x, y, ahora):
        # lo viejo (expirado) se descarta antes de ordenar: menos trabajo
        vivo = self.t > ahora - self.seg
        kk, tt = self.k[vivo], self.t[vivo]
        if len(xyz):
            k = np.floor(xyz / self.VOX).astype(np.int64) + self.OFF
            keys = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
            kk = np.concatenate([kk, keys])
            tt = np.concatenate([tt, np.full(len(keys), ahora)])
        if len(kk):
            # orden estable: entre claves iguales queda al final la mas nueva (radix sort, rapido)
            o = np.argsort(kk, kind="stable")
            kk, tt = kk[o], tt[o]
            ult = np.r_[kk[1:] != kk[:-1], True]
            kk, tt = kk[ult], tt[ult]
        c = self._centros(kk)
        ok = (np.abs(c[:, 0] - x) < self.radio) & (np.abs(c[:, 1] - y) < self.radio)
        self.k, self.t, self._c = kk[ok], tt[ok], c[ok]

    def _centros(self, kk):
        ijk = np.empty((len(kk), 3), np.int64)
        ijk[:, 0] = (kk >> 42) & 0x1FFFFF
        ijk[:, 1] = (kk >> 21) & 0x1FFFFF
        ijk[:, 2] = kk & 0x1FFFFF
        ijk -= self.OFF
        return ((ijk + 0.5) * self.VOX).astype(np.float32)

    def puntos(self):
        c = self._c
        return c if c is not None and len(c) == len(self.k) else self._centros(self.k)

    def limpiar(self):
        self.k = np.zeros(0, np.int64)
        self.t = np.zeros(0, np.float64)
        self._c = None


ANG_PARED = np.radians(np.arange(-20.0, 20.5, 1.0))


def paredes_laterales(fx, fy, h, celda=0.05, min_celdas=6):
    """Distancia del costado izquierdo y derecho del perro a la pared/baranda, medida a la altura del
    centro del cuerpo (inf = no hay), para centrarse en la escalera.

    Pared = una RECTA (hasta 20 grados respecto del perro) de celdas VERTICALES (en la misma celda
    de 5 cm hay alturas que cubren >= 10 cm) de >= 30 cm de largo. Antes contaba cualquier punto al
    costado: encima de la escalera las huellas y contrahuellas de al lado eran "pared a 0 cm" de los
    dos lados, el perro no se centraba y se iba corriendo contra la pared de verdad. Una
    contrahuella es vertical pero cruza: no forma una recta a lo largo del perro.
    (El angulo de la recta no se usa para el rumbo: con celdas de 5 cm sale con +-10 grados de ruido.)"""
    sel = ((fx > -terreno.LARGO2 - 0.25) & (fx < 1.2) & (np.abs(fy) > terreno.ANCHO2) & (np.abs(fy) < 0.8)
           & (h > -0.45) & (h < 1.2))
    if np.count_nonzero(sel) < min_celdas:
        return math.inf, math.inf
    ix = np.floor(fx[sel] / celda).astype(np.int64)
    iy = np.floor(fy[sel] / celda).astype(np.int64)
    col = (ix << 20) + (iy + (1 << 19))
    u, inv = np.unique(col, return_inverse=True)
    alto = np.full(len(u), -np.inf)
    bajo = np.full(len(u), np.inf)
    np.maximum.at(alto, inv, h[sel])
    np.minimum.at(bajo, inv, h[sel])
    vert = (alto - bajo) >= 0.10
    cx = ((u[vert] >> 20) + 0.5) * celda
    cy = (((u[vert] & ((1 << 20) - 1)) - (1 << 19)) + 0.5) * celda
    out = []
    for lado in (1, -1):
        L = cy * lado > terreno.ANCHO2
        X, Y = cx[L], cy[L] * lado               # Y: hacia afuera de ese costado (> 0)
        mejor = None
        if len(X) >= min_celdas:
            for th in ANG_PARED:
                yp = Y - lado * X * math.tan(th)     # distancia a la recta de angulo th (en fx = 0)
                b = np.floor(yp / celda).astype(np.int64)
                b0 = b.min()
                n = np.bincount(b - b0)
                n2 = n + np.r_[n[1:], 0]             # 2 franjas: una recta en celdas de 5 cm cae en 2
                k = int(np.argmax(n2))
                if mejor is None or n2[k] > mejor[0]:
                    en = (b - b0 == k) | (b - b0 == k + 1)
                    mejor = (int(n2[k]), float(np.mean(yp[en])), float(np.ptp(X[en])))
        ok = mejor is not None and mejor[0] >= min_celdas and mejor[2] >= 0.30
        out.append(max(0.0, mejor[1] - terreno.ANCHO2) if ok else math.inf)
    return out[0], out[1]


def guardian(xyz, pose3, suelo, cfg, rampa=False):
    """holguras + mitades del frente con la nube local. pose3 = (x, y, z, yaw).

    Ademas de terreno.holguras agrega:
      alto_delante  distancia (desde el hocico) a algo mas alto que el perfil de una escalera
      pared_izq/der distancia del costado a la pared/baranda (ver paredes_laterales)
      esquina_izq/der  True si algo roza la esquina delantera de ese lado
    """
    if xyz is None or len(xyz) == 0:
        return None, None
    x, y, z, yaw = pose3
    fx, fy = terreno.a_cuerpo(xyz, x, y, yaw)
    h = xyz[:, 2] - suelo
    holg = terreno.holguras(fx, fy, h, obst_min=cfg["obst_min"], rampa=rampa)
    mit = terreno.mitades_frente(fx, fy, h, obst_min=cfg["obst_min"])
    # algo alto (persona, pared) delante, por encima del perfil de una escalera: para la subida
    a = fx - terreno.LARGO2
    m = (a > 0.0) & (a < 0.8) & (np.abs(fy) < terreno.ANCHO2 + 0.05) & (h > 0.55 + 0.5 * a)
    holg["alto_delante"] = float(np.min(a[m])) if np.any(m) else math.inf
    holg["pared_izq"], holg["pared_der"] = paredes_laterales(fx, fy, h)
    # esquinas delanteras: algo pegado a la esquina (entrando en diagonal a una pared)
    obst = (h > cfg["obst_min"]) & (h < cfg["obst_max"])
    zona = (fx > terreno.LARGO2 - 0.12) & (fx < terreno.LARGO2 + 0.10) & obst
    holg["esquina_izq"] = bool(np.count_nonzero(zona & (fy > terreno.ANCHO2 - 0.03) & (fy < terreno.ANCHO2 + 0.07)) >= 2)
    holg["esquina_der"] = bool(np.count_nonzero(zona & (fy < -terreno.ANCHO2 + 0.03) & (fy > -terreno.ANCHO2 - 0.07)) >= 2)
    return holg, mit
