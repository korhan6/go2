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


def guardian(xyz, pose3, suelo, cfg, rampa=False):
    """holguras + mitades del frente con la nube local. pose3 = (x, y, z, yaw).

    Ademas de terreno.holguras agrega:
      alto_delante  distancia (desde el hocico) a algo mas alto que el perfil de una escalera
      pared_izq/der distancia del costado a la pared/baranda mas cercana (cualquier altura)
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
    # paredes a los lados a CUALQUIER altura razonable (en una escalera las paredes y barandas
    # quedan por encima o por debajo de la franja normal): para centrarse en la escalera
    banda = (np.abs(fx) < terreno.LARGO2 + 0.25) & (h > -0.25) & (h < 1.2)
    li = banda & (fy > terreno.ANCHO2)
    ld = banda & (fy < -terreno.ANCHO2)
    holg["pared_izq"] = float(np.min(fy[li]) - terreno.ANCHO2) if np.any(li) else math.inf
    holg["pared_der"] = float(np.min(-fy[ld]) - terreno.ANCHO2) if np.any(ld) else math.inf
    # esquinas delanteras: algo pegado a la esquina (entrando en diagonal a una pared)
    obst = (h > cfg["obst_min"]) & (h < cfg["obst_max"])
    zona = (fx > terreno.LARGO2 - 0.12) & (fx < terreno.LARGO2 + 0.10) & obst
    holg["esquina_izq"] = bool(np.count_nonzero(zona & (fy > terreno.ANCHO2 - 0.03) & (fy < terreno.ANCHO2 + 0.07)) >= 2)
    holg["esquina_der"] = bool(np.count_nonzero(zona & (fy < -terreno.ANCHO2 + 0.03) & (fy > -terreno.ANCHO2 - 0.07)) >= 2)
    return holg, mit
