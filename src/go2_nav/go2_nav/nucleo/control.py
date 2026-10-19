"""Seguidor de ruta (persecucion pura) + guardian anti-choques y anti-caidas.

A bordo la latencia es de milisegundos, asi que el control es directo: gira en el sitio si
el rumbo esta muy mal, y en recta acelera hasta v_max frenando segun lo que ve delante.
"""
import math

import numpy as np

from . import terreno


def ang(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Memoria(object):
    """Nube local: junta las nubes de los ultimos segundos (voxeles de 5 cm, radio 3.5 m)."""
    VOX, OFF = 0.05, 1 << 20

    def __init__(self, segundos=1.5, radio=3.5):
        self.seg, self.radio = segundos, radio
        self.k = np.zeros(0, np.int64)
        self.t = np.zeros(0, np.float64)

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
        c = getattr(self, "_c", None)
        return c if c is not None and len(c) == len(self.k) else self._centros(self.k)

    def limpiar(self):
        self.k = np.zeros(0, np.int64)
        self.t = np.zeros(0, np.float64)
        self._c = None


class Seguidor(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.camino = None
        self.k = 0
        self.girando = False
        self.motivo = ""

    def _abrirse(self, holg, err):
        """No cabe girando en el sitio: se aleja de lo que tiene pegado (de lado o hacia atras)
        hasta que el circulo de giro quede libre. Nunca hacia una caida."""
        ops = []
        for lado, vx, vy in (("izq", 0.0, 0.14), ("der", 0.0, -0.14), ("atras", -0.12, 0.0)):
            d_ob, d_ca, _ = holg[lado]
            if d_ca < 0.45:
                continue
            d = min(d_ob, 1.0)
            # a igualdad, preferir el lado hacia el que quiere girar
            if (lado == "izq" and err > 0) or (lado == "der" and err < 0):
                d += 0.05
            ops.append((d, lado, vx, vy))
        ops.sort(reverse=True)
        if ops and ops[0][0] > 0.22:
            _, lado, vx, vy = ops[0]
            self.motivo = "sin espacio para girar: me abro hacia %s" % lado
            return vx, vy, 0.0, False
        self.motivo = "sin espacio para girar"
        return 0.0, 0.0, 0.0, False

    def poner(self, camino):
        self.camino = list(camino) if camino else None
        self.k = 0
        self.girando = False

    def restante(self, x, y):
        if not self.camino:
            return 0.0
        return math.hypot(self.camino[-1][0] - x, self.camino[-1][1] - y)

    def _objetivo(self, x, y, L):
        c = self.camino
        # punto mas cercano (solo hacia adelante, ventana de 2 m)
        fin = min(len(c), self.k + 12)
        d = [math.hypot(c[i][0] - x, c[i][1] - y) for i in range(self.k, fin)]
        if d:
            self.k += int(np.argmin(d))
        acc = 0.0
        for i in range(self.k, len(c) - 1):
            acc += math.hypot(c[i + 1][0] - c[i][0], c[i + 1][1] - c[i][1])
            if acc >= L:
                return c[i + 1]
        return c[-1]

    def paso(self, pose, holg, v_max, mitades=None):
        """Devuelve (vx, vy, wz, llegado). pose = (x, y, yaw) en el marco del mapa."""
        cfg = self.cfg
        x, y, yaw = pose
        if not self.camino:
            return 0.0, 0.0, 0.0, True
        if self.restante(x, y) < 0.25:
            return 0.0, 0.0, 0.0, True
        tx, ty = self._objetivo(x, y, cfg["lookahead"])
        err = ang(math.atan2(ty - y, tx - x) - yaw)
        lim = math.radians(cfg["girar_si"])
        if abs(err) > lim:
            self.girando = True
        elif abs(err) < math.radians(12):
            self.girando = False
        d_ob, d_ca, d_hu = holg["adelante"] if holg else (math.inf, math.inf, math.inf)
        giro_libre = holg["giro"] if holg else math.inf
        self.motivo = ""
        if self.girando:
            if giro_libre < cfg["radio_giro"]:
                return self._abrirse(holg, err)
            w = math.copysign(max(cfg["w_min"], min(cfg["w_max"], 1.6 * abs(err))), err)
            return 0.0, 0.0, w, False
        # velocidad segun lo que hay delante
        v = v_max * (1.0 - 0.55 * abs(err) / lim)
        frenar, parar = cfg["frenar_desde"], cfg["parar_a"]
        if d_ob < frenar:
            v *= max(0.0, min(1.0, (d_ob - parar) / (frenar - parar)))
            v = (max(v, 0.08) if d_ob > parar + 0.10 else min(v, 0.05)) if d_ob > parar else 0.0
            if d_ob <= parar:
                self.motivo = "obstaculo delante"
        if d_ca < 0.35:
            v = 0.0
            self.motivo = "caida delante"
        elif d_ca < 0.8:
            v = min(v, 0.15)
        if d_hu < 0.6:
            v = min(v, 0.18)
        # esquina delantera rozando algo: no avanzar hacia ese lado; girar para alejarla
        if holg and (holg.get("esquina_izq") or holg.get("esquina_der")):
            if holg.get("esquina_izq") and holg.get("esquina_der"):
                v = min(v, 0.0 if d_ob < 0.3 else 0.12)
            elif (holg.get("esquina_izq") and err > -0.05) or (holg.get("esquina_der") and err < 0.05):
                self.motivo = "esquina rozando: giro para alejarme"
                w = -0.7 if holg.get("esquina_izq") else 0.7
                if giro_libre >= cfg["radio_giro"]:
                    return 0.0, 0.0, w, False
                return 0.0, 0.0, 0.0, False
        # al final de la ruta, frena suave
        v = min(v, 0.25 + 0.8 * self.restante(x, y))
        Lr = max(0.3, math.hypot(tx - x, ty - y))
        w = 2.0 * v * math.sin(err) / Lr if v > 0.05 else 1.2 * err
        if v <= 0.05 and abs(w) > 0.0:
            w = math.copysign(max(cfg["w_min"], min(cfg["w_max"], abs(w))), w) if abs(err) > 0.15 else 0.0
        w = max(-cfg["w_max"], min(cfg["w_max"], w))
        vy = 0.0
        # marco de puerta: si solo un lado del carril de delante esta tapado, deslizar al otro
        if mitades is not None and v < 0.25 and d_ca > 0.5:
            izq, der = mitades
            if izq < 0.35 and der > 0.6:
                vy = -0.15
            elif der < 0.35 and izq > 0.6:
                vy = 0.15
            if vy and holg:
                lado = "der" if vy < 0 else "izq"
                if holg[lado][0] < 0.08 or holg[lado][1] < 0.3:
                    vy = 0.0
            if vy and v == 0.0:
                v = 0.06
        return v, vy, w, False


def guardian(xyz, pose3, suelo, cfg, rampa=False):
    """holguras + mitades del frente con la nube local. pose3 = (x, y, z, yaw)."""
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
