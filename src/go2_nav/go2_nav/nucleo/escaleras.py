"""Escaleras: deteccion (subida y bajada) y maniobra.

SUBIR (de frente):  ALINEAR -> CENTRAR -> ACERCAR -> MARCHA_ON -> SUBIENDO -> SALIR -> MARCHA_OFF
BAJAR (SIEMPRE en reversa):  ALINEAR -> CENTRAR -> DISTANCIA -> GIRAR 180 -> VERIFICAR ->
                             MARCHA_ON -> BAJANDO (de reversa) -> SALIR -> MARCHA_OFF -> GIRAR_FRENTE

Antes de pisar la escalera, cualquier duda = aborta y vuelve a la marcha normal.
Ya encima de los escalones nunca cambia de marcha: si algo falla se queda quieto (DETENIDO)
y espera a 'go2nav seguir' o al control remoto.
Todo en el marco odom del perro (no depende del mapa).
"""
import math

import numpy as np

from . import terreno
from .control import ang

CONFIRMAR_N, CONFIRMAR_T = 3, 4.0
TOL_POS, TOL_ANG = 0.15, math.radians(10)
NIVEL = math.radians(5)
D_GIRO, D_MIN, D_MAX = 0.65, 0.50, 1.20      # bajada: distancia del centro al borde para girar
D_SUBIR = 0.45                               # subida: centro del perro a 45 cm del primer escalon
LIMITE = {"DETENER": 3, "ALINEAR": 15, "MEDIR": 10, "CENTRAR": 10, "DISTANCIA": 12,
          "ACERCAR": 12, "GIRAR": 20, "VERIFICAR": 6, "MARCHA_ON": 10, "BAJANDO": 90,
          "SUBIENDO": 90, "SALIR": 5, "MARCHA_OFF": 10, "GIRAR_FRENTE": 20, "RESTAURAR": 10}


def media_ang(angs):
    return math.atan2(sum(math.sin(a) for a in angs), sum(math.cos(a) for a in angs))


# ------------------------------------------------------------------ deteccion subida
def subida_detalle(fx, fy, h):
    """Escalera de subida delante: dict(dist, ang, ancho, by) o None.

    En carriles de 10 cm (|lateral| < 0.45 m) busca: suelo, un nivel 8-24 cm mas alto que se
    mantiene (huella) y otro nivel encima. El inicio de cada carril se ajusta a una recta.
    """
    X0, PASO, XMAX = terreno.X0, terreno.PASO, 2.2
    sel = (np.abs(fy) < 0.45) & (fx > X0) & (fx < XMAX) & (h > -terreno.SUELO_TOL) & (h < 1.0)
    fx, fy, h = fx[sel], fy[sel], h[sel]
    if fx.size < 30:
        return None
    lats, inicios = [], []
    nb = int((XMAX - X0) / PASO) + 1
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
    centro = np.abs(lats) <= 0.2
    if np.count_nonzero(centro) < 3:
        return None
    b, a = np.polyfit(lats, inicios, 1)
    if float(np.std(inicios - (a + b * lats))) > 0.08 or abs(b) > 1.2:
        return None
    c = float(0.5 * (lats[0] + lats[-1]))
    norma = math.sqrt(1.0 + b * b)
    return {"bx": float(a + b * c), "by": c, "dist": float(a / norma),
            "ang": float(math.atan2(-b, 1.0)), "ancho": float(lats[-1] - lats[0] + 0.10),
            "abierta": bool(abs(lats[0]) > 0.39 or abs(lats[-1]) > 0.39)}


class Detector(object):
    """Confirma escaleras con varias nubes seguidas (en odom)."""

    def __init__(self):
        self.hist = {"bajar": [], "subir": []}
        self.atras = (None, "nada", 0.0)

    def limpiar(self):
        self.hist = {"bajar": [], "subir": []}

    def actualizar(self, xyz, pose, suelo, ahora, alrededor=False):
        """pose = (x, y, z, yaw). Mira delante (y detras, para la verificacion de la bajada).
        Con alrededor=True mira en 8 direcciones (para descubrir escaleras de costado)."""
        x, y, z, yaw = pose
        if len(xyz) == 0:
            return
        cerca = (np.abs(xyz[:, 0] - x) < 2.5) & (np.abs(xyz[:, 1] - y) < 2.5)
        xyz = xyz[cerca]
        h = xyz[:, 2] - suelo
        dirs = [0.0, math.pi] + ([k * math.pi / 4 for k in (1, 2, 3, 5, 6, 7)] if alrededor else [])
        for d in dirs:
            yd = ang(yaw + d)
            fx, fy = terreno.a_cuerpo(xyz, x, y, yd)
            det, centro = terreno.analizar_bajada(fx, fy, h)
            if d == math.pi:
                self.atras = (det, centro, ahora)
            if det is not None and det["dist"] < 2.0:
                self._sumar("bajar", det, x, y, yd, suelo, ahora)
            sub = subida_detalle(fx, fy, h)
            if sub is not None:
                self._sumar("subir", sub, x, y, yd, suelo, ahora)

    def _sumar(self, tipo, d, x, y, yaw, suelo, ahora):
        c, s = math.cos(yaw), math.sin(yaw)
        ex = x + c * d["bx"] - s * d["by"]
        ey = y + s * d["bx"] + c * d["by"]
        hist = self.hist[tipo]
        hist.append({"t": ahora, "ex": ex, "ey": ey, "yaw_n": ang(yaw + d["ang"]),
                     "ancho": d["ancho"], "abierta": d["abierta"], "prof": d.get("prof", 0.0),
                     "z": suelo})
        del hist[:-12]

    def confirmada(self, tipo, ahora, pos=None):
        rec = [d for d in self.hist[tipo] if ahora - d["t"] < CONFIRMAR_T]
        if len(rec) < CONFIRMAR_N:
            return None
        ex = float(np.median([d["ex"] for d in rec]))
        ey = float(np.median([d["ey"] for d in rec]))
        ref = media_ang([d["yaw_n"] for d in rec])
        ok = [d for d in rec if math.hypot(d["ex"] - ex, d["ey"] - ey) < TOL_POS
              and abs(ang(d["yaw_n"] - ref)) < TOL_ANG]
        if len(ok) < CONFIRMAR_N or ok[-1]["t"] - ok[0]["t"] < 0.25:
            return None
        c = {"tipo": tipo,
             "E": np.array([np.median([d["ex"] for d in ok]), np.median([d["ey"] for d in ok])]),
             "yaw_n": media_ang([d["yaw_n"] for d in ok]),
             "ancho": float(np.median([d["ancho"] for d in ok])),
             "abierta": any(d["abierta"] for d in ok),
             "prof": float(np.median([d["prof"] for d in ok])),
             "z": float(np.median([d["z"] for d in ok]))}
        if pos is not None:
            n = np.array([math.cos(c["yaw_n"]), math.sin(c["yaw_n"])])
            c["dist"] = float(np.dot(c["E"] - np.asarray(pos), n))
        return c


class Maniobra(object):
    """Maquina de estados de una subida o bajada. paso() devuelve (vx, vy, wz)."""

    def __init__(self, robot, cfg, log):
        self.robot, self.cfg, self.log = robot, cfg, log
        self.estado = "INACTIVA"
        self.t_estado = 0.0
        self.m = {}
        self.resultado = None       # dict al terminar (ok, motivo, dz, ...)
        self.info = ""

    @property
    def activa(self):
        return self.estado not in ("INACTIVA", "DETENIDO")

    def en_escalera(self, pitch=0.0):
        if self.estado not in ("SUBIENDO", "BAJANDO", "SALIR", "DETENIDO"):
            return False
        return bool(self.m.get("piso_cambio")) or abs(pitch) > math.radians(6)

    def empezar(self, tipo, c, ahora):
        self.m = {"tipo": tipo, "E": c["E"], "yaw_n": c["yaw_n"], "ancho": c["ancho"],
                  "abierta": c["abierta"], "z_borde": c["z"], "marcha": False}
        self.resultado = None
        self.log("ESCALERA de %s a %.2f m (ancho %.2f m%s): empiezo"
                 % ("SUBIDA" if tipo == "subir" else "BAJADA", c.get("dist", 0.0), c["ancho"],
                    "+" if c["abierta"] else ""))
        self._ir("DETENER", ahora)

    def _ir(self, e, ahora):
        if e != self.estado:
            self.log("escalera: %s -> %s" % (self.estado, e))
        self.estado, self.t_estado = e, ahora

    def _fin(self, ok, motivo, ahora, **extra):
        self.resultado = dict(ok=ok, motivo=motivo, tipo=self.m.get("tipo"), **extra)
        (self.log if ok else self.log)("escalera %s: %s" % ("LISTA" if ok else "cancelada", motivo))
        self._ir("INACTIVA", ahora)

    def abortar(self, motivo, ahora):
        if self.m.get("marcha"):
            self.m["motivo_abortar"] = motivo
            self.m["espera"] = self.robot.modo_escaleras(False)
            self.m["marcha"] = False
            self._ir("RESTAURAR", ahora)
            return
        self._fin(False, motivo, ahora)

    def detener(self, motivo, ahora):
        self.log("DETENIDO EN LA ESCALERA: %s. Usa 'go2nav seguir' o el control remoto." % motivo)
        self.m["motivo_det"] = motivo
        self._ir("DETENIDO", ahora)

    def seguir(self, ahora):
        """Retomar despues de DETENIDO (el usuario lo pidio)."""
        if self.estado == "DETENIDO":
            self.m["P0"] = None
            self._ir("BAJANDO" if self.m["tipo"] == "bajar" else "SUBIENDO", ahora)

    def _girar(self, err):
        return 0.0, 0.0, math.copysign(max(self.cfg["w_min"], min(0.7, 1.5 * abs(err))), err)

    # ---------------------------------------------------------------- un paso
    def paso(self, ahora, pose, det, holg):
        """pose = (x, y, z, yaw, pitch, roll) odom. det = Detector."""
        x, y, z, yaw, pitch, roll = pose
        P = np.array([x, y])
        e, dt = self.estado, ahora - self.t_estado
        tipo = self.m.get("tipo")
        if e in ("INACTIVA", "DETENIDO"):
            return 0.0, 0.0, 0.0
        if dt > LIMITE.get(e, 1e9):
            if e == "MARCHA_OFF":
                self._ir("GIRAR_FRENTE" if tipo == "bajar" else "LISTO", ahora)
            elif e == "GIRAR_FRENTE":
                self._ir("LISTO", ahora)
            elif e == "RESTAURAR":
                self._fin(False, self.m.get("motivo_abortar", "cancelada"), ahora)
            elif self.en_escalera(pitch):
                self.detener("%s tardo demasiado" % e, ahora)
            else:
                self.abortar("%s tardo demasiado" % e, ahora)
            return 0.0, 0.0, 0.0

        if e == "DETENER":
            if dt > 0.8:
                c = det.confirmada(tipo, ahora, P)
                if c is None:
                    self.abortar("perdi de vista la escalera", ahora)
                elif c["ancho"] < self.cfg["ancho_min_escalera"] and not c["abierta"]:
                    self.abortar("escalera muy angosta (%.2f m)" % c["ancho"], ahora)
                else:
                    self.m.update(E=c["E"], yaw_n=c["yaw_n"])
                    self._ir("ALINEAR", ahora)
            return 0.0, 0.0, 0.0

        if e == "ALINEAR":
            err = ang(self.m["yaw_n"] - yaw)
            if abs(err) < math.radians(3):
                self.m["P_medir"] = P.copy()
                self._ir("MEDIR", ahora)
                return 0.0, 0.0, 0.0
            return self._girar(err)

        if e == "MEDIR":
            if dt < 1.0:
                return 0.0, 0.0, 0.0
            c = det.confirmada(tipo, ahora, P)
            if c is None:
                if dt > 8.0:
                    self.abortar("de frente ya no veo la escalera", ahora)
                    return 0.0, 0.0, 0.0
                # muy pegado al borde la cabeza tapa el primer escalon: retrocede un poco
                if (tipo == "bajar" and dt > 2.5 and np.linalg.norm(P - self.m["P_medir"]) < 0.30
                        and holg and holg["atras"][0] > 0.25 and holg["atras"][1] > 0.4):
                    return -0.06, 0.0, 0.0
                return 0.0, 0.0, 0.0
            n = np.array([math.cos(c["yaw_n"]), math.sin(c["yaw_n"])])
            t = np.array([-n[1], n[0]])
            lat = float(np.dot(c["E"] - P, t))
            self.m.update(E=c["E"], yaw_n=c["yaw_n"], n=n, t=t, z_borde=c["z"],
                          meta_lat=P + lat * t, dist=c["dist"])
            self.log("borde a %.2f m, centro de la escalera a %+.2f m" % (c["dist"], lat))
            self._ir("CENTRAR", ahora)
            return 0.0, 0.0, 0.0

        if e == "CENTRAR":
            t_rob = np.array([-math.sin(yaw), math.cos(yaw)])
            err = float(np.dot(self.m["meta_lat"] - P, t_rob))
            w = max(-0.3, min(0.3, 1.2 * ang(self.m["yaw_n"] - yaw)))
            if abs(err) < 0.04 or (holg and holg["izq" if err > 0 else "der"][0] < 0.06):
                d = float(np.dot(self.m["E"] - P, self.m["n"]))
                if tipo == "bajar":
                    self.m["d_obj"] = None if D_MIN <= d <= D_MAX else D_GIRO
                    self._ir("DISTANCIA", ahora)
                else:
                    self._ir("ACERCAR", ahora)
                return 0.0, 0.0, 0.0
            return 0.0, math.copysign(max(0.06, min(0.15, 1.5 * abs(err))), err), w

        if e == "DISTANCIA":          # solo bajada
            d = float(np.dot(self.m["E"] - P, self.m["n"]))
            w = max(-0.3, min(0.3, 1.2 * ang(self.m["yaw_n"] - yaw)))
            if self.m["d_obj"] is None or abs(d - self.m["d_obj"]) < 0.05:
                self.m["dist"] = d
                self.m["yaw_b"] = ang(self.m["yaw_n"] + math.pi)
                self._ir("GIRAR", ahora)
                return 0.0, 0.0, 0.0
            err = d - D_GIRO
            return math.copysign(max(0.05, min(0.12, abs(err))), err), 0.0, w

        if e == "ACERCAR":            # solo subida
            d = float(np.dot(self.m["E"] - P, self.m["n"]))
            w = max(-0.3, min(0.3, 1.2 * ang(self.m["yaw_n"] - yaw)))
            if abs(d - D_SUBIR) < 0.05 or d < D_SUBIR:
                self._marcha_on(ahora)
                return 0.0, 0.0, 0.0
            if holg and holg["adelante"][0] < 0.05 and d > D_SUBIR + 0.25:
                self.abortar("algo delante de la escalera", ahora)
                return 0.0, 0.0, 0.0
            return max(0.05, min(0.15, d - D_SUBIR)), 0.0, w

        if e == "GIRAR":
            err = ang(self.m["yaw_b"] - yaw)
            if abs(err) < math.radians(3):
                self._ir("VERIFICAR", ahora)
                return 0.0, 0.0, 0.0
            if holg and holg["giro"] < self.cfg["radio_giro"]:
                self.abortar("no hay espacio para girar junto a la escalera", ahora)
                return 0.0, 0.0, 0.0
            return self._girar(err)

        if e == "VERIFICAR":          # de espaldas: la bajada tiene que estar detras
            det_a, centro, t_at = det.atras
            if dt < 1.2 or t_at < self.t_estado + 0.3:
                if dt > 5.0:
                    self.log("no tengo vista de atras; confio en lo que vi de frente")
                    self._marcha_on(ahora)
                return 0.0, 0.0, 0.0
            if det_a is None and centro == "suelo":
                self.abortar("detras hay suelo, no escalera", ahora)
            elif det_a is None and centro == "pared":
                self.abortar("hay algo detras", ahora)
            else:
                self._marcha_on(ahora)
            return 0.0, 0.0, 0.0

        if e == "MARCHA_ON":
            if dt > self.m["espera"] and not self.m.get("bal"):
                self.robot.balancear()
                self.m["bal"] = True
            if dt > self.m["espera"] + 0.6:
                self.m.update(z0=z, P0=P.copy(), hz=[], mov=(ahora, P.copy()), piso_cambio=False)
                self._ir("BAJANDO" if tipo == "bajar" else "SUBIENDO", ahora)
            return 0.0, 0.0, 0.0

        if e in ("BAJANDO", "SUBIENDO"):
            bajando = e == "BAJANDO"
            if self.m.get("P0") is None:           # retomado despues de DETENIDO
                self.m.update(P0=P.copy(), mov=(ahora, P.copy()), hz=[])
            yref = self.m["yaw_b"] if bajando else self.m["yaw_n"]
            w = max(-0.3, min(0.3, 1.2 * ang(yref - yaw)))
            f = np.array([math.cos(yref), math.sin(yref)])
            sgn = -1.0 if bajando else 1.0
            s = sgn * float(np.dot(P - self.m["P0"], f))      # avance (hacia atras si baja)
            dz = z - self.m["z0"]
            hz = self.m["hz"]
            hz.append((ahora, z))
            while hz and ahora - hz[0][0] > 1.5:
                hz.pop(0)
            if (dz < -0.06) if bajando else (dz > 0.06):
                self.m["piso_cambio"] = True
            nivelado = abs(pitch) < NIVEL and abs(roll) < math.radians(6)
            estable = (len(hz) > 8 and hz[-1][0] - hz[0][0] > 1.3
                       and max(v for _, v in hz) - min(v for _, v in hz) < 0.03)
            self.info = "%s %.2f m, recorrido %.2f m" % ("bajado" if bajando else "subido", abs(dz), s)
            # alguien en la escalera: espera (no aborta)
            # centrado entre las paredes/barandas de la escalera (si se ven)
            vy = 0.0
            if holg:
                izq, der = holg.get("pared_izq", math.inf), holg.get("pared_der", math.inf)
                if min(izq, der) < 0.40:
                    izq, der = min(izq, 0.6), min(der, 0.6)
                    vy = max(-0.08, min(0.08, 0.6 * (izq - der) / 2.0))
            # (subiendo, los escalones mismos son "obstaculos": solo cuenta algo mas alto)
            bloqueo = bool(holg) and ((holg["atras"][0] < 0.10) if bajando
                                      else (holg.get("alto_delante", 9.0) < 0.25))
            if bloqueo:
                # si es una baranda de un lado, se corre al otro; si no, espera (persona)
                izq, der = holg.get("pared_izq", math.inf), holg.get("pared_der", math.inf)
                if abs(izq - der) > 0.08 and max(izq, der) > 0.10:
                    self.info += " (me corro de la baranda)"
                    return 0.0, (0.08 if izq > der else -0.08), w
                self.info += " (algo en el camino, espero)"
                return 0.0, 0.0, 0.0
            hecho = (dz < -0.10) if bajando else (dz > 0.10)
            if hecho and nivelado and estable:
                if not bajando and det.confirmada("subir", ahora, P) is not None:
                    pass                           # descanso con mas escalones: sigue
                else:
                    self.log("%s: %s" % ("abajo" if bajando else "arriba", self.info))
                    self.m["Ps"] = P.copy()
                    self._ir("SALIR", ahora)
                    return 0.0, 0.0, 0.0
            t0, p0 = self.m["mov"]
            if np.linalg.norm(P - p0) > 0.04:
                self.m["mov"] = (ahora, P.copy())
            elif ahora - t0 > 6.0:
                if not self.m["piso_cambio"] and nivelado:
                    self.abortar("el perro no avanza en la escalera (revisa la marcha / firmware)", ahora)
                else:
                    self.detener("no avanza", ahora)
                return 0.0, 0.0, 0.0
            if bajando and not self.m["piso_cambio"] and s > self.m.get("dist", 0.7) + 0.9:
                self.abortar("pase el borde y no baje: no era escalera", ahora)
                return 0.0, 0.0, 0.0
            if s > 12.0:
                self.detener("recorrio demasiado", ahora)
                return 0.0, 0.0, 0.0
            v = self.cfg["v_bajar"] if bajando else self.cfg["v_subir"]
            return sgn * v, vy, w

        if e == "SALIR":
            bajando = tipo == "bajar"
            yref = self.m["yaw_b"] if bajando else self.m["yaw_n"]
            if np.linalg.norm(P - self.m["Ps"]) > 0.30 or dt > 3.5:
                self.m["dz"] = z - self.m["z0"]
                self.m["espera"] = self.robot.modo_escaleras(False)
                self.m["marcha"] = False
                self.m["bal"] = False
                self._ir("MARCHA_OFF", ahora)
                return 0.0, 0.0, 0.0
            v = self.cfg["v_bajar"] if bajando else self.cfg["v_subir"] * 0.7
            return (-v if bajando else v), 0.0, max(-0.3, min(0.3, 1.2 * ang(yref - yaw)))

        if e == "MARCHA_OFF":
            if dt > self.m["espera"] and not self.m.get("bal"):
                self.robot.balancear()
                self.m["bal"] = True
            if dt > self.m["espera"] + 0.6:
                self._ir("GIRAR_FRENTE" if tipo == "bajar" else "LISTO", ahora)
            return 0.0, 0.0, 0.0

        if e == "GIRAR_FRENTE":
            err = ang(self.m["yaw_n"] - yaw)
            if abs(err) < math.radians(4):
                self._ir("LISTO", ahora)
                return 0.0, 0.0, 0.0
            if holg and holg["giro"] < self.cfg["radio_giro"]:
                self._ir("LISTO", ahora)            # sin espacio: se queda de espaldas
                return 0.0, 0.0, 0.0
            return self._girar(err)

        if e == "LISTO":
            dz = float(self.m.get("dz", 0.0))
            self._fin(True, "%s %.2f m" % ("bajo" if tipo == "bajar" else "subio", abs(dz)), ahora,
                      dz=dz, yaw_n=self.m["yaw_n"], E=[float(v) for v in self.m["E"]])
            return 0.0, 0.0, 0.0

        if e == "RESTAURAR":
            if dt > self.m["espera"] and not self.m.get("bal_r"):
                self.robot.balancear()
                self.m["bal_r"] = True
            if dt > self.m["espera"] + 0.6:
                self._fin(False, self.m.get("motivo_abortar", "cancelada"), ahora)
            return 0.0, 0.0, 0.0
        return 0.0, 0.0, 0.0

    def _marcha_on(self, ahora):
        self.m["espera"] = self.robot.modo_escaleras(True)
        self.m["marcha"] = True
        self.m["bal"] = False
        self._ir("MARCHA_ON", ahora)
