"""Escaleras: confirmacion (subida y bajada) y maniobra.

SUBIR (de frente):  DETENER -> ALINEAR -> MEDIR -> CENTRAR -> ACERCAR -> MARCHA_ON -> SUBIENDO
                    -> SALIR -> MARCHA_OFF -> LISTO
BAJAR (SIEMPRE en reversa):  DETENER -> ALINEAR -> MEDIR -> CENTRAR -> DISTANCIA -> GIRAR 180
                    -> VERIFICAR -> MARCHA_ON -> BAJANDO (de reversa) -> SALIR -> MARCHA_OFF
                    -> GIRAR_FRENTE -> LISTO

Antes de pisar la escalera, cualquier duda = aborta y vuelve a la marcha normal.
Ya encima de los escalones nunca cambia de marcha: si algo falla se queda quieto (DETENIDO)
y espera a 'go2 seguir' o al control remoto.
Todo en el marco odom del perro (no depende del mapa).
"""
import math

import numpy as np

from . import terreno
from .control import ang

CONFIRMAR_N, CONFIRMAR_T = 3, 4.0        # detecciones seguidas (en 4 s) para confirmar
TOL_POS, TOL_ANG = 0.15, math.radians(10)
NIVEL = math.radians(5)                  # pitch con el que el perro esta "a nivel"
D_GIRO, D_MIN, D_MAX = 0.65, 0.50, 1.20  # bajada: distancia del centro al borde para girar 180
D_SUBIR = 0.45                           # subida: centro del perro a 45 cm del primer escalon
LIMITE = {"DETENER": 3, "ALINEAR": 15, "MEDIR": 10, "CENTRAR": 10, "DISTANCIA": 12,
          "ACERCAR": 12, "GIRAR": 20, "VERIFICAR": 6, "MARCHA_ON": 10, "BAJANDO": 90,
          "SUBIENDO": 90, "SALIR": 5, "MARCHA_OFF": 10, "GIRAR_FRENTE": 20, "RESTAURAR": 10}
EN_ESCALERA = ("SUBIENDO", "BAJANDO", "SALIR", "DETENIDO")


def media_ang(angs):
    return math.atan2(sum(math.sin(a) for a in angs), sum(math.cos(a) for a in angs))


def _lim(v, m):
    return max(-m, min(m, v))


class Detector(object):
    """Confirma escaleras con varias nubes seguidas (en odom).

    Mira delante y detras en cada nube; con alrededor=True ademas en 6 direcciones mas (para
    descubrir escaleras de costado mientras explora). Guarda en 'motivo' por que lo que hay
    delante NO es (todavia) una escalera de bajada: es lo que muestra 'go2 escaleras'.
    """

    def __init__(self):
        self.hist = {"bajar": [], "subir": []}
        self.atras = (None, "nada", 0.0)     # (deteccion, estado_centro, t) mirando hacia atras
        self.motivo = ""                      # lo que hay delante, en palabras
        self.centro = "nada"                  # 'caida' | 'suelo' | 'pared' | 'nada' delante

    def limpiar(self):
        self.hist = {"bajar": [], "subir": []}

    def actualizar(self, xyz, pose, suelo, ahora, alrededor=False):
        """pose = (x, y, z, yaw)."""
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
            det, centro, motivo = terreno.analizar_bajada(fx, fy, h)
            if d == 0.0:
                self.motivo, self.centro = motivo, centro
            elif d == math.pi:
                self.atras = (det, centro, ahora)
            if det is not None and det["dist"] < 2.0:
                self._sumar("bajar", det, x, y, yd, suelo, ahora)
            sub = terreno.analizar_subida(fx, fy, h)
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
        """Escalera confirmada (>= CONFIRMAR_N vistas coherentes en CONFIRMAR_T s) o None.
        Con pos=(x, y) agrega 'dist': distancia perpendicular del perro al borde."""
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
    """Maquina de estados de una subida o bajada. paso() devuelve (vx, vy, wz).

    robot: objeto con modo_escaleras(bool) -> segundos de espera, y balancear().
    cfg:   v_subir, v_bajar, ancho_min_escalera, w_min, radio_giro.
    """

    def __init__(self, robot, cfg, log):
        self.robot, self.cfg, self.log = robot, cfg, log
        self.estado = "INACTIVA"
        self.t_estado = 0.0
        self.m = {}                 # memoria de la maniobra en curso
        self.resultado = None       # dict al terminar (ok, motivo, tipo, dz, yaw_n, E)
        self.info = ""

    # ---------------------------------------------------------------- estado
    @property
    def activa(self):
        return self.estado not in ("INACTIVA", "DETENIDO")

    @property
    def bajando(self):
        return self.m.get("tipo") == "bajar"

    def en_escalera(self, pitch=0.0):
        if self.estado not in EN_ESCALERA:
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
        self.log("escalera %s: %s" % ("LISTA" if ok else "cancelada", motivo))
        self._ir("INACTIVA", ahora)

    def abortar(self, motivo, ahora):
        """Cancela ANTES de pisar la escalera (si la marcha ya estaba puesta, la quita primero)."""
        if self.m.get("marcha"):
            self.m["motivo_abortar"] = motivo
            self.m["espera"] = self.robot.modo_escaleras(False)
            self.m["marcha"] = False
            self._ir("RESTAURAR", ahora)
            return
        self._fin(False, motivo, ahora)

    def detener(self, motivo, ahora):
        """Encima de los escalones: quieto, sin cambiar de marcha, hasta 'go2 seguir'."""
        self.log("DETENIDO EN LA ESCALERA: %s. Usa 'go2 seguir' o el control remoto." % motivo)
        self.m["motivo_det"] = motivo
        self._ir("DETENIDO", ahora)

    def seguir(self, ahora):
        """Retomar despues de DETENIDO (el usuario lo pidio)."""
        if self.estado == "DETENIDO":
            self.m["P0"] = None
            self._ir("BAJANDO" if self.bajando else "SUBIENDO", ahora)

    # ---------------------------------------------------------------- ayudas
    def _girar(self, err):
        return 0.0, 0.0, math.copysign(max(self.cfg["w_min"], min(0.7, 1.5 * abs(err))), err)

    def _w_rumbo(self, yref, yaw):
        """Giro suave para mantener el rumbo yref."""
        return _lim(1.2 * ang(yref - yaw), 0.3)

    def _d_borde(self, P):
        """Distancia del centro del perro al borde, a lo largo de la normal de la escalera."""
        return float(np.dot(self.m["E"] - P, self.m["n"]))

    def _marcha_on(self, ahora):
        self.m["espera"] = self.robot.modo_escaleras(True)
        self.m["marcha"] = True
        self.m["bal"] = False
        self._ir("MARCHA_ON", ahora)

    def _vencio(self, e, pitch, ahora):
        """Un estado tardo mas de LIMITE[e] s."""
        tipo = self.m.get("tipo")
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

    # ---------------------------------------------------------------- un paso
    def paso(self, ahora, pose, det, holg):
        """pose = (x, y, z, yaw, pitch, roll) odom. det = Detector. holg = guardian()."""
        e = self.estado
        if e in ("INACTIVA", "DETENIDO"):
            return 0.0, 0.0, 0.0
        if ahora - self.t_estado > LIMITE.get(e, 1e9):
            self._vencio(e, pose[4], ahora)
            return 0.0, 0.0, 0.0
        fn = getattr(self, "_e_" + e.lower(), None)
        if fn is None:
            return 0.0, 0.0, 0.0
        x, y, z, yaw, pitch, roll = pose
        ctx = {"ahora": ahora, "dt": ahora - self.t_estado, "P": np.array([x, y]), "z": z,
               "yaw": yaw, "pitch": pitch, "roll": roll, "det": det, "holg": holg}
        return fn(ctx) or (0.0, 0.0, 0.0)

    # cada estado: devuelve (vx, vy, wz) o None (= quieto)
    def _e_detener(self, c):
        if c["dt"] <= 0.8:
            return None
        conf = c["det"].confirmada(self.m["tipo"], c["ahora"], c["P"])
        if conf is None:
            self.abortar("perdi de vista la escalera", c["ahora"])
        elif conf["ancho"] < self.cfg["ancho_min_escalera"] and not conf["abierta"]:
            self.abortar("escalera muy angosta (%.2f m)" % conf["ancho"], c["ahora"])
        else:
            self.m.update(E=conf["E"], yaw_n=conf["yaw_n"])
            self._ir("ALINEAR", c["ahora"])

    def _e_alinear(self, c):
        err = ang(self.m["yaw_n"] - c["yaw"])
        if abs(err) < math.radians(3):
            self.m["P_medir"] = c["P"].copy()
            self._ir("MEDIR", c["ahora"])
            return None
        return self._girar(err)

    def _e_medir(self, c):
        """De frente y quieto: medir bien el borde y el centro de la escalera."""
        if c["dt"] < 1.0:
            return None
        conf = c["det"].confirmada(self.m["tipo"], c["ahora"], c["P"])
        holg = c["holg"]
        if conf is None:
            if c["dt"] > 8.0:
                self.abortar("de frente ya no veo la escalera", c["ahora"])
                return None
            # muy pegado al borde la cabeza tapa el primer escalon: retrocede un poco
            if (self.bajando and c["dt"] > 2.5 and np.linalg.norm(c["P"] - self.m["P_medir"]) < 0.30
                    and holg and holg["atras"][0] > 0.25 and holg["atras"][1] > 0.4):
                return -0.06, 0.0, 0.0
            return None
        n = np.array([math.cos(conf["yaw_n"]), math.sin(conf["yaw_n"])])
        t = np.array([-n[1], n[0]])
        lat = float(np.dot(conf["E"] - c["P"], t))
        self.m.update(E=conf["E"], yaw_n=conf["yaw_n"], n=n, t=t, z_borde=conf["z"],
                      meta_lat=c["P"] + lat * t, dist=conf["dist"])
        self.log("borde a %.2f m, centro de la escalera a %+.2f m" % (conf["dist"], lat))
        self._ir("CENTRAR", c["ahora"])

    def _e_centrar(self, c):
        """Se desliza de lado hasta quedar en el centro de la escalera."""
        yaw, holg = c["yaw"], c["holg"]
        t_rob = np.array([-math.sin(yaw), math.cos(yaw)])
        err = float(np.dot(self.m["meta_lat"] - c["P"], t_rob))
        w = self._w_rumbo(self.m["yaw_n"], yaw)
        if abs(err) < 0.04 or (holg and holg["izq" if err > 0 else "der"][0] < 0.06):
            if self.bajando:
                d = self._d_borde(c["P"])
                self.m["d_obj"] = None if D_MIN <= d <= D_MAX else D_GIRO
                self._ir("DISTANCIA", c["ahora"])
            else:
                self._ir("ACERCAR", c["ahora"])
            return None
        return 0.0, math.copysign(max(0.06, min(0.15, 1.5 * abs(err))), err), w

    def _e_distancia(self, c):
        """Solo bajada: ponerse a D_GIRO del borde para girar 180 con espacio."""
        d = self._d_borde(c["P"])
        w = self._w_rumbo(self.m["yaw_n"], c["yaw"])
        if self.m["d_obj"] is None or abs(d - self.m["d_obj"]) < 0.05:
            self.m["dist"] = d
            self.m["yaw_b"] = ang(self.m["yaw_n"] + math.pi)
            self._ir("GIRAR", c["ahora"])
            return None
        err = d - D_GIRO
        return math.copysign(max(0.05, min(0.12, abs(err))), err), 0.0, w

    def _e_acercar(self, c):
        """Solo subida: de frente hasta D_SUBIR del primer escalon."""
        d = self._d_borde(c["P"])
        w = self._w_rumbo(self.m["yaw_n"], c["yaw"])
        holg = c["holg"]
        if abs(d - D_SUBIR) < 0.05 or d < D_SUBIR:
            self._marcha_on(c["ahora"])
            return None
        if holg and holg["adelante"][0] < 0.05 and d > D_SUBIR + 0.25:
            self.abortar("algo delante de la escalera", c["ahora"])
            return None
        return max(0.05, min(0.15, d - D_SUBIR)), 0.0, w

    def _e_girar(self, c):
        """Solo bajada: 180 grados, para bajar de espaldas."""
        err = ang(self.m["yaw_b"] - c["yaw"])
        if abs(err) < math.radians(3):
            self._ir("VERIFICAR", c["ahora"])
            return None
        if c["holg"] and c["holg"]["giro"] < self.cfg["radio_giro"]:
            self.abortar("no hay espacio para girar junto a la escalera", c["ahora"])
            return None
        return self._girar(err)

    def _e_verificar(self, c):
        """De espaldas a la bajada, ultima comprobacion antes de poner la marcha:
        1) por odometria el perro sigue a la distancia medida del borde (girando no se corrio);
        2) si el lidar llega a ver detras, no puede haber suelo firme ni una pared ahi."""
        d = self._d_borde(c["P"])             # distancia al borde a lo largo de su normal
        if d < 0.35 or abs(d - self.m.get("dist", D_GIRO)) > 0.30:
            self.abortar("al girar me corri respecto del borde (%.2f m)" % d, c["ahora"])
            return None
        det_a, centro, t_at = c["det"].atras
        if c["dt"] < 1.2 or t_at < self.t_estado + 0.3:
            if c["dt"] > 5.0:
                self.log("no tengo vista de atras; confio en lo que medi de frente")
                self._marcha_on(c["ahora"])
            return None
        if det_a is None and centro == "suelo":
            self.abortar("detras hay suelo, no escalera", c["ahora"])
        elif det_a is None and centro == "pared":
            self.abortar("hay algo detras", c["ahora"])
        else:
            self._marcha_on(c["ahora"])

    def _e_marcha_on(self, c):
        if c["dt"] > self.m["espera"] and not self.m.get("bal"):
            self.robot.balancear()
            self.m["bal"] = True
        if c["dt"] > self.m["espera"] + 0.6:
            self.m.update(z0=c["z"], P0=c["P"].copy(), hz=[], mov=(c["ahora"], c["P"].copy()),
                          piso_cambio=False)
            self._ir("BAJANDO" if self.bajando else "SUBIENDO", c["ahora"])

    def _e_bajando(self, c):
        return self._en_escalones(c, bajando=True)

    def _e_subiendo(self, c):
        return self._en_escalones(c, bajando=False)

    def _en_escalones(self, c, bajando):
        """Encima de los escalones: avanza (hacia atras si baja) manteniendo el rumbo, centrado
        entre paredes/barandas, y espera si algo se cruza. Termina cuando volvio a estar a
        nivel y la altura se estabilizo."""
        ahora, P, z, yaw, holg, det = c["ahora"], c["P"], c["z"], c["yaw"], c["holg"], c["det"]
        if self.m.get("P0") is None:           # retomado despues de DETENIDO
            self.m.update(P0=P.copy(), mov=(ahora, P.copy()), hz=[])
        yref = self.m["yaw_b"] if bajando else self.m["yaw_n"]
        w = self._w_rumbo(yref, yaw)
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
        nivelado = abs(c["pitch"]) < NIVEL and abs(c["roll"]) < math.radians(6)
        estable = (len(hz) > 8 and hz[-1][0] - hz[0][0] > 1.3
                   and max(v for _, v in hz) - min(v for _, v in hz) < 0.03)
        self.info = "%s %.2f m, recorrido %.2f m" % ("bajado" if bajando else "subido", abs(dz), s)
        # centrado entre las paredes/barandas de la escalera (si se ven)
        vy = 0.0
        if holg:
            izq, der = holg.get("pared_izq", math.inf), holg.get("pared_der", math.inf)
            if min(izq, der) < 0.40:
                izq, der = min(izq, 0.6), min(der, 0.6)
                vy = _lim(0.6 * (izq - der) / 2.0, 0.08)
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
            return None
        hecho = (dz < -0.10) if bajando else (dz > 0.10)
        if hecho and nivelado and estable:
            if bajando or det.confirmada("subir", ahora, P) is None:
                self.log("%s: %s" % ("abajo" if bajando else "arriba", self.info))
                self.m["Ps"] = P.copy()
                self._ir("SALIR", ahora)
                return None
            # descanso con mas escalones delante: sigue subiendo
        t0, p0 = self.m["mov"]
        if np.linalg.norm(P - p0) > 0.04:
            self.m["mov"] = (ahora, P.copy())
        elif ahora - t0 > 6.0:
            if not self.m["piso_cambio"] and nivelado:
                self.abortar("el perro no avanza en la escalera (revisa la marcha / firmware)", ahora)
            else:
                self.detener("no avanza", ahora)
            return None
        if bajando and not self.m["piso_cambio"] and s > self.m.get("dist", 0.7) + 0.9:
            self.abortar("pase el borde y no baje: no era escalera", ahora)
            return None
        if s > 12.0:
            self.detener("recorrio demasiado", ahora)
            return None
        v = self.cfg["v_bajar"] if bajando else self.cfg["v_subir"]
        return sgn * v, vy, w

    def _e_salir(self, c):
        """Ya a nivel: unos 30 cm mas para dejar los escalones atras y quitar la marcha."""
        bajando = self.bajando
        yref = self.m["yaw_b"] if bajando else self.m["yaw_n"]
        if np.linalg.norm(c["P"] - self.m["Ps"]) > 0.30 or c["dt"] > 3.5:
            self.m["dz"] = c["z"] - self.m["z0"]
            self.m["espera"] = self.robot.modo_escaleras(False)
            self.m["marcha"] = False
            self.m["bal"] = False
            self._ir("MARCHA_OFF", c["ahora"])
            return None
        v = self.cfg["v_bajar"] if bajando else self.cfg["v_subir"] * 0.7
        return (-v if bajando else v), 0.0, self._w_rumbo(yref, c["yaw"])

    def _e_marcha_off(self, c):
        if c["dt"] > self.m["espera"] and not self.m.get("bal"):
            self.robot.balancear()
            self.m["bal"] = True
        if c["dt"] > self.m["espera"] + 0.6:
            self._ir("GIRAR_FRENTE" if self.bajando else "LISTO", c["ahora"])

    def _e_girar_frente(self, c):
        """Abajo, de espaldas: vuelve a mirar hacia donde iba (si hay espacio)."""
        err = ang(self.m["yaw_n"] - c["yaw"])
        if abs(err) < math.radians(4) or (c["holg"] and c["holg"]["giro"] < self.cfg["radio_giro"]):
            self._ir("LISTO", c["ahora"])      # sin espacio: se queda de espaldas
            return None
        return self._girar(err)

    def _e_listo(self, c):
        dz = float(self.m.get("dz", 0.0))
        self._fin(True, "%s %.2f m" % ("bajo" if self.bajando else "subio", abs(dz)), c["ahora"],
                  dz=dz, yaw_n=self.m["yaw_n"], E=[float(v) for v in self.m["E"]])

    def _e_restaurar(self, c):
        """Abortada con la marcha puesta: la quita y recien entonces termina."""
        if c["dt"] > self.m["espera"] and not self.m.get("bal_r"):
            self.robot.balancear()
            self.m["bal_r"] = True
        if c["dt"] > self.m["espera"] + 0.6:
            self._fin(False, self.m.get("motivo_abortar", "cancelada"), c["ahora"])
