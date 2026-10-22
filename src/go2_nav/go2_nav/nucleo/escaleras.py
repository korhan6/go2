"""Escaleras: confirmacion (subida y bajada) y maniobra.

SUBIR (de frente):  DETENER -> ALINEAR -> MEDIR -> CENTRAR -> ACERCAR -> MARCHA_ON -> SUBIENDO
                    -> MARCHA_OFF -> LISTO
                    (SUBIENDO para en seco cuando el cuerpo queda plano arriba: "LLEGUE ARRIBA")
BAJAR (SIEMPRE en reversa):  DETENER -> ALINEAR -> MEDIR -> CENTRAR -> DISTANCIA -> GIRAR 180
                    (-> ESPACIO si no cabe girando: se aleja de lo que estorba y vuelve a GIRAR)
                    -> VERIFICAR -> MARCHA_ON -> BAJANDO (de reversa) -> MARCHA_OFF -> LISTO
                    (BAJANDO para en seco cuando el cuerpo vuelve a quedar plano: "LLEGUE ABAJO".
                    Abajo se queda MIRANDO LA ESCALERA: no retrocede (atras no ve) ni gira (el primer
                    escalon queda pegado delante y girando lo pisaria))

Encima de los escalones va DERECHO y CENTRADO segun lo que el lidar ve delante (contrahuellas y
paredes: terreno.orientacion_escalera), en las dos direcciones (bajando en reversa el perro
tambien mira escalera arriba).

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
T_CRUDOS = 1.0                           # s de nubes crudas para el rumbo fino de la escalera
BAJADA_LEJOS = 1.6                       # m (centro del perro al borde): mas lejos una bajada no cuenta
BAJADA_ANCHO_MIN = 0.36                  # m: mas angosta no es escalera (el cuerpo mide 0.31)
# tolerancias entre vistas: una POSIBLE bajada se ve de lejos y su borde "camina" a lo largo
# mientras el perro se mueve (se ve otro pedazo), asi que su tolerancia es mas ancha
TOL = {"bajar": (0.15, math.radians(10)), "subir": (0.15, math.radians(10)),
       "posible": (0.40, math.radians(20))}
# FIN de la escalera: en cuanto el cuerpo vuelve a quedar PLANO o casi (las 4 patas abajo / arriba)
# despues de haber ido inclinado. Bajando atras no se ve nada (el L1 mira adelante): no se camina ni
# un paso mas de reversa. El pitch se promedia T_FILTRO s (sin el vaiven de cada paso).
INCLINADO = math.radians(7)              # pitch de "ya estoy en los escalones"
T_INCLINADO = 0.3                        # s inclinado seguidos para contarlo
PLANO_MIN, PLANO_MAX = math.radians(5.5), math.radians(8)   # "casi plano": 35 % de lo mas inclinado, en este rango
T_FILTRO = 0.4
T_PLANO = 0.4                            # s casi plano (ya frenado) para darlo por terminado
RECORRIDO_MAX = 6.0                      # m: una escalera de casa no es mas larga (antes 12)
BUSCAR_MAX = 0.30                        # m que se mueve buscando espacio para girar antes de bajar
D_GIRO, D_MIN, D_MAX = 0.65, 0.50, 1.20  # bajada: distancia del centro al borde para girar 180
D_SUBIR = 0.45                           # subida: centro del perro a 45 cm del primer escalon
# s maximos por estado (los giros esperan quieto entre pulsos finos: ver _girar_a)
LIMITE = {"DETENER": 3, "ALINEAR": 25, "MEDIR": 10, "CENTRAR": 10, "DISTANCIA": 12,
          "ACERCAR": 12, "GIRAR": 30, "ESPACIO": 10, "VERIFICAR": 6, "MARCHA_ON": 10, "BAJANDO": 90,
          "SUBIENDO": 90, "MARCHA_OFF": 10, "RESTAURAR": 10}
EN_ESCALERA = ("SUBIENDO", "BAJANDO", "DETENIDO")
# EMPUJE: el Go2 casi no camina por debajo de ~0.20 m/s (pisa en el sitio) y en la escalera necesita
# a veces mas. Si avanza menos del 40 % de lo que se le pide, la velocidad sube 3 cm/s cada 0.5 s
# (hasta v_escalera_max) y se mantiene cuando ya avanza. Solo en los estados que caminan.
CAMINAN = ("CENTRAR", "DISTANCIA", "ACERCAR", "BAJANDO", "SUBIENDO", "MEDIR", "ESPACIO")
EMPUJE_PASO, EMPUJE_CADA, EMPUJE_VENTANA = 0.03, 0.5, 0.8
# Alinearse SIN PASARSE con un perro que responde tarde (~0.3 s) y no gira lento (w_min):
#   grueso: gira normal hasta que lo que falta (contando lo que va a seguir girando) es ZONA_FINA;
#   espera a que quede quieto;
#   fino:   pulsos cortos que corrigen la MITAD de lo que falta, esperando quieto entre pulsos.
# Nunca cruza la meta aunque el retraso real sea otro (antes: se pasaba y volvia, en zigzag).
LAG_GIRO = 0.35              # s: lo que sigue girando despues de la orden (estimado)
ZONA_FINA = math.radians(25)
QUIETO_W = 0.08              # rad/s medidos: ya no gira
T_ASENTAR = 0.7              # s minimos entre una orden de giro y la siguiente decision fina
                             # (con retraso, justo despues de la orden el perro parece quieto)


def media_ang(angs):
    return math.atan2(sum(math.sin(a) for a in angs), sum(math.cos(a) for a in angs))


def _lim(v, m):
    return max(-m, min(m, v))


def perfil_frente(fx, fy, h, paso=0.10, n=12):
    """Lo mas alto (cm sobre el suelo) en el carril del centro, cada 10 cm desde el hocico: lo que
    el lidar ve delante, en numeros, para diagnosticar una escalera que no se confirma."""
    m = (np.abs(fy) < 0.15) & (h > -0.6) & (h < 1.0)
    a = fx[m] - terreno.LARGO2
    out = []
    for k in range(n):
        s = (a >= k * paso) & (a < (k + 1) * paso)
        out.append("%d" % round(100 * float(np.max(h[m][s]))) if np.any(s) else "-")
    return " ".join(out)


class Detector(object):
    """Confirma escaleras con varias nubes seguidas (en odom).

    Tipos: 'subir', 'bajar' (se VE el primer escalon) y 'posible' (caida empinada de borde
    recto, sin huella vista: de lejos una bajada se ve asi; tambien un balcon). Una 'posible'
    nunca arranca la maniobra: el gestor manda al perro a mirarla de cerca.
    Mira delante y detras en cada nube; con alrededor=True ademas en 6 direcciones mas (para
    descubrir escaleras de costado mientras explora). Guarda en 'motivo' por que lo que hay
    delante NO es (todavia) una escalera de bajada: es lo que muestra 'go2 escaleras'.
    """

    def __init__(self):
        self.hist = {"bajar": [], "subir": [], "posible": []}
        self.atras = (None, "nada", 0.0)     # (deteccion, estado_centro, t) mirando hacia atras
        self.motivo = ""                      # lo que hay delante, en palabras (bajada)
        self.motivo_subida = ""               # idem, para una escalera de subida
        self.subida_frente = (None, -1e9)     # (terreno.analizar_subida delante, t): lo de AHORA, sin historia
        self.centro = "nada"                  # 'caida' | 'suelo' | 'pared' | 'nada' delante
        self.perfil = ""                      # alturas (cm) delante del hocico, cada 10 cm: 'go2 escaleras'
        self.alto_cuerpo = None               # m del cuerpo sobre el suelo estimado (normal ~0.30)
        self.crudos = []                      # (t, nube cruda limpia) del ultimo segundo
        self.orient = None                    # terreno.orientacion_escalera delante (o None), con 't'

    def limpiar(self):
        self.hist = {k: [] for k in self.hist}
        self.crudos, self.orient = [], None

    def actualizar(self, xyz, pose, suelo, ahora, alrededor=False, crudos=None, fino=True):
        """pose = (x, y, z, yaw). crudos: las nubes nuevas SIN voxelizar (para el rumbo fino de la
        escalera: ver terreno.orientacion_escalera). fino=False: no calcularlo esta vez (es lo mas
        pesado; fuera de la escalera alcanza con 2 veces por segundo)."""
        x, y, z, yaw = pose
        if crudos:
            self.crudos.extend((ahora, c) for c in crudos if len(c))
        while self.crudos and ahora - self.crudos[0][0] > T_CRUDOS:
            self.crudos.pop(0)
        if self.crudos and fino:
            o = terreno.orientacion_escalera(np.concatenate([c for _t, c in self.crudos]), x, y, yaw, suelo)
            if o is not None:
                o["t"] = ahora
                o["yaw"] = yaw
            self.orient = o
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
            escalera = det is not None and det["huella"]
            if d == 0.0:
                self.motivo, self.centro = motivo, centro
                self.perfil = perfil_frente(fx, fy, h)
                self.alto_cuerpo = z - suelo
            elif d == math.pi:
                self.atras = (det if escalera else None, centro, ahora)
            # una bajada cuenta hasta BAJADA_LEJOS m (mas lejos, con piso brillante, el piso rasante no
            # devuelve y lo que se ve "abajo" es el reflejo de algo que el lidar no ve directo: el
            # tablero de una mesa) y si es al menos tan ancha como un perro (un reflejo es angosto)
            if escalera and det["dist"] < BAJADA_LEJOS and det["ancho"] >= BAJADA_ANCHO_MIN:
                self._sumar("bajar", det, x, y, yd, suelo, ahora)
            sub = terreno.analizar_subida(fx, fy, h)
            if d == 0.0:
                self.subida_frente = (sub, ahora)
                self.motivo_subida = ("escalera de SUBIDA a %.2f m (ancho %.2f m)" % (sub["dist"], sub["ancho"])
                                      if sub is not None else
                                      "no veo escalones de subida delante (de frente, a 0.5-2 m, centrado)")
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
        tol_pos, tol_ang = TOL[tipo]
        ok = [d for d in rec if math.hypot(d["ex"] - ex, d["ey"] - ey) < tol_pos
              and abs(ang(d["yaw_n"] - ref)) < tol_ang]
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
        self.w_ult = 0.0            # ultimo giro ordenado (para anticipar el frenado)

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

    def _girar_a(self, c, yref, tol):
        """Giro en el sitio hasta el rumbo yref sin pasarse. Devuelve (alineado, (vx, vy, wz))."""
        ahora, yaw = c["ahora"], c["yaw"]
        err = ang(yref - yaw)
        g = self.m.get("giro")
        if g is None or g["yref"] != yref:
            g = self.m["giro"] = {"yref": yref, "t": ahora, "yaw": yaw, "w": 0.0, "pulso": 0.0,
                                  "fino": False, "espera": 0.0}
        dt = ahora - g["t"]
        if dt > 0.05:                             # giro medido (rad/s), suavizado
            g["w"] = 0.5 * g["w"] + 0.5 * ang(yaw - g["yaw"]) / dt
            g["t"], g["yaw"] = ahora, yaw
        quieto = abs(g["w"]) < QUIETO_W and ahora >= g["espera"]
        if ahora < g["pulso"]:                    # pulso fino en curso
            return False, (0.0, 0.0, math.copysign(self.cfg["w_min"], err))
        if abs(err) < tol:
            return quieto, (0.0, 0.0, 0.0)
        # lo que va a seguir girando: lo medido o lo ultimo ordenado, lo que sea mas
        sigue = max(abs(g["w"]), abs(self.w_ult)) * LAG_GIRO
        if not g["fino"] and abs(err) - sigue > ZONA_FINA:
            return False, self._girar(err)       # grueso
        if not g["fino"]:
            g["fino"] = True                      # (una vez cerca, ya no vuelve al grueso)
            g["espera"] = ahora + T_ASENTAR
        if not quieto:
            return False, (0.0, 0.0, 0.0)         # esperar a que termine de girar
        # fino: un pulso que gira la mitad de lo que falta (a w_min), entre 0.1 y 0.6 s
        g["pulso"] = ahora + max(0.1, min(0.6, 0.5 * abs(err) / self.cfg["w_min"]))
        g["espera"] = g["pulso"] + T_ASENTAR
        return False, (0.0, 0.0, math.copysign(self.cfg["w_min"], err))

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
        if e == "MARCHA_OFF":
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
        cmd = self._empuje(ctx, fn(ctx) or (0.0, 0.0, 0.0))
        self.w_ult = cmd[2]
        return cmd

    def _empuje(self, c, cmd):
        """Sube la velocidad de a poco si el perro no avanza lo que se le pide (ver EMPUJE_*)."""
        vx, vy, wz = cmd
        v = math.hypot(vx, vy)
        ahora, P = c["ahora"], c["P"]
        g = self.m.setdefault("emp", {"k": 1.0, "t": ahora, "P": P.copy(), "t_sube": ahora, "v_env": 0.0})
        if v < 0.02 or self.estado not in CAMINAN:
            g["t"], g["P"], g["v_env"] = ahora, P.copy(), 0.0      # quieto o girando: no se mide
            return cmd
        v_max = self.cfg.get("v_escalera_max", 0.40)
        dt = ahora - g["t"]
        if dt >= EMPUJE_VENTANA and g["v_env"] > 0.0:
            real = float(np.linalg.norm(P - g["P"])) / dt
            if real < 0.4 * g["v_env"] and ahora - g["t_sube"] >= EMPUJE_CADA and g["v_env"] < v_max - 1e-3:
                g["k"] = min(v_max / v, (g["v_env"] + EMPUJE_PASO) / v)
                g["t_sube"] = ahora
                self.info_empuje = "empujando %.2f m/s (no avanzaba)" % (v * g["k"])
            g["t"], g["P"] = ahora, P.copy()
        k = min(g["k"], v_max / v) if v > 0 else 1.0
        g["v_env"] = v * k
        return vx * k, vy * k, wz

    def empuje_al_maximo(self):
        g = self.m.get("emp")
        return g is None or g["v_env"] >= self.cfg.get("v_escalera_max", 0.40) - 1e-3

    # cada estado: devuelve (vx, vy, wz) o None (= quieto)
    def _e_detener(self, c):
        if c["dt"] <= 0.8:
            return None
        conf = c["det"].confirmada(self.m["tipo"], c["ahora"], c["P"])
        if conf is None:
            self.abortar("perdi de vista la escalera", c["ahora"])
        elif conf["ancho"] + terreno.RES_ANCHO < self.cfg["ancho_min_escalera"] and not conf["abierta"]:
            self.abortar("escalera muy angosta (%.2f m; minimo %.2f)" % (conf["ancho"], self.cfg["ancho_min_escalera"]),
                         c["ahora"])
        else:
            self.m.update(E=conf["E"], yaw_n=conf["yaw_n"])
            self._ir("ALINEAR", c["ahora"])

    def _e_alinear(self, c):
        listo, cmd = self._girar_a(c, self.m["yaw_n"], math.radians(3))
        if listo:
            self.m["P_medir"] = c["P"].copy()
            self._ir("MEDIR", c["ahora"])
            return None
        return cmd

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
        yaw_n = conf["yaw_n"]
        n = np.array([math.cos(yaw_n), math.sin(yaw_n)])
        t = np.array([-n[1], n[0]])
        lat = float(np.dot(conf["E"] - c["P"], t))
        # rumbo FINO con las contrahuellas y las paredes (terreno.orientacion_escalera): el borde en
        # celdas de 5 cm sale con +-5 grados, y en una escalera angosta eso es ir en diagonal
        o = c["det"].orient
        fino = ""
        if (o is not None and c["ahora"] - o["t"] < 0.6 and (o["contras"] >= 3 or o["paredes"])
                and abs(ang(o["yaw"] + o["ang"] - yaw_n)) < math.radians(15)):
            yaw_n = ang(o["yaw"] + o["ang"])
            n = np.array([math.cos(yaw_n), math.sin(yaw_n)])
            t = np.array([-n[1], n[0]])
            if o["paredes"] == 2:                  # entre dos paredes: el centro entre ellas
                lat = o["centro"]
            fino = " (rumbo fino: %+.1f grados%s)" % (math.degrees(ang(yaw_n - conf["yaw_n"])),
                                                     ", centro entre paredes" if o["paredes"] == 2 else "")
        self.m.update(E=conf["E"], yaw_n=yaw_n, n=n, t=t, z_borde=conf["z"],
                      meta_lat=c["P"] + lat * t, dist=conf["dist"])
        self.log("borde a %.2f m, centro de la escalera a %+.2f m%s" % (conf["dist"], lat, fino))
        if abs(ang(yaw_n - c["yaw"])) > math.radians(3) and not self.m.get("realineado"):
            self.m["realineado"] = True            # alinearse otra vez con el rumbo fino (una sola vez)
            self.m["giro"] = None
            self._ir("ALINEAR", c["ahora"])
            return None
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
        """Solo bajada: 180 grados, para bajar de espaldas. Sin espacio para girar: primero se
        aleja de lo que estorba (ESPACIO) y recien si no lo encuentra, cancela."""
        holg = c["holg"]
        libre = holg["giro"] if holg else math.inf
        # ya girando, 2 cm menos de exigencia (el ruido de una nube no lo frena a mitad del giro)
        if libre < self.cfg["radio_giro"] - (0.02 if self.m.get("girando") else 0.0):
            if self.m.get("buscado", 0.0) < BUSCAR_MAX:
                self.m["P_buscar"] = c["P"].copy()
                self.log("para girar tengo %.0f cm libres (hacen falta %.0f): me alejo de lo que estorba"
                         % (100 * libre, 100 * self.cfg["radio_giro"]))
                self._ir("ESPACIO", c["ahora"])
                return None
            self.abortar("no hay espacio para girar junto a la escalera (%.0f cm libres alrededor, hacen falta %.0f)"
                         % (100 * libre, 100 * self.cfg["radio_giro"]), c["ahora"])
            return None
        listo, cmd = self._girar_a(c, self.m["yaw_b"], math.radians(3))
        if listo:
            self._ir("VERIFICAR", c["ahora"])
            return None
        self.m["girando"] = abs(cmd[2]) > 0.0 or self.m.get("girando", False)
        return cmd

    def _e_espacio(self, c):
        """Se desliza alejandose de lo que no lo deja girar (de lado o hacia atras del borde, nunca
        hacia la bajada), hasta BUSCAR_MAX m en total. Despues vuelve a GIRAR."""
        holg = c["holg"]
        hecho = float(np.linalg.norm(c["P"] - self.m["P_buscar"]))
        total = self.m.get("buscado", 0.0) + hecho
        encontro = holg is not None and holg["giro"] >= self.cfg["radio_giro"] + 0.02
        d = holg.get("giro_dir") if holg else None
        if encontro or d is None or total >= BUSCAR_MAX or c["dt"] > 8.0:
            self.m["buscado"] = total if encontro else BUSCAR_MAX
            self.m["dist"] = self._d_borde(c["P"])          # (VERIFICAR compara contra esto)
            self._ir("GIRAR", c["ahora"])
            return None
        # direccion contraria a lo que estorba, en el marco del perro; nunca hacia el borde
        ex, ey = -math.cos(d), -math.sin(d)
        yaw = c["yaw"]
        n = self.m.get("n")
        if n is not None:
            mundo = np.array([math.cos(yaw) * ex - math.sin(yaw) * ey, math.sin(yaw) * ex + math.cos(yaw) * ey])
            hacia_borde = float(np.dot(mundo, n))
            if hacia_borde > 0.0:                         # quitarle la parte que va hacia la bajada
                mundo = mundo - hacia_borde * n
                ex = math.cos(yaw) * mundo[0] + math.sin(yaw) * mundo[1]
                ey = -math.sin(yaw) * mundo[0] + math.cos(yaw) * mundo[1]
        L = math.hypot(ex, ey)
        if L < 0.2:                                       # lo unico que queda es ir hacia la bajada: no
            self.m["buscado"] = BUSCAR_MAX
            self.m["dist"] = self._d_borde(c["P"])
            self._ir("GIRAR", c["ahora"])
            return None
        v = 0.12
        return v * ex / L, v * ey / L, 0.0

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

    def _inclinacion(self, c):
        """Pitch y roll promediados T_FILTRO s; marca 'inclinado' (ya en los escalones) y lo mas
        inclinado que fue."""
        hp = self.m.setdefault("hp", [])
        hp.append((c["ahora"], c["pitch"], c["roll"]))
        while hp and c["ahora"] - hp[0][0] > T_FILTRO:
            hp.pop(0)
        pf = sum(p for _t, p, _r in hp) / len(hp)
        rf = sum(r for _t, _p, r in hp) / len(hp)
        if abs(pf) > INCLINADO:
            self.m.setdefault("t_incl", c["ahora"])
            if c["ahora"] - self.m["t_incl"] >= T_INCLINADO:
                self.m["inclinado"] = True
            self.m["incl_max"] = max(self.m.get("incl_max", 0.0), abs(pf))
        else:
            self.m.pop("t_incl", None)
        return pf, rf

    def _plano(self, pf, rf):
        """Cuerpo plano o casi: menos del 35 % de lo mas inclinado (entre 5.5 y 8 grados)."""
        lim = min(PLANO_MAX, max(PLANO_MIN, 0.35 * self.m.get("incl_max", 0.0)))
        return abs(pf) < lim and abs(rf) < math.radians(7)

    def _rumbo_escalera(self, c, yref, en_escalones):
        """Rumbo escalera-arriba (odom) y centro de la escalera (de lado, m) segun lo que el lidar ve
        delante (Detector.orient: contrahuellas y paredes). Suavizado, y sin alejarse mas de 20
        grados del rumbo medido al empezar. Bajando, recien cuenta ya en los escalones (antes el
        perro mira el pasillo de arriba, no la escalera)."""
        ye = self.m.get("yaw_esc", yref)
        centro = None
        o = c["det"].orient if c["det"] is not None else None
        if (en_escalones and o is not None and c["ahora"] - o["t"] < 0.6 and (o["contras"] >= 2 or o["paredes"])):
            med = ang(o["yaw"] + o["ang"])
            if abs(ang(med - yref)) < math.radians(20):
                if abs(ang(med - ye)) < math.radians(10):
                    ye = ang(ye + 0.25 * ang(med - ye))
                    self.m["raros"] = 0
                else:                                  # muy distinto: solo si se repite
                    self.m["raros"] = self.m.get("raros", 0) + 1
                    if self.m["raros"] >= 6:
                        ye, self.m["raros"] = med, 0
                centro = o["centro"]
        self.m["yaw_esc"] = ye
        return ye, centro

    def _centrar_linea(self, P, ye, centro, izq, der):
        """vy para ir por el CENTRO de la escalera. La linea del centro se mide antes de empezar
        (MEDIR: entre las paredes o con el ancho de la escalera) y se sigue con la ODOMETRIA: asi se
        corrige aunque el lidar no vea nada al costado (bajando de reversa, al principio la escalera
        y sus paredes quedan detras). Lo que se ve delante (centro de la escalera) corrige la linea
        de a poco; con paredes a los dos lados, mandan ellas."""
        t = np.array([-math.sin(ye), math.cos(ye)])
        L0 = self.m.get("linea")
        if L0 is None:
            L0 = np.array(self.m.get("meta_lat", P), float)
        if centro is not None:
            L0 = L0 + 0.2 * float(np.dot(P + centro * t - L0, t)) * t
        self.m["linea"] = L0
        e = float(np.dot(L0 - P, t))
        if max(izq, der) < 0.40:
            e = (izq - der) / 2.0
        self.m["error_lateral"] = e
        return _lim(0.8 * e, 0.08)

    def _en_escalones(self, c, bajando):
        """Encima de los escalones: avanza (hacia atras si baja) DERECHO y CENTRADO segun la escalera
        que ve delante, y espera si algo se cruza. Termina en cuanto el cuerpo vuelve a quedar plano
        (o casi) despues de ir inclinado."""
        ahora, P, z, yaw, holg = c["ahora"], c["P"], c["z"], c["yaw"], c["holg"]
        if self.m.get("P0") is None:           # retomado despues de DETENIDO
            self.m.update(P0=P.copy(), mov=(ahora, P.copy()), hz=[])
        yref = self.m["yaw_b"] if bajando else self.m["yaw_n"]
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
        pf, rf = self._inclinacion(c)
        nivelado = self._plano(pf, rf)
        estable = (len(hz) > 8 and hz[-1][0] - hz[0][0] > 1.3
                   and max(v for _, v in hz) - min(v for _, v in hz) < 0.03)
        en_escalones = not bajando or self.m.get("inclinado") or self.m.get("piso_cambio")
        ye, centro = self._rumbo_escalera(c, yref, en_escalones)
        w = self._w_rumbo(ye, yaw)
        self.info = "%s %.2f m, recorrido %.2f m, inclinado %.0f" % (
            "bajado" if bajando else "subido", abs(dz), s, math.degrees(abs(pf)))
        izq, der = (holg.get("pared_izq", math.inf), holg.get("pared_der", math.inf)) if holg else (math.inf, math.inf)
        vy = self._centrar_linea(P, ye, centro, izq, der)
        self.info += ", corrido %+.0f cm del centro" % (-100 * self.m["error_lateral"])
        if izq < 0.05:                         # rozando una pared: lejos de ella si o si
            vy = min(vy, -0.06)
        if der < 0.05:
            vy = max(vy, 0.06)
        # (subiendo, los escalones mismos son "obstaculos": solo cuenta algo mas alto)
        bloqueo = bool(holg) and ((holg["atras"][0] < 0.10) if bajando
                                  else (holg.get("alto_delante", 9.0) < 0.25))
        if bloqueo:
            # si es una baranda de un lado, se corre al otro; si no, espera (persona)
            if abs(izq - der) > 0.08 and max(izq, der) > 0.10:
                self.info += " (me corro de la baranda)"
                return 0.0, (0.08 if izq > der else -0.08), w
            self.info += " (algo en el camino, espero)"
            return None
        fin = self._fin_escalera(c, bajando, nivelado, estable, dz, pf, s)
        if fin is not None:
            return fin
        t0, p0 = self.m["mov"]
        if np.linalg.norm(P - p0) > 0.04:
            self.m["mov"] = (ahora, P.copy())
        elif ahora - t0 > 6.0 and self.empuje_al_maximo():
            # (solo si ya se le pidio lo maximo: antes se rendia a 0.15 m/s, donde el Go2 pisa en el sitio)
            if not self.m["piso_cambio"] and not self.m.get("inclinado"):
                self.abortar("el perro no avanza en la escalera (revisa la marcha / firmware)", ahora)
            else:
                self.detener("no avanza", ahora)
            return None
        if bajando and not self.m["piso_cambio"] and not self.m.get("inclinado") and s > self.m.get("dist", 0.7) + 0.9:
            self.abortar("pase el borde y no baje: no era escalera", ahora)
            return None
        if s > RECORRIDO_MAX:
            self.detener("recorrio %.1f m y no termino la escalera" % s, ahora)
            return None
        v = self.cfg["v_bajar"] if bajando else self.cfg["v_subir"]
        g = self.m.get("emp")
        if g and g["k"] > 1.0:
            self.info += " (empujando %.2f m/s: no avanzaba a %.2f)" % (min(v * g["k"], self.cfg.get("v_escalera_max", 0.40)), v)
        return sgn * v, vy, w

    def _fin_escalera(self, c, bajando, nivelado, estable, dz, pf, s):
        """Fin de la escalera, subiendo o bajando: el cuerpo volvio a quedar PLANO (o casi) despues de
        ir inclinado -> para EN SECO, dice LLEGUE ARRIBA / LLEGUE ABAJO y quita la marcha. Si el pitch
        nunca se movio: la altura ya cambio y se estabilizo. Primero frena y despues comprueba quieto
        T_PLANO s: si era un vaiven, sigue. Subiendo, si delante hay mas escalones AHORA (un descanso),
        sigue subiendo. Antes, arriba seguia de frente: miraba si "habia escalera delante" con lo
        visto en los ultimos 4 s (la misma escalera que acababa de subir), y despues caminaba 30 cm
        mas; y exigia que la odometria marcara 10 cm de altura.
        Devuelve (0, 0, 0) mientras comprueba, None si hay que seguir."""
        ahora = c["ahora"]
        cambio = (dz < -0.10) if bajando else (dz > 0.10)
        llego = ((self.m.get("inclinado") and nivelado and (bajando or cambio or s > 0.8))
                 or (cambio and estable and abs(pf) < PLANO_MAX))
        if llego and not bajando:
            sub, t_sub = c["det"].subida_frente if c["det"] is not None else (None, -1e9)
            if sub is not None and ahora - t_sub < 0.6 and sub["dist"] < 1.2:
                llego = False                       # descanso con mas escalones delante: sigue subiendo
                self.info += " (descanso: hay mas escalones delante)"
        if not llego:
            self.m.pop("quieto", None)
            if self.m.pop("t_plano", None) is not None:
                self.info += " (no era el final: sigo)"
            return None
        self.m.setdefault("t_plano", ahora)
        # quieto de verdad antes de cambiar de marcha (cambiarla todavia andando da un tiron)
        q = self.m.setdefault("quieto", [ahora, c["P"].copy()])
        if float(np.linalg.norm(c["P"] - q[1])) > 0.006:
            q[0], q[1] = ahora, c["P"].copy()
        quieto = ahora - q[0] >= 0.3
        if ahora - self.m["t_plano"] < T_PLANO or (not quieto and ahora - self.m["t_plano"] < 1.5):
            self.info += " (cuerpo plano: freno y compruebo)"
            return 0.0, 0.0, 0.0
        if bajando:
            self.log("LLEGUE ABAJO: cuerpo plano, paro de ir para atras y me quedo mirando la escalera (%s)" % self.info)
        else:
            self.log("LLEGUE ARRIBA: cuerpo plano, paro (%s)" % self.info)
        self.m["dz"] = c["z"] - self.m["z0"]
        self.m["espera"] = self.robot.modo_escaleras(False)
        self.m["marcha"] = False
        self.m["bal"] = False
        self._ir("MARCHA_OFF", ahora)
        return 0.0, 0.0, 0.0

    def _e_marcha_off(self, c):
        if c["dt"] > self.m["espera"] and not self.m.get("bal"):
            self.robot.balancear()
            self.m["bal"] = True
        if c["dt"] > self.m["espera"] + 0.6:
            self._ir("LISTO", c["ahora"])

    def _e_listo(self, c):
        dz = float(self.m.get("dz", 0.0))
        self._fin(True, "%s %.2f m" % ("LLEGUE ABAJO: baje" if self.bajando else "LLEGUE ARRIBA: subi", abs(dz)), c["ahora"],
                  dz=dz, yaw_n=self.m["yaw_n"], E=[float(v) for v in self.m["E"]])

    def _e_restaurar(self, c):
        """Abortada con la marcha puesta: la quita y recien entonces termina."""
        if c["dt"] > self.m["espera"] and not self.m.get("bal_r"):
            self.robot.balancear()
            self.m["bal_r"] = True
        if c["dt"] > self.m["espera"] + 0.6:
            self._fin(False, self.m.get("motivo_abortar", "cancelada"), c["ahora"])
