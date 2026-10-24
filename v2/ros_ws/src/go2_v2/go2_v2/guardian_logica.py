"""Logica del guardian (sin ROS: se prueba sola, v2/pruebas/prueba_guardian.py).

El guardian es el UNICO que le habla al perro (/api/sport/request). Todo lo demas (el planificador
local de CMU, el control PS4, 'go2v2 ...') pasa por aca:

  /cmd_vel (TwistStamped, pathFollower de CMU, ~100 Hz)
      -> recorte a los limites
      -> Move {"x","y","z"} a 50 Hz mientras la orden este fresca
      -> si la orden deja de llegar (nodo caido, red, lo que sea) por mas de 'vigencia' s:
         StopMove YA (el perro NO sigue caminando con la ultima orden)
  Estados que cortan todo: ACOSTADO (echado: un Move lo levantaria), BLOQUEADO (de pie trabado),
  PAUSA (freno de emergencia con CIRCULO), y mientras se para / se echa (secuencia de postura).
  Ademas para si la ODOMETRIA (Point-LIO) o el MAPA DE TERRENO dejan de llegar: el pathFollower de
  CMU sigue publicando la ultima velocidad aunque Point-LIO se caiga (caminaria a ciegas). En modo
  MANUAL (R2 sostenido) esas dos revisiones no aplican: es la salida de emergencia de CMU.

Botones del PS4 que YA usa CMU (no tocar): L1 = borrar el grafo de rutas (FAR), R1 = borrar el mapa
de terreno, L2 = modo autonomo, R2 = modo manual. Por eso las combinaciones van con SHARE.

Cada paso devuelve la lista de pedidos (api_id, parametros|None) a mandar.
"""
API_DAMP = 1001
API_BALANCESTAND = 1002
API_STOPMOVE = 1003
API_STANDUP = 1004              # "lock stand": de pie con las juntas trabadas
API_STANDDOWN = 1005            # echarse
API_RECOVERYSTAND = 1006
API_MOVE = 1008
API_FREEWALK = 2045             # firmware >= 1.1.7: marcha agil (la usaba la v1 al levantarse)

# control PS4 (driver hid-playstation de Linux, 13 botones)
BOTON = {"cruz": 0, "circulo": 1, "triangulo": 2, "cuadrado": 3, "l1": 4, "r1": 5,
         "l2": 6, "r2": 7, "share": 8, "options": 9, "ps": 10}
EJE_R2 = 5                      # gatillo R2: < -0.1 = sostenido (modo manual de CMU)


def recortar(v, lim):
    return max(-lim, min(lim, v))


class Guardian(object):
    def __init__(self, vx_max=1.0, vy_max=0.6, wz_max=1.5, vigencia=0.25, periodo=0.02,
                 freewalk=True, zona_muerta=0.01, v_min_caminar=0.0, odom_max=0.6, terreno_max=1.5):
        self.vx_max, self.vy_max, self.wz_max = vx_max, vy_max, wz_max
        self.vigencia = vigencia
        self.periodo = periodo
        self.freewalk = freewalk
        self.zona_muerta = zona_muerta
        # el Go2 casi no avanza por debajo de ~0.2 m/s (pisa en el sitio). 0 = no tocar la orden
        self.v_min_caminar = v_min_caminar
        self.odom_max = odom_max            # s: edad maxima de la odometria con la que se calculo la orden
        self.terreno_max = terreno_max      # s: sin mapa de terreno nuevo -> la evasion no ve nada
        self.edad_odom = None
        self.t_terreno = None
        self.manual = False                 # R2 sostenido: modo manual de CMU (sin evasion)
        self.quieto_hasta = -1e9            # durante una secuencia de postura no se camina
        self.motivo_parada = None           # por que esta parado (odometria / terreno), para avisar una vez
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = -1e9
        self.acostado = False
        self.bloqueado = False
        self.pausa = False
        self.moviendo = False          # la ultima orden mandada fue un Move distinto de cero
        self.t_stop = -1e9
        self.stops_pendientes = 0
        self.agenda = []                # [(t, api, parametros)]: secuencias (levantar, acostar...)
        self.botones_prev = []
        self.avisos = []                # textos para el log
        self.n_move = 0
        self.n_stop = 0
        self.n_vencidas = 0

    # ------------------------------------------------------------------ entradas
    def orden(self, t, vx, vy, wz, edad_odom=None):
        """edad_odom: s desde que se midio la odometria con la que el planificador calculo esta
        orden (el pathFollower sella /cmd_vel con la hora de la odometria)."""
        self.cmd = (vx, vy, wz)
        self.t_cmd = t
        self.edad_odom = edad_odom

    def terreno(self, t):
        self.t_terreno = t

    def _cortar(self, t):
        """Deja de caminar YA: StopMove ahora y otro un poco despues (por si se pierde uno)."""
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = -1e9
        self.moviendo = False
        self.stops_pendientes = 1          # uno de respaldo 0.15 s despues
        self.t_stop = t
        return [(API_STOPMOVE, None)]

    def _soltar(self):
        """Olvida la orden en curso sin mandar nada (antes de pararse: un StopMove a mitad de un
        StandUp podria cortarlo)."""
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = -1e9
        self.moviendo = False
        self.stops_pendientes = 0

    def acostar(self, t):
        self.acostado = True
        self.agenda = [(t + 0.5, API_STANDDOWN, None)]
        self.quieto_hasta = t + 2.0
        self.avisos.append("ACOSTADO: no camina hasta 'levantar' (L1+TRIANGULO o go2v2 levantar)")
        return self._cortar(t)

    def levantar(self, t):
        self.acostado = False
        self.bloqueado = False
        self.pausa = False
        self.agenda = [(t, API_STANDUP, None), (t + 1.5, API_BALANCESTAND, None)]
        if self.freewalk:
            self.agenda.append((t + 2.5, API_FREEWALK, None))
        # que no arranque a caminar a medio pararse (el planificador puede tener una orden lista)
        self.quieto_hasta = t + (3.0 if self.freewalk else 2.0)
        self.avisos.append("levantando")
        self._soltar()
        return []

    def recuperar(self, t):
        """Se cayo o quedo raro: RecoveryStand (se para desde cualquier posicion) y equilibrio."""
        self.acostado = False
        self.bloqueado = False
        self.agenda = [(t, API_RECOVERYSTAND, None), (t + 2.5, API_BALANCESTAND, None)]
        if self.freewalk:
            self.agenda.append((t + 3.5, API_FREEWALK, None))
        self.quieto_hasta = t + (4.0 if self.freewalk else 3.0)
        self.avisos.append("recuperando (RecoveryStand)")
        self._soltar()
        return []

    def bloquear(self, t):
        self.bloqueado = True
        self.agenda = [] if self.acostado else [(t + 0.5, API_STANDUP, None)]
        self.avisos.append("BLOQUEADO: quieto y trabado (L1+SHARE o go2v2 desbloquear)")
        return self._cortar(t)

    def desbloquear(self, t):
        if not self.bloqueado:
            return []
        self.bloqueado = False
        if not self.acostado:
            self.agenda = [(t, API_BALANCESTAND, None)]
            if self.freewalk:
                self.agenda.append((t + 1.0, API_FREEWALK, None))
            self.quieto_hasta = t + 1.5
        self.avisos.append("desbloqueado")
        return []

    def alternar_pausa(self, t):
        self.pausa = not self.pausa
        self.avisos.append("PAUSA: freno de emergencia (CIRCULO otra vez para seguir)" if self.pausa
                           else "pausa terminada: sigo")
        return self._cortar(t) if self.pausa else []

    def joystick(self, t, botones, ejes=()):
        """Combinaciones del PS4 (flanco de subida), con SHARE sostenido (L1 y R1 son de CMU):
        SHARE + CRUZ = acostar, SHARE + TRIANGULO = levantar, SHARE + CUADRADO = bloquear/desbloquear,
        SHARE + CIRCULO = recuperar (RecoveryStand). CIRCULO solo = pausa (freno de emergencia)."""
        prev = self.botones_prev
        self.botones_prev = list(botones)
        self.manual = len(ejes) > EJE_R2 and ejes[EJE_R2] < -0.1

        def apretado(nombre):
            i = BOTON[nombre]
            return i < len(botones) and botones[i] and not (i < len(prev) and prev[i])

        def sostenido(nombre):
            i = BOTON[nombre]
            return i < len(botones) and bool(botones[i])
        out = []
        if sostenido("share"):
            if apretado("cruz"):
                out += self.acostar(t)
            elif apretado("triangulo"):
                out += self.levantar(t)
            elif apretado("cuadrado"):
                out += self.desbloquear(t) if self.bloqueado else self.bloquear(t)
            elif apretado("circulo"):
                out += self.recuperar(t)
        elif apretado("circulo"):
            out += self.alternar_pausa(t)
        return out

    def _sin_datos(self, t):
        """Por que no se puede caminar ahora (None = se puede)."""
        if self.manual:
            return None
        if self.edad_odom is not None and self.edad_odom > self.odom_max:
            return "la odometria (Point-LIO) tiene %.1f s de atraso" % self.edad_odom
        if self.t_terreno is None:
            return "todavia no llega el mapa de terreno (la evasion no ve nada)"
        if t - self.t_terreno > self.terreno_max:
            return "no llega el mapa de terreno hace %.1f s (la evasion no ve)" % (t - self.t_terreno)
        return None

    # ------------------------------------------------------------------ salida
    def paso(self, t):
        """Lo que hay que mandarle al perro ahora (llamar cada 'periodo' s)."""
        out = []
        while self.agenda and self.agenda[0][0] <= t:
            _, api, par = self.agenda.pop(0)
            out.append((api, par))
        if self.stops_pendientes and t - self.t_stop >= 0.15:
            out.append((API_STOPMOVE, None))
            self.stops_pendientes -= 1
            self.t_stop = t
            self.n_stop += 1
        if self.acostado or self.bloqueado or self.pausa or t < self.quieto_hasta:
            return out
        fresca = t - self.t_cmd <= self.vigencia
        if not fresca:
            if self.moviendo:
                # la orden se vencio en pleno movimiento: parar YA
                self.n_vencidas += 1
                self.avisos.append("sin ordenes del planificador hace %.2f s: StopMove" % (t - self.t_cmd))
                out += self._cortar(t)
                self.n_stop += 1
            return out
        motivo = self._sin_datos(t)
        if motivo is not None:
            if self.motivo_parada is None:
                self.avisos.append("PARO: " + motivo)       # una vez, no 50 por segundo
            self.motivo_parada = motivo
            if self.moviendo:
                out += self._cortar(t)
                self.n_stop += 1
            return out
        if self.motivo_parada is not None:
            self.avisos.append("datos al dia otra vez: sigo")
            self.motivo_parada = None
        # avance y lateral se escalan JUNTOS: recortarlos por separado le cambiaria la direccion a la
        # diagonal con la que el planificador esquiva (y podria rozar lo que estaba evitando)
        vx, vy = self.cmd[0], self.cmd[1]
        k = 1.0
        if abs(vx) > self.vx_max:
            k = min(k, self.vx_max / abs(vx))
        if abs(vy) > self.vy_max:
            k = min(k, self.vy_max / abs(vy))
        vx, vy = vx * k, vy * k
        wz = recortar(self.cmd[2], self.wz_max)
        z = self.zona_muerta
        if abs(vx) < z and abs(vy) < z and abs(wz) < z:
            if self.moviendo:
                out += self._cortar(t)
                self.n_stop += 1
            return out
        if self.v_min_caminar > 0:
            v = (vx * vx + vy * vy) ** 0.5
            if 0.05 < v < self.v_min_caminar:
                k = self.v_min_caminar / v
                vx, vy = vx * k, vy * k
        self.moviendo = True
        self.stops_pendientes = 0
        self.n_move += 1
        out.append((API_MOVE, {"x": round(vx, 4), "y": round(vy, 4), "z": round(wz, 4)}))
        return out
