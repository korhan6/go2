"""Control PS4 (DualShock 4) -> ordenes para el perro. Logica pura (sin ROS) para poder probarla.

    mantener L1 + stick izq   avanzar / de lado      mantener L1 + stick der   girar
    R1 (con L1)               rapido                 soltar L1                 el perro se para
    CIRCULO                   PARAR TODO (exploracion, ruta, escalera)  -- funciona SIN L1
    SHARE                     pausa / seguir (exploracion o ruta)
    L1 + TRIANGULO            levantarse             L1 + OPTIONS              subir la escalera de delante
    L1 + PS (boton central)   bajar la escalera de delante (gira 180 y baja en reversa)
    L1 + CUADRADO             guardar un punto aqui (control_1, control_2, ...)

El orden de botones/ejes que entrega el nodo 'joy' depende del driver de Linux:
  kernel (hid-playstation / hid-sony, evdev): 13 botones, ejes LX LY L2 RX RY R2 (+ cruceta)
  SDL (HIDAPI / GameController):              15-16 botones, ejes LX LY RX RY L2 R2
'auto' lo elige por el numero de botones; 'go2 control' mide TU control y lo guarda.
"""

LAYOUTS = {
    "kernel": {"botones": {"cruz": 0, "circulo": 1, "triangulo": 2, "cuadrado": 3, "l1": 4, "r1": 5,
                           "share": 8, "options": 9, "ps": 10},
               "ejes": {"vx": 1, "vy": 0, "wz": 3}},
    "sdl": {"botones": {"cruz": 0, "circulo": 1, "cuadrado": 2, "triangulo": 3, "share": 4, "ps": 5,
                        "options": 6, "l1": 9, "r1": 10},
            "ejes": {"vx": 1, "vy": 0, "wz": 2}},
}
# el nodo joy de ROS 2 da arriba = +1 e izquierda = +1 en los sticks: coincide con ROS (x adelante,
# y izquierda, giro antihorario) -> signo +1. 'go2 control' lo mide igual.
SIGNOS = {"vx": 1.0, "vy": 1.0, "wz": 1.0}

FRENO = 0.20          # m desde el borde del cuerpo: igual que 'go2 manejar'


def elegir_layout(n_botones, propio=None):
    """Indices de botones/ejes: el medido con 'go2 control' (propio) o el que corresponde al driver."""
    if propio and propio.get("botones") and propio.get("ejes"):
        return "propio", propio
    if n_botones == 13:
        return "kernel", LAYOUTS["kernel"]
    if n_botones in (15, 16, 17):
        return "sdl", LAYOUTS["sdl"]
    return None, LAYOUTS["sdl"]


def zona_muerta(v, dz=0.10):
    if abs(v) < dz:
        return 0.0
    return (v - dz * (1 if v > 0 else -1)) / (1.0 - dz)


class Mando(object):
    """Traduce cada mensaje del control en (orden de velocidad | None, [acciones]).

    orden None = no mandar nada (Nav2/gestor mandan). Al soltar L1 se manda UNA orden cero.
    acciones: 'parar', 'pausa_seguir', 'levantar', 'subir', 'bajar', 'guardar'.
    """

    def __init__(self, layout=None, signos=None, v_normal=(0.40, 0.25, 0.8), v_rapido=(0.60, 0.35, 1.1)):
        self.layout = layout
        self.signos = dict(SIGNOS, **(signos or {}))
        self.v_normal, self.v_rapido = v_normal, v_rapido
        self.antes = []
        self.manejando = False
        self.aviso = ""

    def _b(self, botones, nombre):
        i = self.layout["botones"].get(nombre)
        return i is not None and 0 <= i < len(botones) and botones[i] == 1

    def _apreto(self, botones, nombre):
        i = self.layout["botones"].get(nombre)
        if i is None or not (0 <= i < len(botones)) or botones[i] != 1:
            return False
        return not (i < len(self.antes) and self.antes[i] == 1)

    def _eje(self, ejes, k):
        i = self.layout["ejes"].get(k)
        v = ejes[i] if i is not None and 0 <= i < len(ejes) else 0.0
        return self.signos.get(k, 1.0) * zona_muerta(v)

    def paso(self, ejes, botones, holguras=None):
        if self.layout is None:
            self.layout = elegir_layout(len(botones))[1]
        acciones = []
        l1 = self._b(botones, "l1")
        if self._apreto(botones, "circulo"):
            acciones.append("parar")                    # siempre, sin hombre muerto
        if self._apreto(botones, "share"):
            acciones.append("pausa_seguir")
        if l1:
            for boton, accion in (("triangulo", "levantar"), ("options", "subir"), ("ps", "bajar"),
                                  ("cuadrado", "guardar")):
                if self._apreto(botones, boton):
                    acciones.append(accion)
        self.antes = list(botones)

        self.aviso = ""
        if not l1:
            if self.manejando:
                self.manejando = False
                return (0.0, 0.0, 0.0), acciones          # al soltar L1: un cero explicito
            return None, acciones
        self.manejando = True
        vmx, vmy, wm = self.v_rapido if self._b(botones, "r1") else self.v_normal
        vx = self._eje(ejes, "vx") * vmx
        vy = self._eje(ejes, "vy") * vmy
        wz = self._eje(ejes, "wz") * wm
        H = holguras or {}
        if vx > 0 and H.get("adelante") is not None and H["adelante"] < FRENO:
            vx, self.aviso = 0.0, "algo delante: no avanzo"
        if vx < 0 and H.get("atras") is not None and H["atras"] < FRENO:
            vx, self.aviso = 0.0, "algo detras: no retrocedo"
        if vy > 0 and H.get("izq") is not None and H["izq"] < FRENO:
            vy, self.aviso = 0.0, "algo a la izquierda"
        if vy < 0 and H.get("der") is not None and H["der"] < FRENO:
            vy, self.aviso = 0.0, "algo a la derecha"
        return (vx, vy, wz), acciones
