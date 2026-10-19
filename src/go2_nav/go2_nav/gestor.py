"""go2_gestor: el cerebro. Explora (varios pisos), toma escaleras, guarda puntos y va a ellos.

Exploracion por fronteras CON COMPROMISO:
  - elige UNA frontera (la mas cercana por camino real) y la mantiene hasta llegar, hasta que esa
    zona ya se vio, o hasta que Nav2 diga que no se puede (entonces la veta);
  - nunca cambia de meta solo porque aparecio otra mas cerca.

Escaleras (go2_escaleras las detecta y las cruza):
  - cada escalera vista queda como "portal" del piso;
  - con escaleras sin usar el mapeo NO esta completo: al terminar el piso va a la escalera,
    sube DE FRENTE o baja EN REVERSA (gira 180 antes del borde), mapea el otro piso y vuelve;
  - un mapa de slam_toolbox POR PISO: antes de cruzar guarda el del piso; en un piso nuevo
    empieza otro; al volver a un piso conocido lo recarga en el lugar correcto.
  - 'go2 ir A' funciona aunque A este en otro piso (busca las escaleras que llevan alla).
"""
import collections
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener

from go2_nav_interfaces.srv import Comando

from .nube import leer_xyz

try:
    from slam_toolbox.srv import DeserializePoseGraph, Reset, SerializePoseGraph
except ImportError:          # sin slam_toolbox: un solo piso
    DeserializePoseGraph = Reset = SerializePoseGraph = None

DATOS = os.path.expanduser("~/.go2_nav")


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Gestor(Node):
    VRES, VN = 0.10, 1000           # rejilla de "suelo visto": 100 x 100 m, 10 cm

    def __init__(self):
        super().__init__("go2_gestor")
        p = self.declare_parameter
        self.volver = p("volver_al_terminar", True).value
        self.frontera_min = p("frontera_min_celdas", 5).value     # 0.5 m de borde (una puerta)
        # huecos sin ver mas chicos que esto (detras de un mueble, sombra del lidar) NO son zonas a
        # explorar: no valen el viaje y hacian que nunca saliera del cuarto
        self.hueco_min = p("hueco_min_m2", 0.5).value
        # el piso solo cuenta como "visto" cerca del perro: el L1 ve el piso de OTROS cuartos de
        # lejos (por la puerta, bajo los muebles) y los daba por explorados sin entrar
        self.radio_visto = p("radio_visto", 2.5).value
        self.holgura_meta = p("holgura_meta", 0.22).value
        self.t_max_meta = p("t_max_meta", 120.0).value
        self.d_min_meta = p("distancia_min_meta", 0.7).value
        self.politica = p("escaleras", "al_final").value      # al_final | inmediato | off
        self.dz_piso = p("altura_entre_pisos_min", 0.6).value
        self.t_atasco = p("t_atascado", 20.0).value          # s sin moverse con una meta = atascado
        self.auto_calibrar = p("calibrar_antes_de_explorar", True).value
        # requerida: una escalera del lidar solo cuenta si la CAMARA (YOLO) la confirma | off
        self.esc_vision = p("escaleras_vision", "requerida").value
        os.makedirs(os.path.join(DATOS, "mapas"), exist_ok=True)
        self.ruta_puntos = os.path.join(DATOS, "puntos.json")
        self.puntos = self._cargar_puntos()

        cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.lock_rej = threading.Lock()      # la rejilla de lo visto se escribe desde otros hilos
        self.errores = 0
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        qmap = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.cb_mapa, qmap, callback_group=cb)
        self.create_subscription(Odometry, "odom", self.cb_odom, 10, callback_group=cb)
        self.create_subscription(PointCloud2, "/go2/suelo", self.cb_suelo, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(PointCloud2, "/go2/obstaculos", self.cb_obst, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(String, "/go2/escaleras", self.cb_escaleras, 1, callback_group=cb)
        self.create_subscription(String, "/go2/vision", self.cb_vision, 1, callback_group=cb)
        self.create_subscription(String, "/go2/escalera_estado", lambda m: self._json("escalera", m), 1, callback_group=cb)
        self.create_subscription(String, "/go2/limpiador_estado", lambda m: self._json("limpiador", m), 1, callback_group=cb)
        self.create_subscription(String, "/go2/driver_estado", lambda m: self._json("driver", m), 1, callback_group=cb)
        self.create_subscription(Bool, "/go2/control_remoto", self.cb_remoto, 1, callback_group=cb)
        self.create_subscription(Float32, "/go2/bateria", lambda m: setattr(self, "bateria", m.data), 1, callback_group=cb)
        self.pub_estado = self.create_publisher(String, "/go2/estado", 1)
        self.pub_pausa_slam = self.create_publisher(Bool, "/go2/pausar_slam", 1)
        self.pub_prio = self.create_publisher(Twist, "cmd_vel_prioridad", 10)
        # lo que el GESTOR cree que ya vio (blanco = suelo visto, negro = obstaculo) y las fronteras
        # (verde) y la meta (amarillo): para ver en RViz por que va (o no va) a algun lado
        self.pub_mapa_v = self.create_publisher(OccupancyGrid, "/go2/mapa_explorado", qmap)
        self.pub_fr = self.create_publisher(MarkerArray, "/go2/fronteras", 1)
        self.ult_fronteras = []
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.cli_calibrar = self.create_client(Trigger, "/go2/calibrar", callback_group=cb)
        self.cli_calibrar_parar = self.create_client(Trigger, "/go2/calibrar_parar", callback_group=cb)
        self.cli_levantar = self.create_client(Trigger, "/go2/levantar", callback_group=cb)
        self.cli_parar = self.create_client(Trigger, "/go2/parar", callback_group=cb)
        self.cli_esc = self.create_client(Comando, "/go2/escalera", callback_group=cb)
        self.cli_limpiar = [self.create_client(ClearEntireCostmap, s, callback_group=cb) for s in
                            ("/global_costmap/clear_entirely_global_costmap",
                             "/local_costmap/clear_entirely_local_costmap")]
        if SerializePoseGraph is not None:
            self.cli_guardar = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map", callback_group=cb)
            self.cli_cargar = self.create_client(DeserializePoseGraph, "/slam_toolbox/deserialize_map", callback_group=cb)
            self.cli_reset = self.create_client(Reset, "/slam_toolbox/reset", callback_group=cb)
        self.create_service(Comando, "/go2/comando", self.srv_comando, callback_group=MutuallyExclusiveCallbackGroup())

        self.modo, self.motivo = "quieto", "esperando ordenes"
        self.mapa = None
        self.odom = None             # (x, y, z, yaw) en odom
        self.info = {"limpiador": {}, "driver": {}, "escalera": {}}
        self.bateria = None
        self.remoto = False
        self.meta = None
        self.goal_handle = None
        self.resultado = None
        self.restante = None
        self.vetadas = []
        self.sin_fronteras = 0
        self.n_fronteras = 0
        self.miradas = 0
        self.logs = collections.deque(maxlen=60)
        # pisos / escaleras
        self.pisos = []              # dict(z, completo, T=(tx, ty, tyaw) map->odom, archivo)
        self.piso = None
        self.rejillas = {}
        self.portales = []           # dict(id, piso, tipo, x, y, yaw, ancho, usado, destino, retorno, fallos)
        self.cruce = None            # escalera en curso
        self.final = None            # punto final cuando hay que cambiar de piso para llegar
        self.pausa_slam = False
        self.visto = self.ocup = None
        self.t_decae = time.monotonic()
        # atascos
        self.ref_mov = None          # (t, x, y, yaw): ultima vez que se movio de verdad
        self.escape = None           # hilo de la maniobra de escape en curso
        self.escapes = []            # (t, x, y) de los ultimos escapes
        self.calibrando = None       # t0 de la calibracion automatica antes de explorar
        self.reintentar = None       # meta a retomar despues de un escape (ir / casa)
        self.t_cancel = 0.0
        self.reintento_vetos = False
        # vision
        self.vision, self.t_vision = {}, 0.0
        self.candidatos = []         # escaleras del lidar esperando que la camara las confirme
        self.descartadas = []        # (piso, x, y): el lidar vio escalera, la camara NO
        self.aviso_sin_vision = 0.0
        self.create_timer(0.5, self.lazo, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self.publicar_estado, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0, self.publicar_explorado, callback_group=MutuallyExclusiveCallbackGroup())
        self.log("go2_gestor listo (escaleras: %s)" % self.politica)

    # ------------------------------------------------------------------ utilidades
    def log(self, txt):
        self.logs.append(time.strftime("%H:%M:%S ") + txt)
        self.get_logger().info(txt)

    def _json(self, k, m):
        try:
            self.info[k] = json.loads(m.data)
        except ValueError:
            return
        d = self.info[k]
        if k == "driver" and isinstance(d, dict) and (d.get("pausas_lidar") or d.get("lidar_mudo_s")):
            self.t_lidar_malo = time.monotonic()

    def _cargar_puntos(self):
        try:
            with open(self.ruta_puntos) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _guardar_puntos(self):
        with open(self.ruta_puntos, "w") as f:
            json.dump(self.puntos, f, indent=1)

    def T_mapa_odom(self):
        """(tx, ty, tyaw): pose del marco odom en el marco map del piso actual."""
        try:
            t = self.tfb.lookup_transform("map", "odom", rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_q(t.transform.rotation))

    @staticmethod
    def odom_a_mapa(T, x, y, yaw=0.0):
        c, s = math.cos(T[2]), math.sin(T[2])
        return T[0] + c * x - s * y, T[1] + s * x + c * y, ang(yaw + T[2])

    def pose(self):
        try:
            t = self.tfb.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_q(t.transform.rotation))

    def cb_mapa(self, m):
        self.mapa = m

    def cb_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y, p.z, yaw_q(m.pose.pose.orientation))
        if self.piso is None:
            self.pisos.append({"z": p.z, "completo": False, "T": None, "archivo": None})
            self._usar_piso(0)

    def cb_remoto(self, m):
        if m.data and not self.remoto and self.modo in ("explorando", "yendo", "volviendo") and self.cruce is None:
            self.log("control remoto en uso: pauso (go2 seguir para retomar)")
            self.pausar("control remoto en uso")
        self.remoto = m.data

    # ------------------------------------------------------------------ Nav2
    def ir_a(self, x, y, yaw, tipo, nombre=None):
        if not self.nav.server_is_ready():
            self.motivo = "Nav2 no responde (navigate_to_pose)"
            return False
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        # stamp 0 = "el TF mas reciente": con la hora actual, si el TF map->odom venia un poco
        # atrasado, el controlador abortaba ("extrapolation into the future") y yo vetaba la meta
        g.pose.header.stamp.sec, g.pose.header.stamp.nanosec = 0, 0
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        (g.pose.pose.orientation.x, g.pose.pose.orientation.y,
         g.pose.pose.orientation.z, g.pose.pose.orientation.w) = quat_yaw(yaw)
        self.seq = getattr(self, "seq", 0) + 1
        seq = self.seq
        self.meta = {"tipo": tipo, "x": x, "y": y, "yaw": yaw, "nombre": nombre, "t0": time.monotonic()}
        self.resultado = None
        self.restante = None
        espera = 1.0 - (time.monotonic() - self.t_cancel)
        if espera > 0:
            # recien cancelada la meta anterior: si la nueva llega a Nav2 antes que la cancelacion,
            # Nav2 cancela la NUEVA (y el gestor la vetaba como 'inalcanzable'). Espero 1 s.
            threading.Timer(espera, lambda: self._enviar(g, seq)).start()
        else:
            self._enviar(g, seq)
        return True

    def _enviar(self, g, seq):
        if seq != self.seq:
            return
        fut = self.nav.send_goal_async(g, feedback_callback=lambda fb: self.cb_feedback(seq, fb))
        fut.add_done_callback(lambda f: self.cb_aceptado(seq, f))

    # cada resultado se asocia a SU meta: el de una meta vieja (cancelada) no cuenta para la nueva
    def cb_feedback(self, seq, fb):
        if seq == self.seq:
            self.restante = round(float(fb.feedback.distance_remaining), 2)

    def cb_aceptado(self, seq, fut):
        gh = fut.result()
        if seq != self.seq:
            if gh.accepted:
                gh.cancel_goal_async()
                self.t_cancel = time.monotonic()
            return
        if not gh.accepted:
            self.resultado = "rechazada"
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self.cb_resultado(seq, f))

    def cb_resultado(self, seq, fut):
        if seq != self.seq:
            return
        st = fut.result().status
        self.resultado = {GoalStatus.STATUS_SUCCEEDED: "ok", GoalStatus.STATUS_ABORTED: "fallo",
                          GoalStatus.STATUS_CANCELED: "cancelada"}.get(st, "fallo")
        self.goal_handle = None

    def cancelar(self):
        self.seq = getattr(self, "seq", 0) + 1
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
                self.t_cancel = time.monotonic()
            except Exception:
                pass
        self.goal_handle = None
        self.meta = None

    def pausar(self, motivo):
        if self.modo in ("explorando", "yendo", "volviendo"):
            self.modo_antes = self.modo
        self.cancelar()
        self.modo, self.motivo = "pausado", motivo
        if self.cli_parar.service_is_ready():
            self.cli_parar.call_async(Trigger.Request())

    def _llamar(self, cli, req, timeout=15.0):
        """Llamada a un servicio esperando la respuesta (desde el lazo, que es otro hilo)."""
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        fut = cli.call_async(req)
        t = time.monotonic()
        while not fut.done() and time.monotonic() - t < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    # ------------------------------------------------------------------ lazo
    def lazo(self):
        t0 = time.monotonic()
        with self.lock:
            try:
                self._lazo()
                self.errores = 0
            except Exception as e:      # nunca morir: reintentar; si se repite, quieto y avisar
                self.errores += 1
                self.log("ERROR en el lazo (%d): %r" % (self.errores, e))
                if self.errores >= 5:
                    self.cancelar()
                    self.modo, self.motivo = "quieto", "error interno (go2 log)"
        if time.monotonic() - t0 > 2.0:
            self.get_logger().warn("lazo lento: %.1f s" % (time.monotonic() - t0))

    def _lazo(self):
        if self.cruce is not None:
            self.paso_cruce()
            return
        if self.escape is not None:
            if self.escape.is_alive():
                self.motivo = "atascado: maniobra de escape"
                return
            self.escape = None
            self.ref_mov = None
            r, self.reintentar = self.reintentar, None
            if r is not None and self.modo in ("yendo", "volviendo"):
                self.ir_a(r["x"], r["y"], r["yaw"], r["tipo"], r.get("nombre"))   # misma meta, de nuevo
        if self.modo in ("explorando", "yendo", "volviendo") and self.revisar_atasco():
            return
        if self.modo == "explorando":
            self.explorar_paso()
        elif self.modo in ("yendo", "volviendo"):
            if self.meta is None and self.final is not None:
                self.ir_punto(self.final)          # despues de cambiar de piso: seguir
                return
            if self.resultado == "ok":
                n = self.meta["nombre"] if self.meta else "?"
                self.log("llegue a '%s'" % n)
                self.modo, self.motivo = ("terminado" if self.modo == "volviendo" else "quieto"), "llegue a '%s'" % n
                self.meta = None
                self.final = None
            elif self.resultado in ("fallo", "rechazada"):
                self.log("no pude llegar a '%s' (Nav2: %s)" % (self.meta["nombre"] if self.meta else "?", self.resultado))
                self.modo, self.motivo = "quieto", "no pude llegar"
                self.meta = None
                self.final = None

    # ------------------------------------------------------------------ atascos
    def limpiar_costmaps(self):
        for c in self.cli_limpiar:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())

    def revisar_atasco(self):
        """Con una meta activa y sin moverse (ni girar) en t_atascado s: cancela, limpia los
        costmaps y hace una maniobra corta hacia donde hay espacio. Tres veces en el mismo sitio
        = pausa y pide ayuda."""
        pose = self.pose()
        if pose is None or self.meta is None or self.remoto:
            self.ref_mov = None
            return False
        ahora = time.monotonic()
        r = self.ref_mov
        if r is None or math.hypot(pose[0] - r[1], pose[1] - r[2]) > 0.12 or abs(ang(pose[2] - r[3])) > 0.35:
            self.ref_mov = (ahora, pose[0], pose[1], pose[2])
            return False
        if ahora - r[0] < self.t_atasco:
            return False
        self.ref_mov = None
        tipo = self.meta.get("tipo")
        if tipo == "frontera":
            self.vetadas.append((self.meta["x"], self.meta["y"]))
        self.reintentar = dict(self.meta) if self.modo in ("yendo", "volviendo") else None
        self.cancelar()
        self.escapes = [e for e in self.escapes if ahora - e[0] < 180 and math.hypot(e[1] - pose[0], e[2] - pose[1]) < 0.5]
        self.escapes.append((ahora, pose[0], pose[1]))
        if len(self.escapes) > 3:
            self.escapes = []
            self.log("ATASCADO: no salgo de aqui. Muevelo con el control y 'go2 seguir'")
            self.pausar("atascado: muevelo con el control y 'go2 seguir'")
            return True
        L = self.info.get("limpiador") or {}
        self.log("atascado %.0f s sin moverme (adel=%s izq=%s der=%s): maniobra corta y otra ruta"
                 % (self.t_atasco, L.get("adelante"), L.get("izq"), L.get("der")))
        self.escape = threading.Thread(target=self._maniobra_escape, daemon=True)
        self.escape.start()
        return True

    def _maniobra_escape(self):
        def h(k):
            v = (self.info.get("limpiador") or {}).get(k)
            return 1.5 if v is None else v          # nada visto = libre
        L = {k: h(k) for k in ("adelante", "izq", "der")}
        if L["adelante"] >= 0.45:
            acc, v, clave, dur = "adelante", (0.15, 0.0), "adelante", 2.0
        elif max(L["izq"], L["der"]) >= 0.30:
            lado = "izq" if L["izq"] >= L["der"] else "der"
            acc, v, clave, dur = "de lado (%s)" % lado, (0.0, 0.15 if lado == "izq" else -0.15), lado, 1.6
        else:
            # el L1 no ve atras: despacio, y por donde vino (normalmente esta libre)
            acc, v, clave, dur = "atras", (-0.10, 0.0), None, 1.5
        self.log("escape: %s" % acc)
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur and self.modo in ("explorando", "yendo", "volviendo"):
            if clave is not None and h(clave) < 0.08:
                break
            t = Twist()
            t.linear.x, t.linear.y = v
            self.pub_prio.publish(t)
            time.sleep(0.1)
        self.pub_prio.publish(Twist())

    # ------------------------------------------------------------------ explorar
    def explorar_paso(self):
        if self.calibrando is not None:
            L = self.info.get("limpiador") or {}
            est = L.get("calibracion") or ""
            if time.monotonic() - self.calibrando < 3.0 or est.startswith("calibrando"):
                self.motivo = "calibrando antes de explorar (%s)" % (est or "...")
                return
            self.calibrando = None
            self.log("calibracion: %s" % (est or "?"))
            self.limpiar_costmaps()
        pose = self.pose()
        if pose is None or self.mapa is None or self.visto is None:
            self.motivo = "esperando mapa (slam_toolbox) y TF map->base_link"
            return
        if self.meta is None and not self.nav.server_is_ready():
            self.motivo = "esperando a Nav2..."
            return
        m = self.meta
        if m is not None:
            # COMPROMISO: seguir con la misma meta mientras tenga sentido
            if self.resultado == "ok":
                self.meta = None
            elif self.resultado == "rechazada":
                self.motivo = "esperando a Nav2..."
                if time.monotonic() - m["t0"] > 2.0:
                    self.meta = None
            elif self.resultado == "fallo":
                lidar_malo = time.monotonic() - getattr(self, "t_lidar_malo", -1e9) < 15.0
                if m["tipo"] != "mirar" and lidar_malo and m.get("perdonada", 0) < 2:
                    # fallo por cortes del lidar, no porque no se pueda llegar: la misma meta otra vez
                    self.log("Nav2 fallo pero el lidar se estaba cortando: NO la veto, reintento")
                    n = m.get("perdonada", 0) + 1
                    if self.ir_a(m["x"], m["y"], m["yaw"], m["tipo"], m.get("nombre")):
                        self.meta["perdonada"] = n
                    self.motivo = "reintento la meta (lidar cortado)"
                    return
                if m["tipo"] != "mirar":
                    self.log("frontera inalcanzable: la veto")
                    self.vetadas.append((m["x"], m["y"]))
                self.meta = None
            elif time.monotonic() - m["t0"] > self.t_max_meta:
                self.log("frontera demasiado lenta (%.0f s): la veto" % self.t_max_meta)
                self.vetadas.append((m["x"], m["y"]))
                self.cancelar()
            elif (m["tipo"] != "mirar" and time.monotonic() - m["t0"] > 8.0
                  and self._zona_vista(m["x"], m["y"], r=0.6, max_sin_ver=6)):
                self.cancelar()           # la zona ya se descubrio por el camino (toda, no un pedacito)
                self.log("la zona de la meta ya se vio por el camino: siguiente")
            else:
                self.motivo = "explorando piso %d: voy a zona nueva (%.1f m)" % (self.piso, self.restante or 0.0)
                return
        # escalera nueva y politica 'inmediato': ir ya
        if self.politica == "inmediato":
            p = self.portal_pendiente(solo_nuevos=True)
            if p is not None:
                self.iniciar_cruce(p)
                return
        fr = self.fronteras(pose)
        self.n_fronteras = len(fr)
        if not fr:
            self.sin_fronteras += 1
            if self.sin_fronteras >= 10 and self.miradas < 2:
                # antes de darlo por terminado: una vuelta mirando (el L1 no ve atras)
                self.miradas += 1
                self.sin_fronteras = 0
                self.log("no veo zonas nuevas: giro a mirar antes de terminar el piso")
                self.ir_a(pose[0], pose[1], pose[2] + math.pi * 0.9, "mirar")
                return
            if self.sin_fronteras < 10:
                self.motivo = "buscando zonas sin ver..."
                return
            if self.vetadas and not self.reintento_vetos:
                # antes de darlo por terminado: una segunda oportunidad a lo que fallo (un corte del
                # lidar o un rato sin moverse no deben cerrar una zona para siempre)
                self.reintento_vetos = True
                self.log("antes de terminar, reintento %d zona(s) que fallaron" % len(self.vetadas))
                self.vetadas = []
                self.sin_fronteras = 0
                return
            if not self.pisos[self.piso]["completo"]:
                self.pisos[self.piso]["completo"] = True
                self.log("piso %d completo (%.1f m2)" % (self.piso, self.area_vista()))
            # con escaleras sin usar (o pisos sin terminar) NO esta completo
            p = self.portal_pendiente() or self.piso_pendiente()
            if p is not None:
                self.iniciar_cruce(p)
                return
            self.log("MAPEO COMPLETO: %d piso(s), %d escalera(s)" % (len(self.pisos), len(self.portales)))
            if self.volver and "inicio" in self.puntos:
                self.modo, self.motivo = "volviendo", "mapeo completo, vuelvo al inicio"
                self.ir_punto("inicio")
            else:
                self.modo, self.motivo = "terminado", "mapeo completo"
            return
        self.sin_fronteras = 0
        lejos = [f for f in fr if f["d"] >= self.d_min_meta]
        if not lejos:
            if self.miradas < 2:
                self.miradas += 1
                self.log("zona sin ver junto a mi: giro a mirar")
                self.ir_a(pose[0], pose[1], pose[2] + math.pi * 0.9, "mirar")
                return
            lejos = fr
        self.miradas = 0
        f = lejos[0]
        yaw = math.atan2(f["y"] - pose[1], f["x"] - pose[0])
        self.log("nueva meta de exploracion a %.1f m %s (%d celdas de frontera)"
                 % (f["d"], "en linea recta (Nav2 busca el camino)" if f.get("directa") else "de camino", f["n"]))
        self.ir_a(f["x"], f["y"], yaw, "frontera")

    # ------------------------------------------------------------------ escaleras / pisos
    def cb_vision(self, msg):
        try:
            self.vision = json.loads(msg.data)
            self.t_vision = time.monotonic()
        except ValueError:
            pass

    def vision_viva(self):
        return time.monotonic() - self.t_vision < 3.0 and self.vision.get("fuente") not in (None, "ninguna")

    def juicio_camara(self, v):
        """Lo que dice la camara de una escalera que vio el lidar (v en odom): si | no | espera."""
        if not self.vision_viva() or self.odom is None:
            return "espera"
        ox, oy, _oz, oyaw = self.odom
        rel = ang(math.atan2(v["y"] - oy, v["x"] - ox) - oyaw)
        d = math.hypot(v["x"] - ox, v["y"] - oy)
        E = self.vision.get("escalera") or {}
        if E.get("vista"):
            if "x" in E and math.hypot(E["x"] - v["x"], E["y"] - v["y"]) < 1.0:
                return "si"
            if "rumbo" in E and abs(ang(E["rumbo"] - rel)) < 0.45:
                return "si"
        hfov = self.vision.get("hfov", 2.0)
        if abs(rel) < hfov / 2 - 0.2 and 0.4 < d < 3.5:
            return "no"            # la camara esta mirando ahi y no ve escalera
        return "espera"

    def cb_escaleras(self, msg):
        if self.cruce is not None or self.piso is None or self.politica == "off":
            return
        try:
            vistas = json.loads(msg.data)
        except ValueError:
            return
        T = self.T_mapa_odom()
        if T is None:
            return
        ahora = time.monotonic()
        with self.lock:
            for v in vistas:
                if v["ancho"] < 0.45 and not v.get("abierta"):
                    continue
                x, y, yaw = self.odom_a_mapa(T, v["x"], v["y"], v["yaw"])
                if self.esc_vision == "off":
                    self.agregar_portal(v["tipo"], x, y, yaw, v["ancho"])
                    continue
                if any(p["piso"] == self.piso and p["tipo"] == v["tipo"] and math.hypot(p["x"] - x, p["y"] - y) < 1.2
                       for p in self.portales):
                    self.agregar_portal(v["tipo"], x, y, yaw, v["ancho"])     # ya confirmada: solo afina
                    continue
                if any(d[0] == self.piso and math.hypot(d[1] - x, d[2] - y) < 1.0 for d in self.descartadas):
                    continue
                c = next((c for c in self.candidatos if c["piso"] == self.piso and c["tipo"] == v["tipo"]
                          and math.hypot(c["x"] - x, c["y"] - y) < 1.2), None)
                if c is None:
                    c = {"piso": self.piso, "tipo": v["tipo"], "x": x, "y": y, "si": 0, "no": 0.0, "t": ahora}
                    self.candidatos.append(c)
                    self.log("el lidar cree ver una escalera de %s: la confirmo con la camara" % v["tipo"])
                dt = min(1.0, ahora - c["t"])
                c["t"] = ahora
                j = self.juicio_camara(v)
                if j == "si":
                    c["si"] += 1
                    if c["si"] >= 2:
                        self.candidatos.remove(c)
                        self.log("la CAMARA confirma la escalera de %s (%.0f%%)"
                                 % (v["tipo"], 100 * (self.vision.get("escalera") or {}).get("conf", 0)))
                        self.agregar_portal(v["tipo"], x, y, yaw, v["ancho"])
                elif j == "no":
                    c["no"] += dt
                    # de bajada la camara (a 30 cm del piso) ve poco: le doy mas tiempo
                    if c["no"] > (3.0 if v["tipo"] == "subir" else 6.0):
                        self.candidatos.remove(c)
                        self.descartadas.append((self.piso, x, y))
                        self.log("DESCARTO la escalera de %s del lidar: la camara mira ahi y no ve escalera" % v["tipo"])
                elif not self.vision_viva() and ahora - self.aviso_sin_vision > 30:
                    self.aviso_sin_vision = ahora
                    self.log("sin vision (camara/YOLO): no tomo escaleras solo. 'go2 camara' para revisar")

    def agregar_portal(self, tipo, x, y, yaw, ancho, retorno=False, destino=None):
        for p in self.portales:
            if p["piso"] == self.piso and p["tipo"] == tipo and math.hypot(p["x"] - x, p["y"] - y) < 1.2:
                if not p["usado"]:
                    p["x"] += 0.3 * (x - p["x"])
                    p["y"] += 0.3 * (y - p["y"])
                    p["yaw"] = ang(p["yaw"] + 0.3 * ang(yaw - p["yaw"]))
                return p
        p = {"id": len(self.portales), "piso": self.piso, "tipo": tipo, "x": x, "y": y, "yaw": yaw,
             "ancho": ancho, "usado": False, "destino": destino, "retorno": retorno, "fallos": 0}
        self.portales.append(p)
        if not retorno:
            self.log("escalera de %s vista en el piso %d (portal %d)" % (tipo, self.piso, p["id"]))
        return p

    def destino_portal(self, p):
        """Donde pararse antes de la escalera, mirandola. De bajada, mas cerca del borde (el
        primer escalon solo se ve bien de cerca)."""
        d = 0.70 if p["tipo"] == "bajar" else 0.90
        return p["x"] - d * math.cos(p["yaw"]), p["y"] - d * math.sin(p["yaw"])

    def portal_pendiente(self, solo_nuevos=False):
        if self.politica == "off":
            return None
        pose = self.pose()
        mejor, dmin = None, math.inf
        for p in self.portales:
            if p["piso"] != self.piso or p["usado"] or p["fallos"] >= 3:
                continue
            if p["retorno"] and (solo_nuevos or self.pisos[p["destino"]]["completo"]):
                continue
            d = math.hypot(p["x"] - pose[0], p["y"] - pose[1]) if pose else 0.0
            if d < dmin:
                mejor, dmin = p, d
        return mejor

    def piso_pendiente(self):
        """Escalera de este piso que lleva a un piso que todavia no esta terminado."""
        for p in self.portales:
            if (p["piso"] == self.piso and p["destino"] is not None and p["fallos"] < 3
                    and not self.pisos[p["destino"]]["completo"]):
                return p
        return None

    def ruta_pisos(self, desde, hasta):
        cola, visto = [(desde, [])], {desde}
        while cola:
            piso, ruta = cola.pop(0)
            if piso == hasta:
                return ruta
            for p in self.portales:
                if p["piso"] == piso and p["destino"] is not None and p["destino"] not in visto and p["fallos"] < 3:
                    visto.add(p["destino"])
                    cola.append((p["destino"], ruta + [p]))
        return None

    def iniciar_cruce(self, p):
        ax, ay = self.destino_portal(p)
        self.log("voy a la escalera de %s (portal %d)" % (p["tipo"], p["id"]))
        self.cruce = {"portal": p, "fase": "acercando", "t": time.monotonic(), "piso0": self.piso}
        self.motivo = "voy a la escalera de %s" % p["tipo"]
        pose = self.pose()
        if pose is not None and math.hypot(ax - pose[0], ay - pose[1]) < 0.6:
            # ya esta al pie de la escalera: go2_escaleras se alinea y se acerca solo
            self.meta = None
            self.resultado = "ok"
            return
        # llega con el rumbo del camino (sin giro final lento de Nav2); go2_escaleras se alinea
        yaw = math.atan2(ay - pose[1], ax - pose[0]) if pose and math.hypot(ax - pose[0], ay - pose[1]) > 0.3 \
            else p["yaw"]
        if not self.ir_a(ax, ay, yaw, "escalera"):
            self.cruce = None

    def _fallo_cruce(self, motivo):
        p = self.cruce["portal"]
        p["fallos"] += 1
        self.log("escalera no tomada (%s), intento %d" % (motivo, p["fallos"]))
        self.cruce = None
        self.meta = None
        self._pausar_slam(False)

    def paso_cruce(self):
        c = self.cruce
        p = c["portal"]
        E = self.info.get("escalera") or {}
        if c["fase"] == "acercando":
            if self.resultado == "ok":
                self.meta = None
                # 1) guardar el mapa de este piso y donde esta su marco (para volver)
                self._guardar_piso()
                # 2) el SLAM no recibe nada mientras el perro esta inclinado en la escalera
                self._pausar_slam(True)
                T = self.T_mapa_odom() or (0.0, 0.0, 0.0)
                req = Comando.Request()
                req.orden = "tomar"
                req.args = [p["tipo"], str(ang(p["yaw"] - T[2]))]
                r = self._llamar(self.cli_esc, req, 5.0)
                if r is None or not r.ok:
                    self._fallo_cruce("go2_escaleras no respondio")
                    return
                c.update(fase="cruzando", t=time.monotonic())
                self.log("en la escalera de %s: %s" % (p["tipo"], "subo DE FRENTE" if p["tipo"] == "subir"
                                                       else "giro 180 y bajo EN REVERSA"))
            elif self.resultado in ("fallo", "rechazada") or time.monotonic() - c["t"] > self.t_max_meta:
                self.cancelar()
                self._fallo_cruce("no llegue al pie de la escalera")
            else:
                self.motivo = "voy a la escalera de %s (%.1f m)" % (p["tipo"], self.restante or 0.0)
            return
        if c["fase"] == "cruzando":
            self.motivo = "escalera: %s %s" % (E.get("estado", "?"), E.get("info", ""))
            if E.get("detenido"):
                self.motivo = "DETENIDO en la escalera: %s (go2 seguir)" % E.get("motivo_detenido", "")
            res = E.get("resultado")
            if time.monotonic() - c["t"] < 1.0 or not res or E.get("activa"):
                return
            if not res.get("ok"):
                self._fallo_cruce(res.get("motivo", "?"))
                return
            self._llegue_otro_piso(res)
            c.update(fase="asentando", t=time.monotonic(), res=res)
            return
        if c["fase"] == "asentando":
            # esperar a que el SLAM del piso nuevo publique su marco, y registrar la vuelta
            if time.monotonic() - c["t"] < 3.0:
                self.motivo = "nuevo piso: arrancando su mapa..."
                return
            T = self.T_mapa_odom()
            if T is None:
                return
            res = c["res"]
            x, y, yaw = res["pose_odom"]
            yn = res.get("yaw_n", yaw)
            k = 0.45 if res["tipo"] == "subir" else 0.55
            rx, ry = x - k * math.cos(yn), y - k * math.sin(yn)
            mx, my, myaw = self.odom_a_mapa(T, rx, ry, yn + math.pi)
            rev = self.agregar_portal("bajar" if res["tipo"] == "subir" else "subir", mx, my, myaw,
                                      p.get("ancho", 0.8), retorno=True, destino=c["piso0"])
            rev["destino"], rev["retorno"] = c["piso0"], True
            self.cruce = None
            self.log("ahora en el piso %d (%s)" % (self.piso, "conocido" if c.get("conocido") else "nuevo"))
            if self.modo == "volviendo" and not self.pisos[self.piso]["completo"]:
                self.log("este piso no esta terminado: sigo mapeando")
                self.modo = "explorando"
            self.sin_fronteras = 0
            self.miradas = 0

    def _guardar_piso(self):
        P = self.pisos[self.piso]
        P["T"] = self.T_mapa_odom()
        if SerializePoseGraph is None:
            return
        P["archivo"] = os.path.join(DATOS, "mapas", "piso_%d" % self.piso)
        r = self._llamar(self.cli_guardar, SerializePoseGraph.Request(filename=P["archivo"]), 20.0)
        self.log("mapa del piso %d guardado%s" % (self.piso, "" if r is not None else " (sin respuesta)"))

    def _llegue_otro_piso(self, res):
        p = self.cruce["portal"]
        z = res.get("z", self.odom[2] if self.odom else 0.0)
        nuevo = None
        for k, P in enumerate(self.pisos):
            if abs(P["z"] - z) < self.dz_piso:
                nuevo = k
                break
        conocido = nuevo is not None
        self.cruce["conocido"] = conocido
        if not conocido:
            self.pisos.append({"z": z, "completo": False, "T": None, "archivo": None})
            nuevo = len(self.pisos) - 1
        p["usado"], p["destino"] = True, nuevo
        self.log("escalera LISTA (%s): piso %d -> %d" % (res.get("motivo", ""), self.piso, nuevo))
        # mapa del otro piso: nuevo = empezar de cero; conocido = recargar el suyo donde corresponde
        if SerializePoseGraph is not None:
            P = self.pisos[nuevo]
            if conocido and P.get("archivo") and P.get("T") is not None:
                x, y, yaw = res["pose_odom"]
                mx, my, myaw = self.odom_a_mapa(P["T"], x, y, yaw)
                req = DeserializePoseGraph.Request()
                req.filename = P["archivo"]
                req.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
                req.initial_pose = Pose2D(x=mx, y=my, theta=myaw)
                r = self._llamar(self.cli_cargar, req, 30.0)
                self.log("mapa del piso %d recargado%s" % (nuevo, "" if r is not None else " (sin respuesta)"))
            else:
                r = self._llamar(self.cli_reset, Reset.Request(pause_new_measurements=False), 15.0)
                self.log("piso nuevo: mapa nuevo%s" % ("" if r is not None else " (reset sin respuesta)"))
        for cli in self.cli_limpiar:
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())
        self._usar_piso(nuevo)
        self.mapa = None
        self._pausar_slam(False)

    def _usar_piso(self, k):
        self.piso = k
        if k not in self.rejillas:
            self.rejillas[k] = (np.zeros((self.VN, self.VN), np.uint8), np.zeros((self.VN, self.VN), np.float32))
        with self.lock_rej:
            self.visto, self.ocup = self.rejillas[k]
        self.vetadas = []

    def _pausar_slam(self, si):
        self.pausa_slam = si
        self.pub_pausa_slam.publish(Bool(data=si))

    # ------------------------------------------------------------------ puntos
    def ir_punto(self, nombre):
        p = self.puntos.get(nombre)
        if p is None:
            self.modo, self.motivo = "quieto", "no conozco '%s'" % nombre
            return False
        piso = p.get("piso", 0)
        if piso == self.piso:
            self.final = None
            return self.ir_a(p["x"], p["y"], p["yaw"], "punto", nombre)
        ruta = self.ruta_pisos(self.piso, piso)
        if not ruta:
            self.log("no conozco escaleras que lleven al piso %d (punto '%s')" % (piso, nombre))
            self.modo, self.motivo = "quieto", "sin ruta al piso %d" % piso
            self.final = None
            return False
        self.final = nombre
        self.iniciar_cruce(ruta[0])
        return True

    # ------------------------------------------------------------------ lo que el perro ya vio
    # slam_toolbox solo marca libre hasta donde hay una pared: el suelo que el L1 SI ve pero sin
    # pared detras queda "desconocido" y la exploracion terminaba antes de tiempo. Por eso el
    # gestor lleva su propia rejilla de SUELO VISTO (de /go2/suelo) y de obstaculos, por piso.
    def _a_mapa(self, msg, r_max=None):
        if self.pausa_slam or self.visto is None:
            return None
        try:
            t = self.tfb.lookup_transform("map", msg.header.frame_id or "odom", rclpy.time.Time())
        except Exception:
            return None
        xyz = leer_xyz(msg)
        if len(xyz) == 0:
            return None
        yaw = yaw_q(t.transform.rotation)
        c, s_ = math.cos(yaw), math.sin(yaw)
        x = t.transform.translation.x + c * xyz[:, 0] - s_ * xyz[:, 1]
        y = t.transform.translation.y + s_ * xyz[:, 0] + c * xyz[:, 1]
        i = np.floor(y / self.VRES).astype(np.int64) + self.VN // 2
        j = np.floor(x / self.VRES).astype(np.int64) + self.VN // 2
        ok = (i >= 0) & (i < self.VN) & (j >= 0) & (j < self.VN)
        if r_max is not None:
            pose = self.pose()
            if pose is None:
                return None
            ok &= np.hypot(x - pose[0], y - pose[1]) <= r_max
        return i[ok], j[ok]

    def cb_suelo(self, msg):
        r = self._a_mapa(msg, r_max=self.radio_visto)
        if r is not None:
            i, j = r
            with self.lock_rej:
                v = self.visto[i, j]
                self.visto[i, j] = np.where(v < 250, v + 1, v)

    def cb_obst(self, msg):
        r = self._a_mapa(msg)
        with self.lock_rej:
            if r is not None:
                i, j = r
                np.add.at(self.ocup, (i, j), 1.0)
                self.visto[i, j] = np.maximum(self.visto[i, j], 1)
            if time.monotonic() - self.t_decae > 2.0 and self.ocup is not None:   # la gente se va
                self.t_decae = time.monotonic()
                self.ocup *= 0.7

    def _celda(self, x, y):
        return int(math.floor(y / self.VRES)) + self.VN // 2, int(math.floor(x / self.VRES)) + self.VN // 2

    def _mundo(self, i, j):
        return (j - self.VN // 2 + 0.5) * self.VRES, (i - self.VN // 2 + 0.5) * self.VRES

    def _ocupado_slam(self, sl):
        out = np.zeros((sl[0].stop - sl[0].start, sl[1].stop - sl[1].start), bool)
        m = self.mapa
        if m is None:
            return out
        g = np.array(m.data, np.int8).reshape(m.info.height, m.info.width)
        occ = g >= 50
        # sin puntitos sueltos (ruido del lidar en el mapa): una pared tiene vecinos ocupados
        from scipy import ndimage
        vecinos = ndimage.convolve(occ.astype(np.uint8), np.ones((3, 3), np.uint8), mode="constant") - occ
        ii, jj = np.nonzero(occ & (vecinos >= 1))
        if len(ii) == 0:
            return out
        x = m.info.origin.position.x + (jj + 0.5) * m.info.resolution
        y = m.info.origin.position.y + (ii + 0.5) * m.info.resolution
        a = np.floor(y / self.VRES).astype(np.int64) + self.VN // 2 - sl[0].start
        b = np.floor(x / self.VRES).astype(np.int64) + self.VN // 2 - sl[1].start
        ok = (a >= 0) & (a < out.shape[0]) & (b >= 0) & (b < out.shape[1])
        out[a[ok], b[ok]] = True
        return out

    def _zona_vista(self, x, y, r=0.3, max_sin_ver=None):
        i, j = self._celda(x, y)
        k = int(r / self.VRES)
        with self.lock_rej:
            sub = self.visto[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1].copy()
            oc = self.ocup[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1] >= 4.0
        sin_ver = np.count_nonzero((sub == 0) & ~oc)
        if max_sin_ver is not None:
            return sub.size > 0 and sin_ver <= max_sin_ver
        return sub.size > 0 and sin_ver < 0.10 * sub.size

    def area_vista(self):
        if self.visto is None:
            return 0.0
        with self.lock_rej:
            n = np.count_nonzero(self.visto >= 2)
        return round(float(n) * self.VRES ** 2, 1)

    def _zonas_escalera(self, sl):
        """Celdas de las escaleras (y su entorno): no son fronteras a explorar a pie."""
        out = np.zeros((sl[0].stop - sl[0].start, sl[1].stop - sl[1].start), bool)
        for p in self.portales:
            if p["piso"] != self.piso:
                continue
            i, j = self._celda(p["x"], p["y"])
            k = int(0.9 / self.VRES)
            a0, b0 = i - k - sl[0].start, j - k - sl[1].start
            out[max(0, a0):max(0, a0 + 2 * k + 1), max(0, b0):max(0, b0 + 2 * k + 1)] = True
        return out

    def publicar_explorado(self):
        try:
            if self.visto is None:
                return
            with self.lock_rej:
                ii, jj = np.nonzero(self.visto)
                if len(ii) == 0:
                    return
                m = 10
                a0, a1 = max(0, ii.min() - m), min(self.VN, ii.max() + m + 1)
                b0, b1 = max(0, jj.min() - m), min(self.VN, jj.max() + m + 1)
                v = self.visto[a0:a1, b0:b1].copy()
                oc = self.ocup[a0:a1, b0:b1] >= 4.0
            g = np.full(v.shape, -1, np.int8)
            g[v >= 2] = 0
            g[oc] = 100
            msg = OccupancyGrid()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info.resolution = float(self.VRES)
            msg.info.width, msg.info.height = int(g.shape[1]), int(g.shape[0])
            x0, y0 = self._mundo(a0, b0)
            msg.info.origin.position.x = float(x0 - self.VRES / 2)
            msg.info.origin.position.y = float(y0 - self.VRES / 2)
            msg.info.origin.orientation.w = 1.0
            msg.data = g.ravel().tolist()
            self.pub_mapa_v.publish(msg)
            ma = MarkerArray()
            mk = Marker()
            mk.header = msg.header
            mk.ns, mk.id, mk.type, mk.action = "fronteras", 0, Marker.CUBE_LIST, Marker.ADD
            mk.scale.x = mk.scale.y = mk.scale.z = float(self.VRES)
            mk.color.g, mk.color.a = 1.0, 0.9
            mk.pose.orientation.w = 1.0
            mk.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in self.ult_fronteras[:5000]]
            ma.markers.append(mk)
            mt = Marker()
            mt.header = msg.header
            mt.ns, mt.id, mt.type = "meta", 1, Marker.SPHERE
            mt.pose.orientation.w = 1.0
            if self.meta is not None and self.meta.get("tipo") != "mirar":
                mt.action = Marker.ADD
                mt.pose.position.x, mt.pose.position.y, mt.pose.position.z = float(self.meta["x"]), float(self.meta["y"]), 0.2
                mt.scale.x = mt.scale.y = mt.scale.z = 0.3
                mt.color.r, mt.color.g, mt.color.a = 1.0, 0.9, 1.0
            else:
                mt.action = Marker.DELETE
            ma.markers.append(mt)
            self.pub_fr.publish(ma)
        except Exception as e:      # la vista nunca debe tumbar el gestor
            self.get_logger().warn("publicar_explorado: %r" % e, throttle_duration_sec=10.0)

    def _pisado(self, pose, r=0.55):
        """Donde el perro ESTA parado es suelo, aunque el lidar no lo vea (zona ciega bajo el
        cuerpo): sin esto el perro quedaba en una 'isla sin ver', no encontraba camino a ninguna
        zona y daba el piso por terminado sin salir del cuarto."""
        i, j = self._celda(pose[0], pose[1])
        k = int(math.ceil(r / self.VRES))
        a0, b0 = max(0, i - k), max(0, j - k)
        sub = self.visto[a0:i + k + 1, b0:j + k + 1]
        yy, xx = np.mgrid[a0:a0 + sub.shape[0], b0:b0 + sub.shape[1]]
        disco = (yy - i) ** 2 + (xx - j) ** 2 <= k * k
        sub[disco] = np.maximum(sub[disco], 2)
        self.ocup[a0:i + k + 1, b0:j + k + 1][disco] = 0.0

    def fronteras(self, pose):
        from scipy import ndimage
        with self.lock_rej:
            self._pisado(pose)
            ii, jj = np.nonzero(self.visto)
            if len(ii) == 0:
                return []
            m = 8
            sl = (slice(max(0, ii.min() - m), min(self.VN, ii.max() + m + 1)),
                  slice(max(0, jj.min() - m), min(self.VN, jj.max() + m + 1)))
            visto = self.visto[sl].copy()
            ocup_v = self.ocup[sl] >= 4.0
        ocup = ocup_v | self._ocupado_slam(sl)
        libre = (visto >= 2) & ~ocup
        desc = (visto == 0) & ~ocup
        lab_d, _n = ndimage.label(desc)
        tam = np.bincount(lab_d.ravel())
        tam[0] = 0
        desc = (tam >= self.hueco_min / self.VRES ** 2)[lab_d] & desc
        dist = ndimage.distance_transform_edt(~ocup) * self.VRES
        pasable = libre & (dist >= 0.15)
        borde = np.zeros_like(desc)
        borde[1:] |= desc[:-1]
        borde[:-1] |= desc[1:]
        borde[:, 1:] |= desc[:, :-1]
        borde[:, :-1] |= desc[:, 1:]
        # lo que hay mas alla de una escalera se descubre CRUZANDOLA, no como frontera
        # (la holgura NO se exige a toda la frontera: en una puerta de 70 cm solo el centro tiene
        # 22 cm a cada lado y la frontera quedaba "chica" y descartada -> nunca salia del cuarto)
        fr = pasable & borde & ~self._zonas_escalera(sl)
        si, sj = self._celda(pose[0], pose[1])
        dcam = self._bfs(pasable, si - sl[0].start, sj - sl[1].start, self.VRES)
        lab, n = ndimage.label(fr, structure=np.ones((3, 3)))
        out = []
        for k, ob in enumerate(ndimage.find_objects(lab), start=1):
            a, b_ = np.nonzero(lab[ob] == k)
            if len(a) < self.frontera_min:
                continue
            a = a + ob[0].start
            b_ = b_ + ob[1].start
            d = dcam[a, b_]
            holg = dist[a, b_]
            # la meta va donde el perro CABE con holgura (en el centro de una puerta de 60-80 cm
            # hay 30-40 cm): rincones mas angostos no, ahi no puede ni girar sin tocar
            h_min = self.holgura_meta
            if float(holg.max()) < h_min:
                continue
            ok = np.isfinite(d) & (holg >= h_min - 1e-6)
            if not ok.any():
                continue
            lej = np.nonzero(ok & (d >= self.d_min_meta))[0]
            if len(lej):
                b = int(lej[np.argmin(d[lej])])
            else:
                cand = np.nonzero(ok)[0]
                b = int(cand[np.argmin(d[cand])])
            x, y = self._mundo(a[b] + sl[0].start, b_[b] + sl[1].start)
            if any(math.hypot(x - vx, y - vy) < 0.6 for vx, vy in self.vetadas):
                continue
            # preferir lo que tiene DELANTE: cada radian de giro "cuesta" como 0.4 m de camino
            # (asi sigue de frente hacia los cuartos en vez de irse de lado y volver)
            giro = abs(ang(math.atan2(y - pose[1], x - pose[0]) - pose[2]))
            out.append({"x": x, "y": y, "n": int(len(a)), "d": float(d[b]),
                        "costo": float(d[b]) - 0.15 * math.sqrt(len(a)) + 0.4 * giro})
        self.ult_fronteras = [self._mundo(a_ + sl[0].start, b_ + sl[1].start) for a_, b_ in zip(*np.nonzero(fr))]
        if not out:
            # ninguna zona con camino CONOCIDO: igual voy a la mas cercana en linea recta y que
            # Nav2 busque el camino (atraviesa lo no visto). Mejor intentar que quedarse girando.
            for k, ob in enumerate(ndimage.find_objects(lab), start=1):
                a, b_ = np.nonzero(lab[ob] == k)
                if len(a) < self.frontera_min:
                    continue
                a = a + ob[0].start
                b_ = b_ + ob[1].start
                holg = dist[a, b_]
                ok = holg >= 0.15
                if not ok.any():
                    continue
                xs, ys = self._mundo(a + sl[0].start, b_ + sl[1].start)
                d = np.hypot(xs - pose[0], ys - pose[1])
                d[~ok] = np.inf
                cand = np.nonzero(ok & (d >= self.d_min_meta))[0]
                if not len(cand):
                    continue
                b = int(cand[np.argmin(d[cand])])
                x, y = float(xs[b]), float(ys[b])
                if any(math.hypot(x - vx, y - vy) < 0.6 for vx, vy in self.vetadas):
                    continue
                giro = abs(ang(math.atan2(y - pose[1], x - pose[0]) - pose[2]))
                out.append({"x": x, "y": y, "n": int(len(a)), "d": float(d[b]), "directa": True,
                            "costo": float(d[b]) * 1.3 - 0.15 * math.sqrt(len(a)) + 0.4 * giro})
        out.sort(key=lambda o: o["costo"])
        return out

    @staticmethod
    def _bfs(ok, si, sj, r):
        H, W = ok.shape
        dist = np.full((H, W), np.inf)
        if not (0 <= si < H and 0 <= sj < W):
            return dist
        if not ok[si, sj]:
            R = 10
            ii, jj = np.nonzero(ok[max(0, si - R):si + R + 1, max(0, sj - R):sj + R + 1])
            if len(ii) == 0:
                return dist
            k = int(np.argmin((ii - min(si, R)) ** 2 + (jj - min(sj, R)) ** 2))
            si, sj = ii[k] + max(0, si - R), jj[k] + max(0, sj - R)
        dist[si, sj] = 0.0
        q = collections.deque([(si, sj)])
        vec = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
        while q:
            i, j = q.popleft()
            d0 = dist[i, j]
            for di, dj, c in vec:
                a, b = i + di, j + dj
                if 0 <= a < H and 0 <= b < W and ok[a, b] and dist[a, b] > d0 + c * r + 1e-9:
                    dist[a, b] = d0 + c * r
                    q.append((a, b))
        return dist

    # ------------------------------------------------------------------ ordenes
    def srv_comando(self, req, res):
        try:
            with self.lock:
                ok, msg, extra = self.ordenar(req.orden.strip().lower(), list(req.args))
        except Exception as e:      # nunca tumbar el gestor por una orden
            ok, msg, extra = False, "error: %r" % e, {}
        res.ok, res.mensaje, res.json = ok, msg, json.dumps(extra)
        return res

    def _escalera(self, orden):
        req = Comando.Request()
        req.orden = orden
        if self.cli_esc.service_is_ready():
            self.cli_esc.call_async(req)

    def ordenar(self, o, a):
        pose = self.pose()
        if o == "explorar":
            if pose is not None:
                self.puntos["inicio"] = {"x": pose[0], "y": pose[1], "yaw": pose[2], "piso": self.piso or 0}
                self._guardar_puntos()
            self.cancelar()
            self.vetadas = []
            self.sin_fronteras = 0
            self.miradas = 0
            self.final = None
            self.escapes = []
            self.ref_mov = None
            self.reintento_vetos = False
            self.modo, self.motivo = "explorando", "empezando a explorar"
            self.log("EXPLORAR (escaleras: %s)" % self.politica)
            if self.cli_levantar.service_is_ready():
                self.cli_levantar.call_async(Trigger.Request())
            L = self.info.get("limpiador") or {}
            if self.auto_calibrar and self.cli_calibrar.service_is_ready() and "sin" not in a:
                # SIEMPRE antes de explorar: 2 vueltas en el sitio y aprende los fantasmas de hoy
                # (si algo real esta muy cerca, el limpiador corta el giro). Ademas los sigue
                # aprendiendo solo mientras camina.  'go2 explorar sin' = sin la vuelta inicial
                self.cli_calibrar.call_async(Trigger.Request())
                self.calibrando = time.monotonic()
                self.log("primero calibro los fantasmas (2 vueltas en el sitio) y despues exploro")
                return True, "primero calibro los fantasmas (2 vueltas, ~15 s) y despues exploro", {}
            return True, "explorando", {}
        if o in ("parar", "stop"):
            self.cancelar()
            if self.cruce is not None:
                self._escalera("parar")
                if not (self.info.get("escalera") or {}).get("en_escalera"):
                    self.cruce = None
                    self._pausar_slam(False)
            self.final = None
            self.modo, self.motivo = "quieto", "parado por ti"
            self.calibrando = None
            if self.cli_calibrar_parar.service_is_ready():
                self.cli_calibrar_parar.call_async(Trigger.Request())
            if self.cli_parar.service_is_ready():
                self.cli_parar.call_async(Trigger.Request())
            return True, "parado", {}
        if o == "pausa":
            self.pausar("pausado por ti")
            return True, "pausado", {}
        if o == "seguir":
            if (self.info.get("escalera") or {}).get("detenido"):
                self._escalera("seguir")
                return True, "sigo con la escalera", {}
            if self.modo != "pausado":
                return False, "no estaba pausado", {}
            self.modo = getattr(self, "modo_antes", "quieto")
            self.motivo = "sigo"
            return True, "sigo: " + self.modo, {}
        if o == "guardar":
            if not a:
                return False, "uso: go2 guardar NOMBRE", {}
            if pose is None:
                return False, "todavia no se donde estoy (falta el mapa / TF map->base_link)", {}
            self.puntos[a[0]] = {"x": pose[0], "y": pose[1], "yaw": pose[2], "piso": self.piso or 0}
            self._guardar_puntos()
            return True, "punto '%s' guardado (piso %d, x=%.2f y=%.2f)" % (a[0], self.piso or 0, pose[0], pose[1]), {}
        if o in ("ir", "casa"):
            nombre = "inicio" if o == "casa" else (a[0] if a else None)
            if nombre not in self.puntos:
                return False, "no conozco el punto '%s' (go2 puntos)" % nombre, {}
            self.cancelar()
            self.modo, self.motivo = ("volviendo" if o == "casa" else "yendo"), "voy a '%s'" % nombre
            ok = self.ir_punto(nombre)
            return ok, ("voy a '%s'" % nombre) if ok else self.motivo, {}
        if o == "puntos":
            return True, ", ".join("%s (piso %d)" % (k, v.get("piso", 0)) for k, v in sorted(self.puntos.items())) \
                or "(ninguno)", {"puntos": self.puntos}
        if o == "borrar":
            if a and a[0] in self.puntos:
                del self.puntos[a[0]]
                self._guardar_puntos()
                return True, "borrado '%s'" % a[0], {}
            return False, "no existe", {}
        if o == "escaleras":
            if len(a) >= 2 and a[0] == "vision" and a[1] in ("on", "off"):
                self.esc_vision = "requerida" if a[1] == "on" else "off"
                return True, "escaleras con confirmacion de la camara: %s" % a[1], {}
            if a and a[0] in ("al_final", "inmediato", "off"):
                self.politica = a[0]
                return True, "escaleras: %s" % self.politica, {}
            lista = ["portal %d: %s en piso %d%s%s" % (p["id"], p["tipo"], p["piso"],
                     " -> piso %s" % p["destino"] if p["destino"] is not None else " (sin usar)",
                     " [fallos %d]" % p["fallos"] if p["fallos"] else "") for p in self.portales]
            return True, ("politica: %s | camara: %s | vision %s\n" % (
                self.politica, self.esc_vision, "OK" if self.vision_viva() else "NO llega")) + (
                "\n".join(lista) or "(ninguna confirmada)") + (
                "\ndescartadas por la camara: %d" % len(self.descartadas) if self.descartadas else ""), {}
        if o in ("subir", "bajar"):
            # tomar a mano la escalera que tiene delante
            T = self.T_mapa_odom()
            req = Comando.Request()
            req.orden, req.args = "tomar", [o, str(self.odom[3] if self.odom else 0.0)]
            self.cancelar()
            self.modo = "quieto"
            # la escalera que ya vio (si hay una cerca), para no duplicarla
            p = None
            for q in self.portales:
                if (q["piso"] == self.piso and q["tipo"] == o and pose is not None
                        and math.hypot(q["x"] - pose[0], q["y"] - pose[1]) < 2.5):
                    p = q
            if p is None:
                p = self.agregar_portal(o, pose[0] if pose else 0.0, pose[1] if pose else 0.0,
                                        pose[2] if pose else 0.0, 0.8)
            self._guardar_piso()
            self._pausar_slam(True)
            if self.cli_esc.service_is_ready():
                self.cli_esc.call_async(req)
            self.cruce = {"portal": p, "fase": "cruzando", "t": time.monotonic(), "piso0": self.piso}
            return True, "tomo la escalera de %s que tengo delante" % o, {}
        if o == "calibrar":
            if not self.cli_calibrar.service_is_ready():
                return False, "el limpiador no responde", {}
            self.cancelar()
            self.modo, self.motivo = "quieto", "calibrando"
            self.cli_calibrar.call_async(Trigger.Request())
            return True, "calibrando: gira ~2 vueltas en el sitio. Mira 'go2 ver'", {}
        if o == "levantar":
            self.cli_levantar.call_async(Trigger.Request())
            return True, "levantando", {}
        if o == "estado":
            return True, "", self.estado()
        if o == "volcar":
            ruta = os.path.join(DATOS, "vista.npz")
            np.savez_compressed(ruta, visto=self.visto, ocup=self.ocup, vetadas=np.array(self.vetadas),
                                pose=np.array(pose if pose else (0, 0, 0)), res=self.VRES, n=self.VN)
            return True, ruta, {}
        if o == "log":
            return True, "\n".join(self.logs), {}
        return False, "orden desconocida: %s (go2 ayuda)" % o, {}

    def estado(self):
        pose = self.pose()
        E = self.info.get("escalera") or {}
        return {"modo": self.modo, "motivo": self.motivo,
                "pose": [round(v, 2) for v in pose] if pose else None,
                "piso": self.piso, "pisos": len(self.pisos),
                "escaleras": len([p for p in self.portales if not p["retorno"]]),
                "escalera": (E.get("estado") + (" " + E.get("info", "") if E.get("info") else ""))
                if E.get("activa") or E.get("detenido") else None,
                "meta": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in self.meta.items() if k != "t0"}
                if self.meta else None,
                "restante": self.restante, "mapa_m2": self.area_vista(), "fronteras": self.n_fronteras,
                "bateria": self.bateria, "remoto": self.remoto,
                "limpiador": self.info["limpiador"], "driver": self.info["driver"],
                "vision": {k: self.vision.get(k) for k in ("fps", "fuente", "gpu", "escalera")}
                if self.vision_viva() else None,
                "vision_objetos": [o.get("nombre") for o in self.vision.get("objetos", [])][:6]
                if self.vision_viva() else [],
                "ultimo_log": self.logs[-1] if self.logs else ""}

    def publicar_estado(self):
        if self.pose() is not None and "inicio" not in self.puntos and self.mapa is not None:
            p = self.pose()
            self.puntos["inicio"] = {"x": p[0], "y": p[1], "yaw": p[2], "piso": self.piso or 0}
            self._guardar_puntos()
        self.pub_pausa_slam.publish(Bool(data=self.pausa_slam))
        self.pub_estado.publish(String(data=json.dumps(self.estado())))


def main():
    rclpy.init()
    n = Gestor()
    ex = MultiThreadedExecutor(num_threads=5)
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
