"""go2_guardian: el unico nodo que le habla al perro (ver guardian_logica.py).

  /cmd_vel (geometry_msgs/TwistStamped, pathFollower de CMU)  -> /api/sport/request (Move/StopMove)
  /joy (PS4, solo el control fisico) -> SHARE+CRUZ acostar | SHARE+TRIANGULO levantar |
                                    SHARE+CUADRADO bloquear | SHARE+CIRCULO recuperar | CIRCULO pausa
                                    (L1 y R1 los usa CMU: borrar grafo de rutas / mapa de terreno)
  /terrain_map (solo la hora)    -> sin mapa de terreno > 1.5 s: quieto (la evasion no ve)
  servicios std_srvs/Trigger     -> /go2/acostar /go2/levantar /go2/bloquear /go2/desbloquear
                                    /go2/pausa /go2/recuperar
  /go2/guardian_estado (JSON 2 Hz)
Ademas apaga la evasion de obstaculos de Unitree (frena al perro y lo deja pisando en el sitio: la
evasion la hace el planificador de CMU) y manda StopMove al arrancar y al cerrarse.
"""
import itertools
import json
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy, PointCloud2
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from unitree_api.msg import Request, Response

from .guardian_logica import API_STOPMOVE, Guardian

EVITA_SET, EVITA_GET = 1001, 1002          # servicio "obstacles_avoid" de Unitree
# /joy que NO son el control: los botones de RViz de CMU y 'go2v2 ir/parar' (no son combinaciones)
JOY_AJENOS = ("goalpoint_tool", "waypoint_tool", "teleop_panel", "go2v2")


class NodoGuardian(Node):
    def __init__(self):
        super().__init__("go2_guardian")
        P = self.declare_parameter
        self.g = Guardian(vx_max=float(P("vx_max", 1.0).value), vy_max=float(P("vy_max", 0.6).value),
                          wz_max=float(P("wz_max", 1.5).value), vigencia=float(P("vigencia", 0.25).value),
                          freewalk=bool(P("freewalk", True).value),
                          v_min_caminar=float(P("v_min_caminar", 0.0).value),
                          odom_max=float(P("odom_max_edad", 0.6).value),
                          terreno_max=float(P("terreno_max_edad", 1.5).value))
        self.apagar_evita = bool(P("apagar_evitacion_unitree", True).value)
        self.lock = threading.Lock()
        self.ids = itertools.count(int(time.time() * 1000) % 1_000_000_000)   # seguro entre hilos
        self.pub_req = self.create_publisher(Request, "/api/sport/request", 10)
        self.pub_evita = self.create_publisher(Request, "/api/obstacles_avoid/request", 10)
        self.pub_stop = self.create_publisher(Int8, "/stop", 5)
        self.pub_est = self.create_publisher(String, "/go2/guardian_estado", 5)
        grupo = MutuallyExclusiveCallbackGroup()
        self.create_subscription(TwistStamped, "/cmd_vel", self.cb_cmd, 10, callback_group=grupo)
        self.create_subscription(Joy, "/joy", self.cb_joy, 10, callback_group=grupo)
        self.create_subscription(Response, "/api/obstacles_avoid/response", self.cb_evita, 10, callback_group=grupo)
        # el mapa de terreno: solo importa que llegue (raw: sin deserializar la nube en Python)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terreno, 2, callback_group=grupo, raw=True)
        for nombre, fn in (("acostar", self.g.acostar), ("levantar", self.g.levantar),
                           ("bloquear", self.g.bloquear), ("desbloquear", self.g.desbloquear),
                           ("pausa", self.g.alternar_pausa), ("recuperar", self.g.recuperar)):
            self.create_service(Trigger, "/go2/" + nombre, self._srv(nombre, fn), callback_group=grupo)
        self.create_timer(self.g.periodo, self.lazo, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self.estado, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(3.0, self.revisar_evitacion, callback_group=MutuallyExclusiveCallbackGroup())
        self.evita = None
        self.n_cmd = 0
        self.pausa_prev = False
        self.mandar([(API_STOPMOVE, None)])
        self.get_logger().info("go2_guardian listo: /cmd_vel -> perro (Move a %.0f Hz, StopMove si la orden "
                               "se vence %.2f s). PS4: SHARE+CRUZ acostar, SHARE+TRIANGULO levantar, "
                               "SHARE+CUADRADO bloquear, SHARE+CIRCULO recuperar, CIRCULO pausa"
                               % (1.0 / self.g.periodo, self.g.vigencia))

    def ahora(self):
        return time.monotonic()

    # ------------------------------------------------------------------ entradas
    def cb_cmd(self, m):
        # el pathFollower de CMU sella /cmd_vel con la hora de la odometria que uso: si esa hora es
        # vieja, Point-LIO dejo de publicar y la orden se calculo a ciegas
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        edad = self.get_clock().now().nanoseconds * 1e-9 - st if st > 0 else None
        with self.lock:
            self.g.orden(self.ahora(), m.twist.linear.x, m.twist.linear.y, m.twist.angular.z, edad)
            self.n_cmd += 1

    def cb_terreno(self, _m):
        with self.lock:
            self.g.terreno(self.ahora())

    def cb_joy(self, m):
        if m.header.frame_id in JOY_AJENOS:
            return
        with self.lock:
            out = self.g.joystick(self.ahora(), list(m.buttons), list(m.axes))
        self.mandar(out)

    def _srv(self, nombre, fn):
        def cb(_req, res):
            with self.lock:
                out = fn(self.ahora())
            self.mandar(out)
            res.success = True
            res.message = nombre
            return res
        return cb

    def cb_evita(self, m):
        if m.header.identity.api_id != EVITA_GET or m.header.status.code != 0:
            return
        try:
            self.evita = bool(json.loads(m.data).get("enable"))
        except (ValueError, AttributeError):
            pass

    # ------------------------------------------------------------------ salida
    def pedido(self, api, parametros=None, pub=None, sin_respuesta=True):
        r = Request()
        r.header.identity.id = next(self.ids)
        r.header.identity.api_id = api
        r.header.policy.noreply = sin_respuesta
        r.parameter = json.dumps(parametros) if parametros is not None else ""
        (pub or self.pub_req).publish(r)

    def mandar(self, lista):
        for api, par in lista:
            self.pedido(api, par)

    def lazo(self):
        with self.lock:
            out = self.g.paso(self.ahora())
            avisos, self.g.avisos = self.g.avisos, []
            g = self.g
            pausa = g.pausa or g.bloqueado or g.acostado or self.ahora() < g.quieto_hasta
        self.mandar(out)
        for a in avisos:
            self.get_logger().warn(a)
        if pausa != self.pausa_prev:
            # que el seguidor de ruta de CMU tambien frene (y no arranque de golpe al soltar)
            self.pub_stop.publish(Int8(data=15 if pausa else 0))
            self.pausa_prev = pausa

    def revisar_evitacion(self):
        if not self.apagar_evita:
            return
        if self.evita:
            self.get_logger().warn("evasion de obstaculos de Unitree PRENDIDA: la apago (la hace CMU)")
            self.pedido(EVITA_SET, {"enable": False}, pub=self.pub_evita)
        self.pedido(EVITA_GET, None, pub=self.pub_evita, sin_respuesta=False)

    def estado(self):
        with self.lock:
            g = self.g
            d = {"acostado": g.acostado, "bloqueado": g.bloqueado, "pausa": g.pausa,
                 "moviendo": g.moviendo, "orden": [round(v, 3) for v in g.cmd],
                 "orden_hace_s": round(self.ahora() - g.t_cmd, 2) if g.t_cmd > 0 else None,
                 "cmd_hz": self.n_cmd * 2, "move_hz": g.n_move * 2, "stops": g.n_stop,
                 "vencidas": g.n_vencidas, "evasion_unitree": self.evita, "manual": g.manual,
                 "parado_por": g.motivo_parada,
                 "edad_odom_s": None if g.edad_odom is None else round(g.edad_odom, 3),
                 "terreno_hace_s": None if g.t_terreno is None else round(self.ahora() - g.t_terreno, 2)}
            self.n_cmd = 0
            g.n_move = 0
        self.pub_est.publish(String(data=json.dumps(d)))

    def cerrar(self):
        for _ in range(3):
            self.pedido(API_STOPMOVE)
            time.sleep(0.05)


def main():
    # sin el manejador de Ctrl+C de rclpy: el de rclpy apaga ROS ANTES de que se pueda mandar el
    # ultimo StopMove (y el perro podria seguir con la ultima orden). Aca: Ctrl+C -> StopMove -> fin
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    n = NodoGuardian()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            n.get_logger().error("guardian: %r" % (e,))
    finally:
        try:
            n.cerrar()
        except Exception:
            pass
        try:
            ex.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
