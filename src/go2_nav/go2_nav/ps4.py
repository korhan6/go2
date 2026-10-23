"""go2_ps4: control PS4 (DualShock 4, USB o Bluetooth) para el perro. Ver nucleo/mando.py.

  /joy (nodo joy) -> /cmd_vel_prioridad (el driver le da prioridad sobre Nav2) + ordenes al gestor.
  Con L1 suelto no manda nada: Nav2 / la exploracion siguen solos. Frena igual que 'go2 manejar'
  si hay algo a menos de 20 cm del cuerpo (lo que mide el limpiador).
  Indices de botones: ~/.go2_nav/ps4.yaml (lo escribe 'go2 control'); si no existe, se eligen por
  el numero de botones del control (kernel 13 / SDL 15-16).
"""
import json
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_nav_interfaces.srv import Comando

from .nucleo.mando import Mando, elegir_layout

PROPIO = os.path.expanduser("~/.go2_nav/ps4.yaml")


def leer_propio():
    try:
        with open(PROPIO) as f:
            return yaml.safe_load(f) or None
    except (OSError, yaml.YAMLError):
        return None


class Ps4(Node):
    def __init__(self):
        super().__init__("go2_ps4")
        self.propio = leer_propio()
        self.mando = None
        self.holguras, self.t_holg = {}, 0.0
        self.t_joy = 0.0
        self.orden, self.t_orden = None, 0.0
        self.pub = self.create_publisher(Twist, "cmd_vel_prioridad", 10)
        self.create_subscription(Joy, "/joy", self.cb_joy, 10)
        self.create_subscription(String, "/go2/limpiador_estado", self.cb_holg, 1)
        self.cli = self.create_client(Comando, "/go2/comando")
        self.cli_levantar = self.create_client(Trigger, "/go2/levantar")
        self.create_timer(0.05, self.lazo)               # 20 Hz: el driver da 0.4 s de vigencia
        self.create_timer(5.0, self.revisar)
        self.get_logger().info("control PS4 listo: L1 + sticks = manejar | CIRCULO = parar | L1+PS = bajar escalera | "
                               "L1+CRUZ = acostar | L1+TRIANGULO = levantar | L1+SHARE = bloquear/desbloquear")

    def cb_holg(self, m):
        try:
            self.holguras, self.t_holg = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def cb_joy(self, m):
        self.t_joy = time.monotonic()
        if self.mando is None:
            nombre, lay = elegir_layout(len(m.buttons), self.propio)
            signos = (self.propio or {}).get("signos") if nombre == "propio" else None
            self.mando = Mando(lay, signos)
            if nombre is None:
                self.get_logger().warn("control con %d botones: no lo conozco, uso el orden SDL. "
                                       "Corre 'go2 control' para medirlo" % len(m.buttons))
            else:
                self.get_logger().info("control: %d botones, %d ejes -> orden '%s' (L1 = boton %d)"
                                       % (len(m.buttons), len(m.axes), nombre, lay["botones"]["l1"]))
        H = self.holguras if time.monotonic() - self.t_holg < 1.0 else None
        orden, acciones = self.mando.paso(list(m.axes), list(m.buttons), H)
        if orden is not None:
            self.orden, self.t_orden = orden, time.monotonic()
            if self.mando.aviso:
                self.get_logger().warn(self.mando.aviso, throttle_duration_sec=2.0)
        elif self.orden != (0.0, 0.0, 0.0):   # un cero pendiente (L1 recien soltado) sale primero en lazo()
            self.orden = None
        for a in acciones:
            self.accion(a)

    def lazo(self):
        # el joy publica solo cuando algo cambia: la ultima orden se repite mientras L1 siga apretado
        if self.orden is None:
            return
        if time.monotonic() - self.t_joy > 1.0:          # el control se desconecto: un cero y soltar el mando
            self.orden = (0.0, 0.0, 0.0)
            if self.mando is not None:
                self.mando.manejando = False
            self.get_logger().warn("el control dejo de mandar: paro y devuelvo el mando a Nav2",
                                   throttle_duration_sec=5.0)
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = self.orden
        self.pub.publish(t)
        if self.orden == (0.0, 0.0, 0.0) and not (self.mando and self.mando.manejando):
            self.orden = None                             # el cero ya salio: Nav2 vuelve a mandar

    def comando(self, orden, *args):
        if not self.cli.service_is_ready():
            self.get_logger().warn("el gestor no responde a '%s'" % orden)
            return
        r = Comando.Request()
        r.orden, r.args = orden, list(args)
        fut = self.cli.call_async(r)
        fut.add_done_callback(lambda f: self.get_logger().info(
            "control -> %s: %s" % (orden, f.result().mensaje if f.result() else "sin respuesta")))

    def accion(self, a):
        if a == "parar":
            self.comando("parar")
        elif a == "pausa_seguir":
            self.comando("seguir" if self._pausado() else "pausa")
        elif a == "levantar":
            if self.cli_levantar.service_is_ready():
                self.cli_levantar.call_async(Trigger.Request())
        elif a in ("subir", "bajar", "acostar", "bloqueo"):
            self.comando(a)
        elif a == "guardar":
            self.guardar_siguiente()

    def guardar_siguiente(self):
        """Guarda 'control_N' con el primer N libre (pregunta los puntos que ya hay: no pisa ninguno)."""
        if not self.cli.service_is_ready():
            self.get_logger().warn("el gestor no responde: no guardo el punto")
            return
        r = Comando.Request()
        r.orden = "puntos"

        def listo(f):
            try:
                hay = json.loads(f.result().json).get("puntos", {}) if f.result() else {}
            except ValueError:
                hay = {}
            n = 1
            while "control_%d" % n in hay:
                n += 1
            self.comando("guardar", "control_%d" % n)
        self.cli.call_async(r).add_done_callback(listo)

    def _pausado(self):
        return getattr(self, "_estado_modo", "") == "pausado"

    def revisar(self):
        if self.t_joy == 0.0:
            self.get_logger().info("esperando el control PS4 (/joy). Prendelo o conectalo; "
                                   "'go2 control' lo diagnostica", throttle_duration_sec=30.0)


def main():
    rclpy.init()
    n = Ps4()
    n.create_subscription(String, "/go2/estado",
                          lambda m: setattr(n, "_estado_modo", (json.loads(m.data) or {}).get("modo", "")), 1)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
