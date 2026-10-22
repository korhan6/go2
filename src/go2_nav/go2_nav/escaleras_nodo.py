"""go2_escaleras: detecta escaleras y las cruza (nucleo: nucleo/terreno.py y nucleo/escaleras.py).

  SUBIR  (de frente): alinear -> centrar -> acercar -> marcha de escaleras -> subir -> salir
  BAJAR  (SIEMPRE en reversa): alinear -> centrar -> distancia -> GIRAR 180 -> verificar que
         detras hay escalera -> marcha -> bajar de reversa -> salir -> girar de frente

Antes de pisar la escalera, cualquier duda = cancela. Encima de los escalones nunca cambia de
marcha: si algo falla se queda quieto (DETENIDO) y espera 'go2 seguir' o el control remoto.
Un balcon o un desnivel sin escalones NUNCA se toma como escalera de bajada: hace falta VER el
primer escalon. El L1 (en la cabeza, a ~30 cm del suelo) lo tiene en su sombra hasta estar a
menos de ~0.55 m del borde, y pegado al borde deja de ver el suelo de delante: la bajada se
confirma con el hocico entre ~0.25 y ~0.55 m del borde. Por eso el perro se acerca despacio
hasta 25 cm (los pies quedan a 42 cm) y, si se paso, retrocede un poco (MEDIR). Lo que ve y
por que no confirma sale en 'go2 escaleras'.

Entradas: /utlidar/cloud_deskewed, /utlidar/robot_odom
Salidas:  /go2/escaleras (JSON: escaleras confirmadas y posibles bajadas, marco odom)
          /go2/escalera_estado (JSON: estado de la maniobra)
          cmd_vel_prioridad (solo durante la maniobra; el driver le da prioridad sobre Nav2)
          /go2/marcha (String: escaleras_on | escaleras_off | balancear) -> go2_driver
Servicio: /go2/escalera  (orden: tomar <subir|bajar> <yaw_odom> | parar | seguir)
"""
import collections
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from go2_nav_interfaces.srv import Comando
from unitree_go.msg import SportModeState

from . import ajustes
from .nube import leer_xyz, rpy_de_quat
from .nucleo import config, terreno
from .nucleo.control import Memoria, ang, guardian
from .nucleo.escaleras import EN_ESCALERA, Detector, Maniobra

# acercamiento antes de confirmar: distancia minima del HOCICO al borde (bajada) o al primer
# escalon (subida). De bajada hay que acercarse hasta que el lidar vea la huella (ver arriba).
D_ACERCAR = {"bajar": 0.25, "subir": 0.30}


class RobotMarcha(object):
    """Lo que la maniobra le pide al 'robot': cambiar de marcha y balancear (via go2_driver)."""

    def __init__(self, nodo):
        self.nodo = nodo

    def modo_escaleras(self, activo):
        self.nodo.pub_marcha.publish(String(data="escaleras_on" if activo else "escaleras_off"))
        return 0.8

    def balancear(self):
        self.nodo.pub_marcha.publish(String(data="balancear"))


class Escaleras(Node):
    def __init__(self):
        super().__init__("go2_escaleras")
        self.cfg = dict(config.DEFECTO)
        p = ajustes.declarador(self)          # 0 donde va 0.0 no tumba el nodo
        for k in ("v_subir", "v_bajar", "ancho_min_escalera", "v_escalera_max", "w_min", "radio_giro", "obst_min",
                  "obst_max", "sesgo_cerca"):
            self.cfg[k] = p(k, self.cfg[k]).value
        # 'ros2 param set /go2_escaleras radio_giro 0.32' cambia el valor EN VIVO (la maniobra usa este cfg)
        self.add_on_set_parameters_callback(self.cb_parametros)
        self.memoria = Memoria()
        self.detector = Detector()
        self.maniobra = Maniobra(RobotMarcha(self), self.cfg, self.log)
        self.pose = None
        self.imu = None             # (roll, pitch, t) del IMU del perro
        self.hist_alto = collections.deque(maxlen=8)     # celdas con algo arriba del piso (reflejos)
        self.suelo = None
        self.nubes = []
        self.holg = None
        self.espera = None          # acercamiento antes de empezar: dict
        self.t_alrededor = 0.0
        self.t_fino = 0.0
        self.resultado = None
        self.logs = []

        sd = qos_profile_sensor_data
        # la nube SIN fantasmas del limpiador (misma nube del L1, sin el cuerpo ni los fantasmas)
        self.create_subscription(PointCloud2, p("topico_nube", "/go2/nube_sin_fantasmas").value,
                                 self.cb_nube, sd)
        self.create_subscription(Odometry, "/odom", self.cb_odom, sd)   # la del driver (1 sola copia desde el perro)
        # inclinacion del cuerpo del IMU del perro: la bajada termina cuando el cuerpo queda plano
        self.create_subscription(SportModeState, "/lf/sportmodestate", self.cb_sport, sd)
        self.pub_esc = self.create_publisher(String, "/go2/escaleras", 1)
        self.pub_est = self.create_publisher(String, "/go2/escalera_estado", 1)
        self.pub_vel = self.create_publisher(Twist, "cmd_vel_prioridad", 10)
        self.pub_marcha = self.create_publisher(String, "/go2/marcha", 10)
        self.create_service(Comando, "/go2/escalera", self.srv)
        self.create_timer(0.1, self.lazo)
        self.create_timer(0.5, self.publicar)
        self.get_logger().info("go2_escaleras listo: subir de frente, bajar SIEMPRE en reversa")

    def cb_parametros(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name in self.cfg:
                v = ajustes.tipo_como(prm.value, config.DEFECTO[prm.name])
                if v is None:
                    return SetParametersResult(successful=False, reason="%s: valor %r no sirve" % (prm.name, prm.value))
        for prm in params:
            if prm.name in self.cfg:
                self.cfg[prm.name] = ajustes.tipo_como(prm.value, config.DEFECTO[prm.name])
                self.log("ajuste en vivo: %s = %s" % (prm.name, self.cfg[prm.name]))
        return SetParametersResult(successful=True)

    def log(self, txt):
        self.logs.append(time.strftime("%H:%M:%S ") + txt)
        del self.logs[:-30]
        self.get_logger().info(txt)

    # ------------------------------------------------------------------ entradas
    def cb_odom(self, m):
        p = m.pose.pose.position
        yaw, pitch, roll = rpy_de_quat(m.pose.pose.orientation)
        if self.imu is not None and time.monotonic() - self.imu[2] < 0.5:
            # la inclinacion mayor de las dos (odometria del lidar e IMU): "plano" = plano en las dos
            if abs(self.imu[1]) > abs(pitch):
                pitch = self.imu[1]
            if abs(self.imu[0]) > abs(roll):
                roll = self.imu[0]
        self.pose = (p.x, p.y, p.z, yaw, pitch, roll)

    def cb_sport(self, m):
        r = m.imu_state.rpy
        self.imu = (float(r[0]), float(r[1]), time.monotonic())

    def cb_nube(self, m):
        self.nubes.append(leer_xyz(m))
        del self.nubes[:-4]

    # ------------------------------------------------------------------ lazo 10 Hz
    def lazo(self):
        if self.pose is None:
            return
        ahora = time.monotonic()
        x, y, z, yaw, pitch, roll = self.pose
        nuevas, self.nubes = self.nubes, []
        limpios = []
        for xyz in nuevas:
            limpio = terreno.quitar_propio(xyz, x, y, z, yaw, self.suelo)
            limpio = terreno.corregir_cerca(limpio, x, y, yaw, self.cfg["sesgo_cerca"])
            # (reflejos del piso brillante: el limpiador ya los quita; esto cubre lo que se le escape)
            limpio = terreno.quitar_reflejos(limpio, self.suelo if self.suelo is not None else z - terreno.ALTURA_BASE,
                                             self.hist_alto)
            self.memoria.sumar(limpio, x, y, ahora)
            limpios.append(limpio)
        if not nuevas:
            self.memoria.sumar(np.zeros((0, 3), np.float32), x, y, ahora)
        local = self.memoria.puntos()
        en_escalera = self.maniobra.estado in EN_ESCALERA
        if en_escalera:
            self.suelo = z - terreno.ALTURA_BASE
        else:
            self.suelo = terreno.suelo_abs(local, x, y, z, yaw, previo=self.suelo)
        if self.suelo is None:
            return
        self.holg, _ = guardian(local, (x, y, z, yaw), self.suelo, self.cfg, rampa=abs(pitch) > math.radians(4))
        if nuevas and len(local):
            alrededor = not en_escalera and ahora - self.t_alrededor > 1.0
            if alrededor:
                self.t_alrededor = ahora
            # rumbo fino de la escalera: siempre en la maniobra; si no, 2 veces por segundo (go2 escaleras)
            fino = self.maniobra.activa or self.espera is not None or ahora - self.t_fino > 0.5
            if fino:
                self.t_fino = ahora
            self.detector.actualizar(local, (x, y, z, yaw), self.suelo, ahora, alrededor=alrededor, crudos=limpios,
                                     fino=fino)

        if self.espera is not None:
            self.acercarse(ahora)
            return
        if self.maniobra.activa:
            vx, vy, wz = self.maniobra.paso(ahora, self.pose, self.detector, self.holg)
            self.mover(vx, vy, wz)
            if self.maniobra.resultado is not None:
                r = dict(self.maniobra.resultado)
                r["z"] = z
                r["pose_odom"] = [x, y, yaw]
                self.resultado = r
                self.detector.limpiar()
                self.memoria.limpiar()
                self.suelo = None
                self.mover(0.0, 0.0, 0.0)
        elif self.maniobra.estado == "DETENIDO":
            self.mover(0.0, 0.0, 0.0)

    def mover(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub_vel.publish(t)

    # ------------------------------------------------------------------ acercamiento
    def acercarse(self, ahora):
        """Mirando la escalera: espera a confirmarla; si no, se acerca despacio (el primer
        escalon de una bajada solo se ve de cerca), nunca a menos de D_ACERCAR del borde."""
        e = self.espera
        x, y, z, yaw, pitch, roll = self.pose
        err = ang(e["yaw"] - yaw)
        # el giro que lleva sigue ~0.25 s despues de parar: se cuenta para no pasarse
        if abs(err - 0.25 * e.get("w", 0.0)) > math.radians(6) and ahora - e["t"] < 8.0:
            e["w"] = math.copysign(max(self.cfg["w_min"], min(0.8, 1.5 * abs(err))), err)
            self.mover(0.0, 0.0, e["w"])
            e["t_quieto"] = ahora
            return
        e["w"] = 0.0
        c = self.detector.confirmada(e["tipo"], ahora, (x, y))
        if c is not None and c.get("dist", 9.0) < 2.2:
            self.mover(0.0, 0.0, 0.0)
            self.espera = None
            self.maniobra.empezar(e["tipo"], c, ahora)
            return
        e.setdefault("p0", (x, y))
        e.setdefault("t_quieto", ahora)
        avance = math.hypot(x - e["p0"][0], y - e["p0"][1])
        h = self.holg
        d_min = D_ACERCAR[e["tipo"]]
        if h is not None and ahora - e["t_quieto"] < 12.0 and avance < 0.80:
            # caida = SUPERFICIE por debajo del suelo (un escalon); los reflejos del piso brillante
            # no cuentan. hueco = el suelo se acaba sin nada visible (balcon): tampoco se acerca
            d_ob, d_ca, d_hu = h["adelante"]
            libre = min(d_ca, d_hu, d_ob)
            # subida con el hocico PEGADO al escalon (< 10 cm): la cabeza tapa el primer escalon y no
            # se confirma. Retrocede despacio hasta ~15 cm (como mucho 20 cm: atras no se ve)
            g = e.get("atras")
            if (e["tipo"] == "subir" and ahora - e["t_quieto"] > 2.0 and
                    (g is not None or d_ob < 0.10) and (g is None or g["activo"])):
                g = e.setdefault("atras", {"p": (x, y), "activo": True, "emp": {"v": 0.10, "t": ahora, "p": (x, y)}})
                atras = math.hypot(x - g["p"][0], y - g["p"][1])
                if d_ob < 0.15 and atras < 0.20 and h["atras"][0] > 0.15:
                    e["freno"] = "pegado al escalon (%.0f cm): retrocedo un poco para verlo" % (100 * d_ob)
                    self.mover(-self._v_acercar(g, ahora, (x, y), 0.25), 0.0, max(-0.3, min(0.3, 1.2 * err)))
                    return
                g["activo"] = False
                self.log("retrocedi %.2f m para ver el primer escalon" % atras)
            if libre > d_min:
                # el Go2 no camina a 0.07 m/s (pisa en el sitio): arranca a 0.10 y, si no avanza,
                # sube de a poco hasta 0.25 m/s (mas despacio cerca del borde)
                v = self._v_acercar(e, ahora, (x, y), 0.25 if libre > d_min + 0.20 else 0.15)
                self.mover(v, 0.0, max(-0.3, min(0.3, 1.2 * err)))
                return
            e["freno"] = ("algo delante a %.0f cm" % (100 * d_ob) if d_ob <= d_min else
                          "borde/caida a %.0f cm" % (100 * d_ca) if d_ca <= d_min else
                          "el suelo se acaba a %.0f cm (sin nada visible)" % (100 * d_hu))
        self.mover(0.0, 0.0, 0.0)
        if ahora - e["t_quieto"] > 14.0:
            self.espera = None
            visto = self.detector.motivo_subida if e["tipo"] == "subir" else self.detector.motivo
            motivo = "no confirme la escalera de %s: %s%s" % (
                e["tipo"], visto or "no la veo",
                " | me frene por: %s" % e["freno"] if e.get("freno") else
                " | me acerque %.2f m sin verla" % avance)
            self.resultado = {"ok": False, "motivo": motivo, "tipo": e["tipo"], "sin_empezar": True,
                              "z": z, "pose_odom": [x, y, yaw]}
            self.log(motivo)

    @staticmethod
    def _v_acercar(e, ahora, p, v_max):
        """Velocidad para acercarse: 0.10 m/s y +0.03 cada 0.5 s si avanza menos del 40 %."""
        g = e.setdefault("emp", {"v": 0.10, "t": ahora, "p": p})
        dt = ahora - g["t"]
        if dt >= 0.5:
            real = math.hypot(p[0] - g["p"][0], p[1] - g["p"][1]) / dt
            if real < 0.4 * g["v"]:
                g["v"] = min(0.25, g["v"] + 0.03)
            g["t"], g["p"] = ahora, p
        return min(g["v"], v_max)

    # ------------------------------------------------------------------ ordenes
    def srv(self, req, res):
        o, a = req.orden, list(req.args)
        ahora = time.monotonic()
        res.ok, res.mensaje, res.json = True, "", "{}"
        if o == "tomar":
            if self.maniobra.activa or self.espera is not None:
                res.ok, res.mensaje = False, "ya estoy en una escalera"
                return res
            tipo = a[0] if a else "subir"
            yaw_o = float(a[1]) if len(a) > 1 else (self.pose[3] if self.pose else 0.0)
            self.resultado = None
            self.espera = {"tipo": tipo, "yaw": yaw_o, "t": ahora}
            self.log("voy a %s: me alineo y confirmo la escalera" % tipo)
            res.mensaje = "tomando la escalera de %s" % tipo
        elif o == "parar":
            self.espera = None
            if self.maniobra.activa and self.maniobra.en_escalera(self.pose[4] if self.pose else 0.0):
                self.maniobra.detener("parado por ti", ahora)
            elif self.maniobra.activa:
                self.maniobra.abortar("parado por ti", ahora)
            self.mover(0.0, 0.0, 0.0)
            res.mensaje = "parado"
        elif o == "seguir":
            if self.maniobra.estado == "DETENIDO":
                self.maniobra.seguir(ahora)
                res.mensaje = "sigo con la escalera"
            else:
                res.ok, res.mensaje = False, "no estoy detenido en una escalera"
        else:
            res.ok, res.mensaje = False, "orden desconocida"
        return res

    def publicar(self):
        ahora = time.monotonic()
        vistas = []
        if self.pose is not None:
            for tipo in ("subir", "bajar", "posible"):
                c = self.detector.confirmada(tipo, ahora, self.pose[:2])
                if c is None or not 0.3 < c.get("dist", 0.0) < 2.5:
                    continue
                if tipo == "posible" and any(v["tipo"] == "bajar" for v in vistas):
                    continue                    # ya se ve la escalera de verdad
                # 'posible': caida sin el primer escalon a la vista (de lejos una bajada se ve asi)
                vistas.append({"tipo": "bajar" if tipo == "posible" else tipo, "posible": tipo == "posible",
                               "x": float(c["E"][0]), "y": float(c["E"][1]),
                               "yaw": float(c["yaw_n"]), "ancho": float(c["ancho"]),
                               "abierta": bool(c["abierta"]), "dist": float(c["dist"]), "z": float(c["z"])})
        self.pub_esc.publish(String(data=json.dumps(vistas)))
        m = self.maniobra
        est = {"estado": "ACERCANDOME" if self.espera is not None else m.estado, "info": m.info,
               "activa": bool(m.activa or self.espera is not None),
               # lo que el lidar ve delante y por que (no) es una bajada: 'go2 escaleras'
               "delante": self.detector.centro, "diagnostico": self.detector.motivo,
               "diagnostico_subida": self.detector.motivo_subida,
               "perfil": self.detector.perfil,
               # la escalera que ve delante: hacia donde va (grados respecto del perro) y su centro
               "rumbo_escalera": None if (self.detector.orient is None or ahora - self.detector.orient["t"] > 1.0) else
               {"grados": round(math.degrees(self.detector.orient["ang"]), 1),
                "centro_cm": round(100 * self.detector.orient["centro"]),
                "paredes": self.detector.orient["paredes"], "contrahuellas": self.detector.orient["contras"]},
               "corrido_cm": None if "error_lateral" not in m.m else round(-100 * m.m["error_lateral"]),
               "alto_cuerpo": None if self.detector.alto_cuerpo is None else round(self.detector.alto_cuerpo, 3),
               "en_escalera": bool(m.en_escalera(self.pose[4] if self.pose else 0.0)),
               "detenido": m.estado == "DETENIDO", "motivo_detenido": m.m.get("motivo_det", ""),
               "resultado": self.resultado, "log": self.logs[-1] if self.logs else "",
               "suelo": None if self.suelo is None else round(self.suelo, 3),
               "adelante": None if not self.holg else [round(v, 2) for v in self.holg["adelante"]],
               "puntos": int(len(self.memoria.k))}
        self.pub_est.publish(String(data=json.dumps(est)))


def main():
    rclpy.init()
    n = Escaleras()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
