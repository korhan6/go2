"""go2_perro_sim: un Go2 FALSO que habla igual que el real por DDS, para probar todo sin el perro.

Publica los mismos topicos que el Go2 (con el reloj del perro corrido 13 h, como el real):
  /utlidar/cloud_deskewed, /utlidar/robot_odom, /lf/sportmodestate, /lf/lowstate
y obedece /api/sport/request (Move 1008, StopMove 1003, StandUp...).
El L1 simulado tiene los defectos medidos en el perro real: ve sus patas y cabeza, error de
altura de cerca, reflejos del piso brillante bajo el suelo y fantasmas pegados al cuerpo.

Mundos: casa (2 cuartos + pasillo + puerta), pasillo (cuarto con cama y pasillo angosto al lado),
dos_puertas (dos cuartos unidos por dos puertas), puerta (como la casa real: puerta de 72 cm con
la hoja abierta, pasillo y escalera que baja), simple.
"""
import json
import os
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32, String

from unitree_api.msg import Request
from unitree_go.msg import LowState, SportModeState

from . import mundo as M
from .nube import crear_nube

DESFASE_RELOJ = 13 * 3600.0     # el reloj del Go2 real estaba 13 h corrido


def mundo_pasillo():
    """Cuarto 4x4 con cama; al lado de la cama un pasillo de 0.62 m que lleva a otro cuarto."""
    m = M.Mundo(9.0, 6.0)
    m.rect_pared(4.0, 0.0, 4.1, 6.0)             # pared entre cuartos
    m.rect_libre(4.0, 4.6, 4.1, 5.22)            # hueco del pasillo (0.62 m)
    m.rect_pared(1.2, 3.2, 3.6, 4.6)             # cama (borde a 0.62 m de la pared de arriba)
    m.rect_pared(0.1, 1.0, 0.6, 2.2)             # ropero
    m.rect_pared(6.0, 1.0, 7.0, 1.6)             # mueble en el otro cuarto
    return m, (1.2, 1.4, 0.0)


def mundo_dos_puertas():
    m = M.Mundo(9.0, 7.0)
    m.rect_pared(4.0, 0.0, 4.1, 7.0)
    m.rect_libre(4.0, 1.4, 4.1, 2.3)             # puerta 1 (0.9 m)
    m.rect_libre(4.0, 4.7, 4.1, 5.6)             # puerta 2 (0.9 m)
    m.rect_pared(1.5, 3.2, 2.5, 3.8)
    return m, (1.0, 3.5, 0.0)


def elegir(nombre):
    if nombre == "puerta":
        return M.mundo_puerta()
    if nombre == "pasillo":
        return mundo_pasillo()
    if nombre == "dos_puertas":
        return mundo_dos_puertas()
    if nombre == "simple":
        return M.cuarto_simple(), (1.5, 1.5, 0.3)
    if nombre == "escalera":
        # la escalera angosta de la casa: el perro abajo, con el hocico a 13 cm del primer escalon
        # (y una pared 90 cm detras del pie de la escalera, para ver que la bajada para a tiempo)
        return M.escalera_pasillo(pared_abajo=2.1), (3.0 - 0.36 - 0.13, 1.425, 0.0)
    return M.casa_demo(), (1.5, 1.5, 0.3)


class PerroSim(Node):
    def __init__(self):
        super().__init__("go2_perro_sim")
        nombre = self.declare_parameter("mundo", "casa").value
        ruido = self.declare_parameter("ruido_real", True).value
        # fantasmas pegados al cuerpo: el perro real NO los tiene una vez corregido el error de
        # altura del L1 (grabacion del 18-oct). Se pueden activar para probar 'go2 calibrar'.
        fantasmas = self.declare_parameter("fantasmas", False).value
        # el L1 real da ~2000 puntos por nube (21600/s); el lidar simulado ve TODO lo visible
        # (decenas de miles). 0 = todo; 2500 = parecido al L1 (para probar "de verdad")
        self.puntos_nube = int(self.declare_parameter("puntos_por_nube", 0).value)
        mundo, (x, y, yaw) = elegir(nombre)
        inicio = self.declare_parameter("inicio", "").value      # "x,y,yaw" para empezar en otro lado
        if inicio:
            x, y, yaw = [float(v) for v in inicio.split(",")]
        # en la escalera, el lidar como el L1 de verdad (mas bajo, no ve atras ni huellas mas altas que el)
        self.r = M.RobotSim(mundo, x=x, y=y, yaw=yaw, ruido_real=ruido, l1_real=(nombre == "escalera"))
        self.r.fantasmas = bool(fantasmas)
        self.t0 = time.monotonic()
        sd = qos_profile_sensor_data
        self.pub_nube = self.create_publisher(PointCloud2, "/utlidar/cloud_deskewed", sd)
        self.pub_odom = self.create_publisher(Odometry, "/utlidar/robot_odom", sd)
        self.pub_sport = self.create_publisher(SportModeState, "/lf/sportmodestate", sd)
        self.pub_low = self.create_publisher(LowState, "/lf/lowstate", sd)
        self.pub_choques = self.create_publisher(Int32, "/sim/choques", 1)
        self.pub_verdad = self.create_publisher(String, "/sim/verdad", 1)
        self.create_subscription(Request, "/api/sport/request", self.cb_req, 50)
        # el perro simulado solo sube/baja escalones con la marcha de escaleras puesta
        self.create_subscription(String, "/go2/marcha", self.cb_marcha, 10)
        self.create_timer(0.02, self.paso)
        self.create_timer(0.05, self.pub_estado)
        self.create_timer(1.0, self.pub_info)
        self.n_move = 0
        try:
            self.respuesta = tuple(float(v) for v in os.environ.get("GO2_SIM_RESPUESTA", "1,0,1").split(","))
        except ValueError:
            self.respuesta = (1.0, 0.0, 1.0)
        self.get_logger().info("Go2 simulado en el mundo '%s' (ruido real: %s)" % (nombre, ruido))

    def stamp_perro(self):
        t = time.time() + DESFASE_RELOJ
        from builtin_interfaces.msg import Time
        return Time(sec=int(t), nanosec=int((t % 1) * 1e9))

    def cb_req(self, m):
        api = m.header.identity.api_id
        if api == 1008:
            try:
                p = json.loads(m.parameter)
                x, y, z = p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0)
                # perro "real": hace menos de lo pedido y no camina por debajo de una zona muerta
                # (GO2_SIM_RESPUESTA="factor_v,zona_muerta_v,factor_w", p.ej. "0.7,0.08,0.5")
                fv, zm, fw = self.respuesta
                v = math.hypot(x, y)
                k = (fv * max(0.0, v - zm) / v) if v > 1e-6 else 0.0
                self.r.mover(x * k, y * k, z * fw)
                self.r.t_ult_cmd = self.r.t
                self.n_move += 1
            except ValueError:
                pass
        elif api in (1003, 1001, 1005):
            self.r.mover(0.0, 0.0, 0.0)

    def cb_marcha(self, m):
        if m.data == "escaleras_on":
            self.r.escaleras = True
        elif m.data == "escaleras_off":
            self.r.escaleras = False

    def paso(self):
        self.r._fisica_hasta(time.monotonic() - self.t0)
        for _t, xyz in self.r.nubes_nuevas():
            if 0 < self.puntos_nube < len(xyz):
                xyz = xyz[self.r.rng.choice(len(xyz), self.puntos_nube, replace=False)]
            self.pub_nube.publish(crear_nube(xyz, "odom", self.stamp_perro()))

    def pub_estado(self):
        x, y, z, yaw, pitch, roll, _ = self.r.pose()
        o = Odometry()
        o.header.stamp = self.stamp_perro()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        q = o.pose.pose.orientation
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = self.r.v
        self.pub_odom.publish(o)
        s = SportModeState()
        s.body_height = 0.32
        s.imu_state.rpy = [float(roll), float(pitch), float(yaw)]
        s.mode = 1
        self.pub_sport.publish(s)

    def pub_info(self):
        lo = LowState()
        lo.bms_state.soc = 80
        self.pub_low.publish(lo)
        self.pub_choques.publish(Int32(data=int(self.r.choques)))
        self.pub_verdad.publish(String(data=json.dumps({"x": round(self.r.x, 2), "y": round(self.r.y, 2),
                                                        "yaw": round(self.r.yaw, 2), "choques": self.r.choques,
                                                        "move_hz": self.n_move,
                                                        "donde": self.r.donde_choques[-3:],
                                                        "z": round(self.r.z, 2)})))
        self.n_move = 0
        try:
            self.respuesta = tuple(float(v) for v in os.environ.get("GO2_SIM_RESPUESTA", "1,0,1").split(","))
        except ValueError:
            self.respuesta = (1.0, 0.0, 1.0)


def main():
    rclpy.init()
    n = PerroSim()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
