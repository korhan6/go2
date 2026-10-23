"""go2_driver: puente DIRECTO por DDS entre el Go2 y ROS 2 (sin WebRTC, sin bridge intermedio).

Con el cable Ethernet la laptop queda en la red interna del perro (como la mochila EDU).
Los topicos del perro aparecen tal cual en ROS 2 (rmw_cyclonedds):
  /utlidar/robot_odom   nav_msgs/Odometry      (odometria lidar-inercial, ~150 Hz)
  /lf/sportmodestate    unitree_go/SportModeState (altura del cuerpo, modo; 20 Hz)
  /lf/lowstate          unitree_go/LowState    (bateria, control remoto; 20 Hz)
  /api/sport/request    unitree_api/Request    (ordenes: Move=1008, StandUp=1004...)

Este nodo:
  - publica /odom y el TF odom->base_link con la HORA DE LA LAPTOP (el reloj del perro esta
    corrido horas: con sus stamps Nav2 y el TF fallan);
  - recibe /cmd_vel (Nav2) y /cmd_vel_prioridad (calibracion, teleop) y manda Move a 20 Hz desde
    un timer propio. Si nadie manda nada por 0.4 s, el perro se detiene;
  - si alguien mueve los joysticks del control, suelta el mando (el control manda);
  - servicios /go2/levantar, /go2/parar, /go2/acostar (StandDown), /go2/bloquear y /go2/desbloquear
    (bloqueado = de pie con las juntas trabadas e IGNORA toda orden de movimiento: Nav2, teleop, control);
  - APAGA la evasion de obstaculos PROPIA de Unitree (la de la app): con ella prendida el perro
    frena solo cerca de las cosas y se queda pisando en el sitio ("temblando");
  - zona muerta: ordenes minusculas (que el perro no puede caminar) = quieto, no pisar en el sitio.
"""
import json
import math
import os
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from unitree_api.msg import Request, Response
try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:
    CollisionMonitorState = None
from unitree_go.msg import LowState, SportModeState

from . import ajustes

API_MOVE = 1008
API_STOPMOVE = 1003
API_STANDUP = 1004             # "lock stand": de pie con las juntas trabadas (no equilibra ni camina)
API_STANDDOWN = 1005           # echarse en el piso
API_BALANCESTAND = 1002
API_FREEWALK = 2045
API_BODYHEIGHT = 1013          # altura del cuerpo RELATIVA a la normal, {"data": m}, de -0.18 a +0.03
ALTURA_MIN, ALTURA_MAX = -0.18, 0.03
EVITA_SET, EVITA_GET = 1001, 1002          # servicio "obstacles_avoid" de Unitree
# orden de los motores del Go2 en LowState.motor_state (unitree_sdk2) = nombres del urdf/go2.urdf
JUNTAS = ["%s_%s_joint" % (pata, j) for pata in ("FR", "FL", "RR", "RL") for j in ("hip", "thigh", "calf")]


def seguro(f):
    import functools
    import traceback

    @functools.wraps(f)
    def g(self, *a, **k):
        try:
            return f(self, *a, **k)
        except Exception as e:
            self.get_logger().error("%s: %r\n%s" % (f.__name__, e, traceback.format_exc(limit=4)),
                                    throttle_duration_sec=5.0)
    return g


class Driver(Node):
    def __init__(self):
        super().__init__("go2_driver")
        p = ajustes.declarador(self)          # 0 donde va 0.0 no tumba el nodo
        self.vx_max = p("vx_max", 0.8).value
        self.vy_max = p("vy_max", 0.4).value
        self.wz_max = p("wz_max", 1.2).value
        self.t_vigencia = p("t_vigencia", 0.4).value       # s que vale una orden de velocidad
        self.marcha_libre = p("freewalk", True).value       # firmware >= 1.1.7: FreeWalk
        self.pausa_control = p("pausa_con_control", True).value
        self.apagar_evita = p("apagar_evitacion_unitree", True).value
        # zona muerta: por debajo de esto el Go2 no camina, solo pisa en el sitio
        self.min_v = p("v_min", 0.02).value
        self.min_w = p("w_min", 0.05).value
        # el Go2 casi no avanza por debajo de ~0.20 m/s (pisa en el sitio): si Nav2 pide menos y hay
        # espacio libre en esa direccion, sube a esta velocidad
        self.v_efectiva = p("v_efectiva", 0.20).value
        # FRENO segun la distancia (todas las ordenes: Nav2, control, teleop). El Go2 Air no camina
        # por debajo de ~0.20 m/s y tarda ~0.3 s en parar: a 0.20 m/s recorre ~6 cm despues de la
        # orden. Para quedar a 'parar_a' (5 cm) del cuerpo hay que cortar a 5 + 6 = 11 cm, y cerca
        # de algo ir a 0.20 (no menos: se quedaria pisando en el sitio, "frena lejos")
        self.parar_a = p("parar_a", 0.05).value
        self.latencia = p("latencia_parada", 0.30).value
        self.umbral_parada = self.parar_a + self.v_efectiva * self.latencia
        self.frenadas = 0
        self.w_efectiva = p("w_efectiva", 0.40).value
        # giro en el sitio MAS CHICO que esto = "ya casi estoy alineado": se para en vez de subirlo
        # a w_efectiva. Antes cualquier correccion chica de Nav2 se volvia 0.40 rad/s, el perro se
        # pasaba de la linea, Nav2 corregia al otro lado y asi sin fin (zigzag)
        self.w_quieto = p("w_quieto", 0.20).value
        # EMPUJE para Nav2: si el perro avanza menos del 40 % de lo que se le manda (pisa en el sitio),
        # la orden sube de a poco (x1.15 cada 0.6 s, giro incluido: misma curva) hasta v_empuje_max,
        # SOLO con espacio libre en esa direccion y sin el monitor de choques frenando
        self.v_empuje_max = p("v_empuje_max", 0.45).value
        self.emp_k, self.emp_t, self.emp_p, self.emp_v = 1.0, 0.0, None, 0.0
        self.pose_xy = None
        # la compensacion de giro (go2 calibrar_mov) nunca pide mas de esto x lo que quiere Nav2
        self.comp_w_max = p("compensacion_giro_max", 1.5).value
        self.holguras = {}
        self.t_holg = 0.0
        # respuesta REAL del perro a las ordenes (go2 calibrar_mov): se compensa
        self.ruta_mov = os.path.expanduser("~/.go2_nav/movimiento.json")
        self.curvas, self.t_mov_archivo, self.t_mov_revisado = None, None, 0.0
        self.en_escalera = False
        self.calibrando_mov = False      # mientras se mide, sin compensar

        self.tf = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_patas = self.create_publisher(JointState, "joint_states", 10)
        self.t_patas = 0.0
        self.pub_req = self.create_publisher(Request, "/api/sport/request", 10)
        self.pub_bat = self.create_publisher(Float32, "/go2/bateria", 1)
        self.pub_rem = self.create_publisher(Bool, "/go2/control_remoto", 1)
        self.pub_lidar = self.create_publisher(String, "/utlidar/switch", 1)
        self.pub_diag = self.create_publisher(String, "/go2/driver_estado", 1)
        self.pub_evita = self.create_publisher(Request, "/api/obstacles_avoid/request", 10)

        # odometria: lidar = /utlidar/robot_odom (la del L1 en el perro) | patas = SportModeState
        # (patas + IMU del cuerpo). 'go2 escala' mide las dos contra el lidar crudo y elige.
        self.fuente_odom = p("fuente_odom", "lidar").value
        self.escala_odom = float(p("escala_odom", 1.0).value)
        self.odom0 = None
        self.t_pub_odom = 0.0
        self.fuente_actual = None
        self.ultima_pose = None
        self.enganche = {}
        self.t_glim = 0.0
        # map->odom: lo publica este nodo con la correccion de go2_slam3d (con slam_toolbox, no)
        self.tf_mapa = bool(p("tf_mapa", True).value)
        self.correccion = (0.0, 0.0, 0.0, 0.0)
        lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Float64MultiArray, "/go2/correccion",
                                 lambda m: setattr(self, "correccion", tuple(m.data[:4])) if len(m.data) >= 4 else None, lat)
        self.t_sport_rapido = 0.0
        self.pub_odom_perro = self.create_publisher(Odometry, "/go2/odom_perro", 50)
        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, "/utlidar/robot_odom", self.cb_odom, sd)
        if self.fuente_odom == "glim":
            self.create_subscription(Odometry, p("topico_glim", "/glim_ros/odom").value, self.cb_glim, 50)
        self.create_subscription(SportModeState, "/sportmodestate", self.cb_sport_rapido, sd)
        self.create_subscription(SportModeState, "/lf/sportmodestate", self.cb_sport, sd)
        self.create_subscription(LowState, "/lf/lowstate", self.cb_low, sd)
        self.create_subscription(Twist, "cmd_vel", self.cb_cmd, 10)
        self.create_subscription(Twist, "cmd_vel_prioridad", self.cb_cmd_prio, 10)
        self.create_subscription(String, "/go2/marcha", self.cb_marcha, 10)
        self.create_subscription(Response, "/api/obstacles_avoid/response", self.cb_evita, 10)
        # altura del cuerpo en la escalera: bajando en reversa el L1 (bajo el menton) rozaba el borde
        # del escalon de arriba cuando el perro se agachaba. Se sube el cuerpo con BodyHeight y se
        # comprueba con la respuesta del perro y la altura medida (SportModeState.body_height)
        self.altura_esc = p("altura_escaleras", 0.03).value
        self.create_subscription(Response, "/api/sport/response", self.cb_sport_resp, 10)
        self.altura_pedida, self.altura_resp, self.altura_info = 0.0, None, ""
        self.rid_altura = None
        self.create_subscription(String, "/go2/holguras", self.cb_holg, 5)        # ~15 Hz, cada nube
        self.frenado = {}            # lado -> True mientras esta frenado (histeresis 11 -> 15 cm)
        # sin nubes del lidar Nav2 maneja a ciegas (y el TF del mapa se atrasa): pausa
        self.create_subscription(Header, "/go2/lidar_vivo", self.cb_vivo, 5)
        self.t_vivo = 0.0
        self.pausas_lidar = 0
        self.create_subscription(Bool, "/go2/midiendo_mov", lambda m: setattr(self, "calibrando_mov", m.data), 10)
        # el monitor de choques avisa cuando ESTA frenando: ahi nunca se sube la velocidad
        self.cm_accion = 0
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     lambda m: setattr(self, "cm_accion", int(m.action_type)), 10)
        self.create_service(Trigger, "/go2/levantar", self.srv_levantar)
        self.create_service(Trigger, "/go2/parar", self.srv_parar)
        self.create_service(Trigger, "/go2/acostar", self.srv_acostar)
        self.create_service(Trigger, "/go2/bloquear", self.srv_bloquear)
        self.create_service(Trigger, "/go2/desbloquear", self.srv_desbloquear)
        self.create_service(Trigger, "/go2/bloquear_alternar", self.srv_alternar)
        self.bloqueado = False       # no se mueve con NADA hasta 'go2 desbloquear'
        self.acostado = False        # echado: no se mueve hasta 'go2 levantar'

        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = 0.0
        self.cmd_prio = (0.0, 0.0, 0.0)
        self.t_prio = 0.0
        self.t_remoto = 0.0
        self.altura = None
        self.modo = None
        self.soc = None
        self.n_odom = 0
        self.t_odom = 0.0
        self.movia = False
        self.ceros = 0
        self.rid = int(time.time() * 1000) % 1000000000
        self.envios = 0
        self.evita = None            # evasion de Unitree: True / False / None (sin respuesta)
        self.ultima_orden = (0.0, 0.0, 0.0)
        self.muertas = 0
        self.subidas = 0
        self.create_timer(0.05, self.lazo)          # 20 Hz
        self.create_timer(1.0, self.diag)
        self.create_timer(0.5, self.patas_de_pie)
        self.create_timer(2.0, self.prender_lidar)
        self.create_timer(3.0, self.revisar_evitacion)
        self.get_logger().info("go2_driver listo: /cmd_vel -> Move a 20 Hz, odometria '%s' (escala %.3f)"
                               % (self.fuente_odom, self.escala_odom))

    # ------------------------------------------------------------------ entradas
    @seguro
    def cb_odom(self, m):
        if self.fuente_odom == "lidar":
            q = m.pose.pose.orientation
            p = m.pose.pose.position
            v = m.twist.twist
            self.publicar_odom(m.header.stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                               (v.linear.x, v.linear.y, v.linear.z), (v.angular.x, v.angular.y, v.angular.z),
                               fuente="lidar")

    @seguro
    def publicar_odom(self, stamp_perro, pos, quat, vlin, vang, fuente="patas"):
        """/odom + TF odom->base_link (hora de la laptop) y /go2/odom_perro (hora del perro, para
        go2_nube y go2_slam3d). escala_odom corrige la escala de x, y de las patas ('go2 escala').
        Si cambia la fuente (GLIM se calla -> patas, y vuelve), la pose sigue CONTINUA: la nueva
        fuente se engancha donde quedo la anterior (sin saltos para Nav2)."""
        ahora_m = time.monotonic()
        if ahora_m - self.t_pub_odom < 0.01:
            return                       # /sportmodestate llega a ~500 Hz: TF a 100 Hz basta (y no ahoga a Nav2)
        self.t_pub_odom = ahora_m
        k = self.escala_odom if fuente == "patas" else 1.0
        if fuente == "patas":
            if self.odom0 is None:
                self.odom0 = pos
            pos = (self.odom0[0] + k * (pos[0] - self.odom0[0]), self.odom0[1] + k * (pos[1] - self.odom0[1]), pos[2])
        yaw = math.atan2(2 * (quat[3] * quat[2] + quat[0] * quat[1]), 1 - 2 * (quat[1] ** 2 + quat[2] ** 2))
        if fuente != self.fuente_actual:
            if self.ultima_pose is not None:
                # enganche: correccion (x, y, z, giro) que lleva la pose nueva a la ultima publicada
                lx, ly, lz, lyaw = self.ultima_pose
                dyaw = lyaw - yaw
                c, s_ = math.cos(dyaw), math.sin(dyaw)
                self.enganche[fuente] = (lx - (c * pos[0] - s_ * pos[1]), ly - (s_ * pos[0] + c * pos[1]), lz - pos[2], dyaw)
                self.get_logger().warn("odometria: %s -> %s (enganchada sin salto)" % (self.fuente_actual, fuente))
            self.fuente_actual = fuente
        ex, ey, ez, eyaw = self.enganche.get(fuente, (0.0, 0.0, 0.0, 0.0))
        if eyaw or ex or ey or ez:
            c, s_ = math.cos(eyaw), math.sin(eyaw)
            pos = (c * pos[0] - s_ * pos[1] + ex, s_ * pos[0] + c * pos[1] + ey, pos[2] + ez)
            qz, qw = math.sin(eyaw / 2), math.cos(eyaw / 2)          # q_enganche * q
            x0, y0, z0, w0 = quat
            quat = (qw * x0 - qz * y0, qw * y0 + qz * x0, qw * z0 + qz * w0, qw * w0 - qz * z0)
            yaw += eyaw
        x, y, z = pos
        self.ultima_pose = (x, y, z, yaw)
        ahora = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = ahora
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        r = o.pose.pose.orientation
        r.x, r.y, r.z, r.w = quat
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = k * vlin[0], k * vlin[1], vlin[2]
        o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = vang
        self.pub_odom.publish(o)
        o.header.stamp = stamp_perro
        self.pub_odom_perro.publish(o)
        t = TransformStamped()
        t.header.stamp = ahora
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation = o.pose.pose.orientation
        if self.tf_mapa:
            # map->odom (correccion del SLAM, /go2/correccion) con el MISMO sello: Nav2 nunca ve
            # un TF del mapa viejo ni dos poses distintas del perro
            cx, cy, cz, cyaw = self.correccion
            tm = TransformStamped()
            tm.header.stamp = ahora
            tm.header.frame_id, tm.child_frame_id = "map", "odom"
            tm.transform.translation.x, tm.transform.translation.y, tm.transform.translation.z = cx, cy, cz
            tm.transform.rotation.z, tm.transform.rotation.w = math.sin(cyaw / 2), math.cos(cyaw / 2)
            self.tf.sendTransform([t, tm])
        else:
            self.tf.sendTransform(t)
        self.pose_xy = (x, y)
        self.n_odom += 1
        self.t_odom = time.monotonic()

    @seguro
    def cb_glim(self, m):
        """Odometria de GLIM (pose de la IMU del cuerpo en su 'odom') -> base_link."""
        self.t_glim = time.monotonic()
        q, p, v = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
        x, y, z, w = q.x, q.y, q.z, q.w
        # base = imu * T_imu_base (la IMU esta en (-0.02557, 0, 0.04232) del centro, sin giro)
        tx, ty, tz = 0.02557, 0.0, -0.04232
        bx = p.x + (1 - 2 * (y * y + z * z)) * tx + 2 * (x * y - z * w) * ty + 2 * (x * z + y * w) * tz
        by = p.y + 2 * (x * y + z * w) * tx + (1 - 2 * (x * x + z * z)) * ty + 2 * (y * z - x * w) * tz
        bz = p.z + 2 * (x * z - y * w) * tx + 2 * (y * z + x * w) * ty + (1 - 2 * (x * x + y * y)) * tz
        self.publicar_odom(m.header.stamp, (bx, by, bz), (x, y, z, w), (v.linear.x, v.linear.y, v.linear.z),
                           (v.angular.x, v.angular.y, v.angular.z), fuente="glim")

    @seguro
    def cb_sport_rapido(self, m):
        self.t_sport_rapido = time.monotonic()
        self._odom_patas(m)

    def _odom_patas(self, m):
        if self.fuente_odom == "glim":
            if time.monotonic() - self.t_glim < 0.5:
                return                   # manda GLIM; las patas solo si GLIM se calla (respaldo)
        elif self.fuente_odom != "patas":
            return
        q = m.imu_state.quaternion                      # w x y z
        st = Header().stamp
        st.sec, st.nanosec = int(m.stamp.sec), int(m.stamp.nanosec)
        self.publicar_odom(st, tuple(float(v) for v in m.position), (float(q[1]), float(q[2]), float(q[3]), float(q[0])),
                           tuple(float(v) for v in m.velocity), (0.0, 0.0, float(m.yaw_speed)))

    @seguro
    def cb_sport(self, m):
        self.altura = float(m.body_height)
        self.modo = int(m.mode)
        if time.monotonic() - self.t_sport_rapido > 0.5:      # sin /sportmodestate rapido: esta (20 Hz)
            self._odom_patas(m)

    @seguro
    def cb_low(self, m):
        self.soc = int(m.bms_state.soc)
        # angulos de los 12 motores -> /joint_states: el modelo del perro en RViz mueve las patas
        q = [float(m.motor_state[i].q) for i in range(12)]
        if any(abs(v) > 1e-3 for v in q) and time.monotonic() - self.t_patas > 0.05:
            self.t_patas = time.monotonic()
            self.publicar_patas(q)
        try:
            wr = bytes(bytearray(m.wireless_remote))
            lx, rx, ry, _l2, ly = struct.unpack_from("<5f", wr, 4)
            if max(abs(lx), abs(rx), abs(ry), abs(ly)) > 0.15:
                self.t_remoto = time.monotonic()
        except Exception:
            pass

    def publicar_patas(self, q):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name, js.position = list(JUNTAS), list(q)
        self.pub_patas.publish(js)

    def patas_de_pie(self):
        """Sin angulos de los motores (simulador, o todavia no llegan): el perro de pie."""
        if time.monotonic() - self.t_patas > 1.0:
            self.publicar_patas([0.0, 0.8, -1.5] * 4)

    @seguro
    def cb_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.t_cmd = time.monotonic()

    @seguro
    def cb_cmd_prio(self, m):
        self.cmd_prio = (m.linear.x, m.linear.y, m.angular.z)
        self.t_prio = time.monotonic()

    def cb_vivo(self, _m):
        self.t_vivo = time.monotonic()

    def lidar_mudo(self):
        return self.t_vivo > 0 and time.monotonic() - self.t_vivo > 0.7

    def cb_holg(self, m):
        try:
            self.holguras = json.loads(m.data)
            self.t_holg = time.monotonic()
        except ValueError:
            pass

    # ------------------------------------------------------------------ compensacion
    def cargar_curvas(self):
        ahora = time.monotonic()
        if ahora - self.t_mov_revisado < 3.0:
            return
        self.t_mov_revisado = ahora
        try:
            t = os.path.getmtime(self.ruta_mov)
        except OSError:
            self.curvas = None
            return
        if t == self.t_mov_archivo:
            return
        try:
            with open(self.ruta_mov) as f:
                d = json.load(f)
            self.curvas = {k: sorted([(0.0, 0.0)] + [tuple(p) for p in d[k]]) for k in ("v", "w")}
            self.t_mov_archivo = t
            self.get_logger().info("compensacion de movimiento cargada: v %s | w %s" % (d["v"], d["w"]))
        except (OSError, ValueError, KeyError, TypeError):
            self.curvas = None

    @staticmethod
    def inversa(curva, deseado):
        """Que orden hay que dar para que el perro haga 'deseado' de verdad (curva medida)."""
        real = 0.0
        pts = []
        for c, r in curva:
            real = max(real, r)                    # monotona
            pts.append((c, real))
        for (c0, r0), (c1, r1) in zip(pts, pts[1:]):
            if r1 >= deseado and r1 > r0:
                return c0 + (c1 - c0) * (deseado - r0) / (r1 - r0)
        c, r = pts[-1]
        return deseado * (c / r) if r > 0.02 else deseado

    def compensar(self, vx, vy, wz):
        if self.curvas is None or self.en_escalera:
            return vx, vy, wz
        v = math.hypot(vx, vy)
        if v >= self.min_v:
            k = min(self.vx_max, self.inversa(self.curvas["v"], v)) / v
            vx, vy = vx * k, vy * k
        if abs(wz) >= self.min_w:
            w = min(self.inversa(self.curvas["w"], abs(wz)), self.comp_w_max * abs(wz), self.wz_max)
            wz = math.copysign(w, wz)
        return vx, vy, wz

    def efectiva(self, vx, vy, wz):
        """Orden chica pero real -> la minima que el Go2 si camina (solo con espacio libre)."""
        v = math.hypot(vx, vy)
        H = self.holguras if time.monotonic() - self.t_holg < 2.0 else None
        if H is not None and self.min_v <= v < self.v_efectiva and self.cm_accion == 0:
            if abs(vx) >= abs(vy):
                lado = "adelante" if vx > 0 else "atras"
                costados = ("izq", "der")
            else:
                lado = "izq" if vy > 0 else "der"
                costados = ("adelante",)
            d = H.get(lado)
            libre = (d is None and lado != "atras") or (d is not None and d >= self.umbral_parada)
            # y que no vaya rozando: los costados con algo de aire
            libre = libre and all(H.get(k) is None or H.get(k) >= 0.10 for k in costados)
            if libre:
                # el giro se escala IGUAL que el avance: misma curva que calculo Nav2, solo mas
                # rapido. Antes subia solo el avance (0.08 -> 0.20 m/s con el mismo giro): el arco
                # salia 2.5 veces mas abierto y la esquina del perro cortaba el marco de la puerta
                k = min(self.v_efectiva / v, self.wz_max / abs(wz)) if abs(wz) > 1e-3 else self.v_efectiva / v
                if k > 1.0:
                    vx, vy, wz = vx * k, vy * k, wz * k
                    self.subidas += 1
        elif v < self.min_v and abs(wz) < self.w_efectiva:
            # giro en el sitio: una correccion minima se ignora (casi alineado: mejor parar que
            # pasarse); una correccion real se sube al minimo con el que el Go2 si gira
            wz = 0.0 if abs(wz) < self.w_quieto else math.copysign(self.w_efectiva, wz)
        return vx, vy, wz

    def gobernar(self, vx, vy):
        """Velocidad maxima HACIA cada lado segun lo libre que hay (holguras del limpiador, desde el
        borde del cuerpo): < umbral (11 cm) -> no avanza hacia ahi; hasta umbral + 20 cm -> 0.20 m/s
        (lo minimo que camina); mas lejos sube 1.2 m/s por metro. Alejarse siempre se puede."""
        H = self.holguras if time.monotonic() - self.t_holg < 1.0 else None
        if H is None:
            return vx, vy

        def tope(d, lado):
            if d is None:
                return None
            # histeresis: frena a < umbral (11 cm) y suelta recien a > umbral + 4 cm (no tironea)
            if d < self.umbral_parada or (self.frenado.get(lado) and d < self.umbral_parada + 0.04):
                self.frenado[lado] = True
                return 0.0
            self.frenado[lado] = False
            return max(self.v_efectiva, self.v_efectiva + 1.2 * (d - self.umbral_parada - 0.20))
        for comp, lado_pos, lado_neg in ((0, "adelante", "atras"), (1, "izq", "der")):
            v = (vx, vy)[comp]
            lado = lado_pos if v > 0 else lado_neg
            t = tope(H.get(lado), lado) if v else None
            if t is not None and abs(v) > t:
                v = math.copysign(t, v)
                if t == 0.0:
                    self.frenadas += 1
                if comp == 0:
                    vx = v
                else:
                    vy = v
        return vx, vy

    def empuje(self, vx, vy, wz):
        """Si el perro no avanza lo que se le pide, subir la orden de a poco (ver __init__)."""
        v = math.hypot(vx, vy)
        ahora = time.monotonic()
        H = self.holguras if ahora - self.t_holg < 2.0 else None
        if v < self.min_v or self.en_escalera or self.cm_accion != 0 or H is None or self.pose_xy is None:
            self.emp_k, self.emp_p = 1.0, None
            return vx, vy, wz
        lado = ("adelante" if vx > 0 else "atras") if abs(vx) >= abs(vy) else ("izq" if vy > 0 else "der")
        d = H.get(lado)
        if d is None or d < 0.35:                 # sin espacio medido en esa direccion: nunca empujar
            self.emp_k, self.emp_p = 1.0, None
            return vx, vy, wz
        if self.emp_p is None:
            self.emp_t, self.emp_p = ahora, self.pose_xy
        elif ahora - self.emp_t >= 0.6:
            real = math.hypot(self.pose_xy[0] - self.emp_p[0], self.pose_xy[1] - self.emp_p[1]) / (ahora - self.emp_t)
            if real < 0.4 * self.emp_v:
                self.emp_k = min(self.emp_k * 1.15, self.v_empuje_max / v)
            elif real > 0.8 * self.emp_v:
                self.emp_k = max(1.0, self.emp_k * 0.95)          # ya avanza: vuelve de a poco
            self.emp_t, self.emp_p = ahora, self.pose_xy
        k = max(1.0, min(self.emp_k, self.v_empuje_max / v, self.wz_max / abs(wz) if abs(wz) > 1e-3 else 9.0))
        self.emp_v = v * k
        return vx * k, vy * k, wz * k

    def cb_evita(self, m):
        if m.header.identity.api_id != EVITA_GET or m.header.status.code != 0:
            return
        try:
            self.evita = bool(json.loads(m.data).get("enable"))
        except (ValueError, AttributeError):
            pass

    def revisar_evitacion(self):
        """Pregunta si la evasion de Unitree esta prendida y, si lo esta, la apaga."""
        if self.evita and self.apagar_evita:
            self.get_logger().warn("evasion de obstaculos de Unitree PRENDIDA: la apago "
                                   "(frena al perro y lo deja pisando en el sitio)")
            self._req(EVITA_SET, {"enable": False}, pub=self.pub_evita)
        self._req(EVITA_GET, None, sin_respuesta=False, pub=self.pub_evita)

    def cb_marcha(self, m):
        """Marcha para escaleras (la pide go2_escaleras). Firmware >= 1.1.7 (MCF): FreeWalk es la
        marcha agil que sube y baja escalones. Ademas: 'altura X' (m, relativa) para probar."""
        if m.data == "escaleras_off":
            self.en_escalera = False
            if self.altura_pedida:
                self.poner_altura(0.0)
        if m.data == "escaleras_on":
            self.en_escalera = True
            self.get_logger().info("marcha de escaleras: FreeWalk, cuerpo %+.0f cm" % (100 * self.altura_esc))
            self._req(API_FREEWALK)
            # despues del cambio de marcha (que puede volver la altura a la normal)
            threading.Timer(0.6, lambda: self.poner_altura(self.altura_esc)).start()
        elif m.data == "balancear":
            self._req(API_BALANCESTAND)
            if self.marcha_libre:
                threading.Timer(0.3, lambda: self._req(API_FREEWALK)).start()
            if self.en_escalera:
                threading.Timer(0.9, lambda: self.poner_altura(self.altura_esc)).start()
        elif m.data.startswith("altura"):
            try:
                self.poner_altura(float(m.data.split()[1]))
            except (IndexError, ValueError):
                pass

    def poner_altura(self, h):
        """Pide la altura del cuerpo (relativa) y despues de 1.5 s dice si el perro la tomo."""
        h = max(ALTURA_MIN, min(ALTURA_MAX, float(h)))
        antes = self.altura
        self.altura_pedida, self.altura_resp = h, None
        self.rid_altura = self.rid + 1
        self._req(API_BODYHEIGHT, {"data": h}, sin_respuesta=False)

        def revisar():
            ahora = self.altura
            cambio = (ahora - antes) if (ahora is not None and antes is not None) else None
            if self.altura_resp is None:
                r = "el perro no respondio"
            elif self.altura_resp == 0:
                r = "el perro la ACEPTO"
            else:
                r = "el perro la RECHAZO (codigo %s: este firmware/marcha no la permite)" % self.altura_resp
            self.altura_info = "altura %+.0f cm: %s%s" % (
                100 * h, r, "" if cambio is None else ", la altura medida cambio %+.1f cm (%.2f -> %.2f m)"
                % (100 * cambio, antes, ahora))
            self.get_logger().info(self.altura_info)
        threading.Timer(1.5, revisar).start()

    def cb_sport_resp(self, m):
        if m.header.identity.api_id == API_BODYHEIGHT and m.header.identity.id == self.rid_altura:
            self.altura_resp = int(m.header.status.code)

    # ------------------------------------------------------------------ salida
    def _req(self, api, parametro=None, sin_respuesta=True, pub=None):
        r = Request()
        self.rid += 1
        r.header.identity.id = self.rid
        r.header.identity.api_id = api
        r.header.policy.noreply = sin_respuesta
        r.parameter = json.dumps(parametro) if parametro is not None else ""
        (pub or self.pub_req).publish(r)

    def remoto_activo(self):
        return self.pausa_control and time.monotonic() - self.t_remoto < 1.0

    @seguro
    def lazo(self):
        ahora = time.monotonic()
        nav = False
        if ahora - self.t_prio < self.t_vigencia:
            vx, vy, wz = self.cmd_prio
        elif ahora - self.t_cmd < self.t_vigencia:
            vx, vy, wz = self.cmd
            nav = True
        else:
            vx = vy = wz = 0.0
        if self.remoto_activo() or self.bloqueado or self.acostado:
            vx = vy = wz = 0.0
        if nav and self.lidar_mudo() and (vx or vy or wz):
            vx = vy = wz = 0.0              # quieto hasta que vuelvan las nubes
            self.pausas_lidar += 1
        vx = max(-self.vx_max, min(self.vx_max, vx))
        vy = max(-self.vy_max, min(self.vy_max, vy))
        wz = max(-self.wz_max, min(self.wz_max, wz))
        if (vx or vy or wz) and abs(vx) < self.min_v and abs(vy) < self.min_v and abs(wz) < self.min_w:
            # orden minuscula: el perro solo pisaria en el sitio ("temblando") -> quieto
            self.muertas += 1
            vx = vy = wz = 0.0
        self.cargar_curvas()
        if (vx or vy or wz) and not self.calibrando_mov:
            vx, vy, wz = self.compensar(vx, vy, wz)
        if nav and (vx or vy or wz):
            # minimo para Nav2 (la escalera, la calibracion y el escape mandan su velocidad)
            vx, vy, wz = self.efectiva(vx, vy, wz)
            vx, vy, wz = self.empuje(vx, vy, wz)
        else:
            self.emp_k, self.emp_p = 1.0, None
        if (vx or vy) and not self.calibrando_mov and not self.en_escalera:
            vx, vy = self.gobernar(vx, vy)
        self.ultima_orden = (vx, vy, wz)
        if abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(wz) > 1e-3:
            self._req(API_MOVE, {"x": vx, "y": vy, "z": wz})
            self.envios += 1
            self.movia, self.ceros = True, 0
        elif self.movia:
            # al parar: Move(0) unas veces y StopMove; despues silencio (el control queda libre)
            self._req(API_MOVE, {"x": 0.0, "y": 0.0, "z": 0.0})
            self.ceros += 1
            if self.ceros >= 3:
                self._req(API_STOPMOVE)
                self.movia = False

    def prender_lidar(self):
        # la app puede apagar el L1 (ahorro); esto lo deja prendido
        self.pub_lidar.publish(String(data="ON"))

    @seguro
    def diag(self):
        if self.soc is not None:
            self.pub_bat.publish(Float32(data=float(self.soc)))
        self.pub_rem.publish(Bool(data=self.remoto_activo()))
        d = {"odom_hz": self.n_odom, "odom_hace_s": round(time.monotonic() - self.t_odom, 2) if self.t_odom else None,
             "altura": self.altura, "modo": self.modo, "bateria": self.soc, "move_hz": self.envios,
             "remoto": self.remoto_activo(), "evitacion_unitree": self.evita,
             "orden": [round(v, 2) for v in self.ultima_orden], "ordenes_minimas": self.muertas,
             "subidas": self.subidas, "compensado": self.curvas is not None, "empuje": round(self.emp_k, 2),
             "lidar_mudo_s": round(time.monotonic() - self.t_vivo, 1) if self.lidar_mudo() else 0.0,
             "pausas_lidar": self.pausas_lidar, "altura_info": self.altura_info,
             "bloqueado": self.bloqueado, "acostado": self.acostado, "frenadas": self.frenadas}
        self.n_odom = 0
        self.envios = 0
        self.muertas = 0
        self.subidas = 0
        self.pausas_lidar = 0
        self.pub_diag.publish(String(data=json.dumps(d)))

    # ------------------------------------------------------------------ servicios
    def _levantar(self):
        self._req(API_STANDUP)
        time.sleep(1.5)
        self._req(API_BALANCESTAND)
        time.sleep(1.0)
        if self.marcha_libre:
            self._req(API_FREEWALK)

    def srv_levantar(self, req, res):
        self.acostado = False
        if self.altura is not None and self.altura > 0.22:
            res.success, res.message = True, "ya estaba de pie (altura %.2f m)" % self.altura
            return res
        threading.Thread(target=self._levantar, daemon=True).start()
        res.success, res.message = True, "levantando"
        return res

    def srv_parar(self, req, res):
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_prio = (0.0, 0.0, 0.0)
        self.t_cmd = self.t_prio = 0.0
        self._req(API_MOVE, {"x": 0.0, "y": 0.0, "z": 0.0})
        self._req(API_STOPMOVE)
        res.success, res.message = True, "parado"
        return res

    def _quieto_ya(self):
        # corta la orden en curso y deja el lazo en silencio (Move(0) a un perro echado lo levanta)
        self.cmd = self.cmd_prio = (0.0, 0.0, 0.0)
        self.t_cmd = self.t_prio = 0.0
        self.movia, self.ceros = False, 0
        self._req(API_MOVE, {"x": 0.0, "y": 0.0, "z": 0.0})
        self._req(API_STOPMOVE)

    def srv_acostar(self, req, res):
        self.acostado = True
        self._quieto_ya()

        def echar():
            time.sleep(0.5)
            self._req(API_STANDDOWN)
        threading.Thread(target=echar, daemon=True).start()
        self.get_logger().info("ACOSTADO: no se mueve hasta 'go2 levantar'")
        res.success, res.message = True, "acostandose (para pararlo: go2 levantar)"
        return res

    def srv_bloquear(self, req, res):
        self.bloqueado = True
        self._quieto_ya()
        if not self.acostado:
            threading.Thread(target=lambda: (time.sleep(0.5), self._req(API_STANDUP)), daemon=True).start()
        self.get_logger().warn("BLOQUEADO: ignoro Nav2, teleop y el control hasta 'go2 desbloquear'")
        res.success, res.message = True, "bloqueado: quieto y trabado (go2 desbloquear para soltarlo)"
        return res

    def srv_desbloquear(self, req, res):
        estaba = self.bloqueado
        self.bloqueado = False
        if estaba and not self.acostado:
            def soltar():
                self._req(API_BALANCESTAND)
                time.sleep(1.0)
                if self.marcha_libre:
                    self._req(API_FREEWALK)
            threading.Thread(target=soltar, daemon=True).start()
        self.get_logger().info("desbloqueado")
        res.success, res.message = True, "desbloqueado" if estaba else "no estaba bloqueado"
        return res

    def srv_alternar(self, req, res):
        return self.srv_desbloquear(req, res) if self.bloqueado else self.srv_bloquear(req, res)


def main():
    rclpy.init()
    n = Driver()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            n._req(API_STOPMOVE)
        except Exception:
            pass
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
