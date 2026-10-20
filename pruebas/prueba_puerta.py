"""Prueba de punta a punta (ROS 2 + Nav2 + slam_toolbox reales, perro simulado): el perro empieza
en un cuarto mirando una puerta de 72 cm (como la casa real, foto del 19-oct) y tiene que
cruzarla explorando solo.

    python3 pruebas/prueba_puerta.py [segundos] [puntos_por_nube] [explorar|meta]

    puntos_por_nube: 2500 = lidar ralo como el L1 real (por defecto); 0 = el lidar simulado completo
    explorar (por defecto): 'go2 explorar' y ver si cruza solo
    meta: mandarlo DIRECTO a un punto del pasillo al otro lado de la puerta (Nav2 navigate_to_pose):
          la misma puerta cada vez, sin el azar de la exploracion (para comparar cambios)
Sale con codigo 0 si cruzo la puerta sin chocar.
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from go2_nav_interfaces.srv import Comando

T_MAX = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
PUNTOS = sys.argv[2] if len(sys.argv) > 2 else "2500"
MODO = sys.argv[3] if len(sys.argv) > 3 else "explorar"
Y_PUERTA = 4.1          # la pared del cuarto termina en y = 4.1: pasado esto, cruzo
LOG = os.environ.get("PRUEBA_LOG", "/tmp/prueba_puerta.log")

lanz = subprocess.Popen(["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=puerta",
                         "puntos_sim:=" + PUNTOS], stdout=open(LOG, "w"), stderr=subprocess.STDOUT,
                        preexec_fn=os.setsid)
rclpy.init()
n = Node("prueba_puerta")
est, ver = {}, {}
n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
n.create_subscription(String, "/sim/verdad", lambda m: ver.update(json.loads(m.data)), 1)
cli = n.create_client(Comando, "/go2/comando")
mapas = {}
qmap = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
malos = []


def validar(m, top):
    """Las mismas reglas que nav2_util::validateMsg(OccupancyGrid): si falla, Nav2 lo rechaza."""
    import math
    q = m.info.origin.orientation
    errores = [e for e, mal in (
        ("frame_id vacio", not m.header.frame_id),
        ("nanosec >= 1e9", m.header.stamp.nanosec >= 1000000000),
        ("resolucion <= 0", not m.info.resolution > 1e-6),
        ("ancho o alto 0", m.info.width == 0 or m.info.height == 0),
        ("data != ancho x alto", len(m.data) != m.info.width * m.info.height),
        ("cuaternion no unitario", abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) >= 1e-4),
        ("NaN/inf en el origen", not all(math.isfinite(v) for v in (m.info.origin.position.x, m.info.origin.position.y,
                                                                    m.info.origin.position.z, m.info.resolution))),
    ) if mal]
    if errores and len(malos) < 5:
        malos.append("%s: %s (%dx%d, res %s, frame %r)" % (top, ", ".join(errores), m.info.width, m.info.height,
                                                            m.info.resolution, m.header.frame_id))
    mapas[top] = m


plan = {"p": None}
desvios = []
n.create_subscription(Path, "/plan", lambda m: plan.__setitem__("p", [(q.pose.position.x, q.pose.position.y)
                                                                     for q in m.poses]), 1)


def medir_desvio():
    """Distancia del perro a la linea verde de RViz (/plan): cuanto se sale de la ruta."""
    import math
    pts = plan["p"]
    if not pts or "x" not in ver or len(pts) < 2:
        return
    d = min(math.hypot(px - ver["x"], py - ver["y"]) for px, py in pts)
    if d < 1.5:                     # (un plan viejo de otra meta no cuenta)
        desvios.append(d)


for top in ("/global_costmap/costmap", "/local_costmap/costmap", "/go2/mapa_paredes", "/map"):
    n.create_subscription(OccupancyGrid, top, lambda m, t=top: validar(m, t),
                          qmap if top in ("/go2/mapa_paredes", "/map") else 1)


def en_huella(x, y, v):
    """(x, y) dentro de la huella de Nav2 del perro en la pose v."""
    import math
    from go2_nav.nucleo.config import en_huella as dentro
    c, sn = math.cos(v.get("yaw", 0.0)), math.sin(v.get("yaw", 0.0))
    dx, dy = x - v["x"], y - v["y"]
    return dentro(c * dx + sn * dy, -sn * dx + c * dy)


def ascii_mapa(m, x0=3.9, x1=6.1, y0=3.2, y1=4.9, paso=0.05, pose=None):
    """La zona de la puerta (coordenadas del mundo = odom = map en esta prueba).
    # letal  + inscrito (el centro no puede pasar)  : costo alto  . costo bajo  ? desconocido"""
    import math
    if m is None:
        return "(no llego)"
    W, H, r = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    filas = []
    y = y1
    while y >= y0:
        f = ""
        x = x0
        while x <= x1:
            if abs(x - ver.get("x", -9)) < paso / 2 + 0.01 and abs(y - ver.get("y", -9)) < paso / 2 + 0.01:
                f += "R"
            elif pose is not None and en_huella(x, y, pose):
                i, j = int(math.floor((y - oy) / r)), int(math.floor((x - ox) / r))
                v = m.data[i * W + j] if 0 <= i < H and 0 <= j < W else -1
                f += "X" if v >= 99 else "o"
            else:
                i, j = int(math.floor((y - oy) / r)), int(math.floor((x - ox) / r))
                v = m.data[i * W + j] if 0 <= i < H and 0 <= j < W else -1
                f += "?" if v < 0 else "#" if v >= 100 else "+" if v >= 99 else ":" if v >= 50 else "." if v > 0 else " "
            x += paso
        filas.append(f)
        y -= paso
    return "\n".join(filas)


def orden(o, *a):
    r = Comando.Request()
    r.orden, r.args = o, list(a)
    f = cli.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=15)
    return f.result()


def esperar(s, cond=lambda: False):
    t = time.monotonic()
    while time.monotonic() - t < s and not cond():
        rclpy.spin_once(n, timeout_sec=0.1)


cruzo, t_cruce = False, None
hist_cm = __import__("collections").deque(maxlen=3)        # (verdad, costmap local) cada 0.5 s
try:
    if not cli.wait_for_service(timeout_sec=90):
        print("el gestor no arranco (mira %s)" % LOG)
        sys.exit(2)
    esperar(60, lambda: est.get("pose") is not None and est.get("mapa_m2"))
    esperar(5)
    print("listo: %s" % est.get("motivo"), flush=True)
    if MODO == "meta":
        nav = ActionClient(n, NavigateToPose, "navigate_to_pose")
        nav.wait_for_server(timeout_sec=30)
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = 5.6, 4.7      # el pasillo, pasando la puerta
        g.pose.pose.orientation.w = 1.0
        nav.send_goal_async(g)
        print("meta directa al pasillo (5.6, 4.7)", flush=True)
    else:
        print(orden("explorar", "sin").mensaje, flush=True)
    t0, ult, y_max = time.monotonic(), 0.0, 0.0
    while time.monotonic() - t0 < T_MAX:
        esperar(0.5)
        y_max = max(y_max, ver.get("y", 0.0))
        medir_desvio()
        hist_cm.append((dict(ver), mapas.get("/local_costmap/costmap")))
        if ver.get("choques") and "choque" not in mapas:       # el costmap ~1 s ANTES del primer choque
            mapas["choque"] = hist_cm[0]
        if not cruzo and ver.get("y", 0.0) > Y_PUERTA + 0.3:
            cruzo, t_cruce = True, time.monotonic() - t0
            print("%4.0fs CRUZO LA PUERTA (y=%.2f)" % (t_cruce, ver["y"]), flush=True)
        if time.monotonic() - ult > 10:
            ult = time.monotonic()
            D = est.get("driver") or {}
            print("%4.0fs %-10s verdad x=%s y=%s z=%s | choques %s | orden %s | %s" % (
                time.monotonic() - t0, est.get("modo"), ver.get("x"), ver.get("y"), ver.get("z"),
                ver.get("choques"), D.get("orden"), est.get("motivo")), flush=True)
        if cruzo and time.monotonic() - t0 > t_cruce + 20:
            break
        if MODO != "meta" and est.get("modo") in ("terminado", "quieto", "pausado") and time.monotonic() - t0 > 30:
            break
    r = orden("log")
    print("---- log del gestor\n" + (r.mensaje if r else ""))
    print("choques (x, y, yaw, vx, vy, t, marcha_escaleras): %s" % ver.get("donde"))
    print("mapas que Nav2 rechazaria: %s" % (malos or "ninguno"))
    if desvios:
        ds = sorted(desvios)
        print("DESVIO de la ruta (linea verde): promedio %.2f m, 90%% del tiempo < %.2f m, maximo %.2f m"
              % (sum(ds) / len(ds), ds[int(0.9 * (len(ds) - 1))], ds[-1]))
    esperar(1.0)
    if "choque" in mapas:
        v0, cm = mapas["choque"]
        ver_bak = dict(ver)
        ver.update(v0)
        print("---- /local_costmap/costmap ~1 s ANTES del primer choque (R = perro en x=%.2f y=%.2f yaw=%.2f; "
              "o = huella, X = huella sobre algo inscrito/letal; la puerta va de x 4.6 a 5.32 en y 4.0..4.1)\n%s"
              % (v0["x"], v0["y"], v0["yaw"], ascii_mapa(cm, v0["x"] - 1.1, v0["x"] + 1.1, v0["y"] - 1.0, v0["y"] + 0.8,
                                                         pose=v0)))
        ver.clear()
        ver.update(ver_bak)
    for top in ("/map", "/go2/mapa_paredes", "/global_costmap/costmap", "/local_costmap/costmap"):
        print("---- %s alrededor de la puerta (x 3.9..6.1 de izq a der, y 4.9..3.2 de arriba a abajo; "
              "la puerta va de x 4.6 a 5.32 en y 4.0..4.1)\n%s" % (top, ascii_mapa(mapas.get(top))))
    print("FINAL: cruzo=%s (%s s) y_max=%.2f choques=%s modo=%s" % (
        cruzo, None if t_cruce is None else round(t_cruce), y_max, ver.get("choques"), est.get("modo")))
finally:
    os.killpg(os.getpgid(lanz.pid), signal.SIGINT)
    time.sleep(4)
    try:
        os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
sys.exit(0 if cruzo and not ver.get("choques") else 1)
