"""Piso de ceramica BRILLANTE (mundo 'brillante': mesa delante, puerta al costado), con todo ROS: el
perro gira despacio en el sitio mirando todo el cuarto, y se cuentan las celdas LETALES de los
costmaps donde en realidad hay piso libre. Antes el reflejo del piso (la mesa y las paredes espejadas
debajo del suelo) llenaba el mapa de "caidas" y RViz mostraba al perro "sin espacio".

    python3 pruebas/prueba_brillo.py [segundos_girando]

Sale con codigo 0 si no quedan celdas letales falsas en el piso.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "go2_nav"))
from go2_nav import perro_sim  # noqa: E402

T_GIRO = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
LOG = os.environ.get("PRUEBA_LOG", "/tmp/prueba_brillo.log")
mundo, _ = perro_sim.elegir("brillante")
mundo.preparar()

lanz = subprocess.Popen(["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=brillante",
                         "puntos_sim:=2500"], stdout=open(LOG, "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
rclpy.init()
n = Node("prueba_brillo")
est, ver, mapas = {}, {}, {}
n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
n.create_subscription(String, "/sim/verdad", lambda m: ver.update(json.loads(m.data)), 1)
n.create_subscription(String, "/go2/limpiador_estado", lambda m: est.__setitem__("limp", json.loads(m.data)), 1)
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
for top in ("/global_costmap/costmap", "/local_costmap/costmap"):
    n.create_subscription(OccupancyGrid, top, lambda m, t=top: mapas.__setitem__(t, m), 1)
pub = n.create_publisher(Twist, "cmd_vel_prioridad", 10)


def esperar(s, cond=lambda: False, girar=0.0):
    t = time.monotonic()
    while time.monotonic() - t < s and not cond():
        if girar:
            tw = Twist()
            tw.angular.z = girar
            pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)


def falsas(m):
    """Celdas letales (100) en piso de verdad: ni pared ni pata de mesa a menos de 10 cm, y a la
    vista del perro (a menos de 3 m)."""
    W, H, r = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    d = np.array(m.data, np.int16).reshape(H, W)
    ii, jj = np.nonzero(d >= 100)
    malas = 0
    for i, j in zip(ii, jj):
        x, y = ox + (j + 0.5) * r, oy + (i + 0.5) * r
        if math.hypot(x - 1.2, y - 2.6) > 3.0:
            continue
        cerca = any(mundo.es_pared(x + dx, y + dy) for dx in (-0.1, 0, 0.1) for dy in (-0.1, 0, 0.1))
        if not cerca:
            malas += 1
    return malas


res = {}
try:
    esperar(60, lambda: est.get("pose") is not None)
    esperar(8)
    print("girando %.0f s en el sitio..." % T_GIRO, flush=True)
    esperar(T_GIRO, girar=0.6)
    esperar(3)
    res = {t: falsas(m) for t, m in mapas.items()}
    L = est.get("limp") or {}
    print("FINAL: celdas letales FALSAS en el piso: global %s, local %s | reflejos quitados en la ultima nube: %s"
          " | caidas marcadas: %s | choques %s" % (res.get("/global_costmap/costmap"), res.get("/local_costmap/costmap"),
                                                   L.get("reflejos"), L.get("caidas"), ver.get("choques")), flush=True)
finally:
    os.killpg(os.getpgid(lanz.pid), signal.SIGINT)
    time.sleep(4)
    try:
        os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
sys.exit(0 if res and not any(res.values()) else 1)
