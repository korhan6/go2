"""Obstaculo en el camino visto de lejos (Nav2 real, perro simulado): el perro tiene que pasarlo
por al lado SIN frenar, y el freno de emergencia (collision_monitor) casi no tiene que actuar.

    python3 pruebas/prueba_obstaculo.py [puntos_por_nube]

Mundo 'simple': cuarto de 8 x 6 m con una caja de 60 x 60 cm en (3.0..3.6, 2.0..2.6). El perro
sale de (1.5, 1.5) y va a (5.5, 2.3): la caja queda en el medio. Sale con codigo 0 si llego sin
chocar.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

PUNTOS = sys.argv[1] if len(sys.argv) > 1 else "2500"
META = (5.5, 2.3)
LOG = os.environ.get("PRUEBA_LOG", "/tmp/prueba_obstaculo.log")

lanz = subprocess.Popen(["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=simple",
                         "inicio_sim:=1.5,1.5,0.3", "puntos_sim:=" + PUNTOS],
                        stdout=open(LOG, "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
rclpy.init()
n = Node("prueba_obstaculo")
est, ver, cm = {}, {}, {"frenos": 0, "lentos": 0, "n": 0}
n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
n.create_subscription(String, "/sim/verdad", lambda m: ver.update(json.loads(m.data)), 1)


def cb_cm(m):
    cm["n"] += 1
    if m.action_type in (1, 3):           # STOP / APPROACH: freno de emergencia actuando
        cm["frenos"] += 1
    elif m.action_type == 2:              # SLOWDOWN: la zona de delante del hocico
        cm["lentos"] += 1


n.create_subscription(CollisionMonitorState, "/collision_monitor_state", cb_cm, 10)


def esperar(s, cond=lambda: False):
    t = time.monotonic()
    while time.monotonic() - t < s and not cond():
        rclpy.spin_once(n, timeout_sec=0.1)


try:
    esperar(60, lambda: est.get("pose") is not None and est.get("mapa_m2"))
    esperar(5)
    nav = ActionClient(n, NavigateToPose, "navigate_to_pose")
    nav.wait_for_server(timeout_sec=30)
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x, g.pose.pose.position.y = META      # (mundo = odom = map en el simulador)
    g.pose.pose.orientation.w = 1.0
    t0 = time.monotonic()
    nav.send_goal_async(g)
    llego, ult = False, 0.0
    while time.monotonic() - t0 < 90:
        esperar(0.5)
        if math.hypot(ver.get("x", 0) - META[0], ver.get("y", 0) - META[1]) < 0.45:
            llego = True
            break
        if time.monotonic() - ult > 10:
            ult = time.monotonic()
            print("%4.0fs verdad x=%s y=%s choques=%s frenos=%d lentos=%d" % (
                time.monotonic() - t0, ver.get("x"), ver.get("y"), ver.get("choques"), cm["frenos"], cm["lentos"]),
                flush=True)
    t = time.monotonic() - t0
    print("FINAL: llego=%s en %.0f s | choques=%s | freno de emergencia actuando %d de %d ciclos | zona lenta %d"
          % (llego, t, ver.get("choques"), cm["frenos"], cm["n"], cm["lentos"]), flush=True)
finally:
    os.killpg(os.getpgid(lanz.pid), signal.SIGINT)
    time.sleep(4)
    try:
        os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
sys.exit(0 if llego and not ver.get("choques") else 1)
