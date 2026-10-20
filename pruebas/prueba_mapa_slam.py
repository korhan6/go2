"""El MAPA DEL SLAM con un lidar ralo como el L1 (ROS 2 + slam_toolbox reales, perro simulado): el
perro gira en el cuarto de la puerta, avanza un poco y vuelve a girar; despues se compara /map con
el mundo de verdad.

    python3 pruebas/prueba_mapa_slam.py

Mide:
  - paredes dibujadas: de las paredes del cuarto, cuantas salen NEGRAS en el mapa (a +-10 cm). En
    la casa salian como puntitos sueltos;
  - puntitos falsos: celdas negras a mas de 10 cm de toda pared o mueble;
  - rayos que atraviesan paredes: celdas BLANCAS (libres) donde el perro no puede llegar (detras
    de una pared).
GO2_SIM_MOTAS (obstaculos falsos de un instante) se pasa al simulador.
"""
import json
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

LOG = os.environ.get("PRUEBA_LOG", "/tmp/prueba_mapa_slam.log")
mundo, (X0, Y0, _) = perro_sim.elegir("puerta")

lanz = subprocess.Popen(["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=puerta",
                         "puntos_sim:=2500"], stdout=open(LOG, "w"), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
rclpy.init()
n = Node("prueba_mapa_slam")
est, ver, mapas = {}, {}, {}
n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
n.create_subscription(String, "/sim/verdad", lambda m: ver.update(json.loads(m.data)), 1)
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, "/map", lambda m: mapas.__setitem__("map", m), q)
pub = n.create_publisher(Twist, "cmd_vel_prioridad", 10)


def esperar(s, vx=0.0, wz=0.0):
    t = time.monotonic()
    while time.monotonic() - t < s:
        if vx or wz:
            tw = Twist()
            tw.linear.x, tw.angular.z = vx, wz
            pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)


def alcanzable():
    """Piso al que el perro puede llegar (sin cruzar paredes) desde donde empieza."""
    from scipy import ndimage
    etiquetas, _ = ndimage.label(~mundo.pared)
    i, j = mundo.c(X0, Y0)
    return etiquetas == etiquetas[i, j]


def comparar(m):
    from scipy import ndimage
    W, H, r = m.info.width, m.info.height, m.info.resolution
    d = np.asarray(m.data, np.int16).reshape(H, W)
    jj, ii = np.meshgrid(np.arange(W), np.arange(H))
    x = m.info.origin.position.x + (jj + 0.5) * r
    y = m.info.origin.position.y + (ii + 0.5) * r
    wi = np.clip((y / 0.05).astype(int), 0, mundo.H - 1)
    wj = np.clip((x / 0.05).astype(int), 0, mundo.W - 1)
    # el mapa del SLAM llevado a la rejilla del mundo
    negro = np.zeros_like(mundo.pared)
    blanco = np.zeros_like(mundo.pared)
    negro[wi[d >= 65], wj[d >= 65]] = True
    blanco[wi[(d >= 0) & (d < 25)], wj[(d >= 0) & (d < 25)]] = True
    pared = mundo.pared
    # las paredes del CUARTO (pegadas al piso al que llega el perro, a menos de 4.5 m de donde gira y
    # de este lado de la puerta): con el L1 las ve todas al girar
    yy, xx = np.mgrid[0:mundo.H, 0:mundo.W] * 0.05 + 0.025
    vista = (pared & ndimage.binary_dilation(alcanzable(), iterations=1)
             & (np.hypot(xx - X0, yy - Y0) < 4.5) & (yy < 4.15))
    dibujada = vista & ndimage.binary_dilation(negro, iterations=2)
    falsos = negro & ~ndimage.binary_dilation(pared, iterations=2)
    fuera = blanco & ~ndimage.binary_dilation(alcanzable(), iterations=2)
    return int(vista.sum()), int(dibujada.sum()), int(falsos.sum()), int(fuera.sum()), int(blanco.sum())


res = None
try:
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90 and est.get("pose") is None:
        esperar(0.5)
    esperar(6)
    print("girando, avanzando y girando otra vez...", flush=True)
    esperar(30, wz=0.4)
    esperar(4, vx=0.2)
    esperar(20, wz=-0.4)
    esperar(5)
    if "map" not in mapas:
        print("FINAL: no llego /map")
    else:
        m = mapas["map"]
        if os.environ.get("GUARDAR_MAPA"):
            np.savez(os.environ["GUARDAR_MAPA"], d=np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width),
                     o=[m.info.origin.position.x, m.info.origin.position.y, m.info.resolution])
        vista, dib, falsos, fuera, libres = comparar(m)
        res = (vista, dib, falsos, fuera)
        print("FINAL: paredes dibujadas %d de %d celdas vistas (%.0f %%) | puntitos falsos %d | rayos que "
              "atraviesan paredes %d celdas (de %d libres) | choques %s"
              % (dib, vista, 100.0 * dib / max(1, vista), falsos, fuera, libres, ver.get("choques")), flush=True)
finally:
    os.killpg(os.getpgid(lanz.pid), signal.SIGINT)
    time.sleep(4)
    try:
        os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
sys.exit(0 if res and res[1] >= 0.6 * res[0] else 1)
