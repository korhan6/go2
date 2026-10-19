"""Prueba de punta a punta: lanza todo con el perro simulado, ordena 'explorar' y mide.

    python3 prueba_e2e.py pasillo 300
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_nav_interfaces.srv import Comando

mundo = sys.argv[1] if len(sys.argv) > 1 else "pasillo"
t_max = float(sys.argv[2]) if len(sys.argv) > 2 else 300
log = open("/tmp/e2e_%s.log" % mundo, "w")
lanz = subprocess.Popen(["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=" + mundo],
                        stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
rclpy.init()
n = Node("prueba_e2e")
est, verdad = {}, {}
n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
n.create_subscription(String, "/sim/verdad", lambda m: verdad.update(json.loads(m.data)), 1)
cli = n.create_client(Comando, "/go2/comando")


def orden(o, *a):
    r = Comando.Request()
    r.orden, r.args = o, list(a)
    f = cli.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    return f.result()


def esperar(s):
    t = time.monotonic()
    while time.monotonic() - t < s:
        rclpy.spin_once(n, timeout_sec=0.1)


try:
    cli.wait_for_service(timeout_sec=60)
    # esperar a que haya mapa y TF
    t0 = time.monotonic()
    while time.monotonic() - t0 < 60:
        esperar(1.0)
        if est.get("pose") is not None and est.get("mapa_m2"):
            break
    print("listo en %.0f s: %s" % (time.monotonic() - t0, est.get("motivo")), flush=True)
    print(orden("explorar").mensaje, flush=True)
    t0 = time.monotonic()
    ult = 0
    while time.monotonic() - t0 < t_max:
        esperar(1.0)
        if time.monotonic() - ult > 10:
            ult = time.monotonic()
            L = est.get("limpiador") or {}
            print("%4.0fs %-10s mapa %s m2 pose %s choques %s | %s | lidar %sHz %sms move %s" % (
                time.monotonic() - t0, est.get("modo"), est.get("mapa_m2"), est.get("pose"),
                verdad.get("choques"), est.get("motivo"), L.get("nubes_hz"), L.get("ms_max"),
                (est.get("driver") or {}).get("move_hz")), flush=True)
        if est.get("modo") in ("terminado",) or (est.get("modo") == "quieto" and time.monotonic() - t0 > 20):
            break
    print(orden("volcar").mensaje)
    r = orden("log")
    print("---- log del gestor\n" + (r.mensaje if r else ""))
    print("FINAL: modo=%s mapa=%s m2 choques=%s verdad=%s tiempo=%.0f s" % (
        est.get("modo"), est.get("mapa_m2"), verdad.get("choques"), verdad, time.monotonic() - t0))
finally:
    os.killpg(os.getpgid(lanz.pid), signal.SIGINT)
    time.sleep(3)
    try:
        os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
