"""La escalera ANGOSTA de la casa (mundo 'escalera' del simulador, con el lidar como el L1 de verdad),
con todo ROS andando como en el perro: 'go2 subir' con el hocico a 13 cm del primer escalon, y
'go2 bajar' desde arriba con una pared 90 cm detras del pie de la escalera.

    python3 pruebas/prueba_escalera_angosta.py [subir|bajar|todo]

Sale con codigo 0 si las dos salieron bien y sin choques.
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

QUE = sys.argv[1] if len(sys.argv) > 1 else "todo"
ARRIBA = "6.39,1.425,3.14159"        # arriba, con el hocico a 60 cm del borde, mirando la bajada


def correr(orden_esc, inicio=None):
    args = ["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=escalera", "puntos_sim:=2500"]
    if inicio:
        args.append("inicio_sim:=" + inicio)
    log = open("/tmp/esc_angosta_%s.log" % orden_esc, "w")
    lanz = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    n = Node("prueba_esc_" + orden_esc)
    est, ver, esc = {}, {}, {}
    n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
    n.create_subscription(String, "/sim/verdad", lambda m: ver.update(json.loads(m.data)), 1)
    n.create_subscription(String, "/go2/escalera_estado", lambda m: esc.update(json.loads(m.data)), 1)
    cli = n.create_client(Comando, "/go2/comando")

    def orden(o, *a):
        r = Comando.Request()
        r.orden, r.args = o, list(a)
        f = cli.call_async(r)
        rclpy.spin_until_future_complete(n, f, timeout_sec=25)
        return f.result()

    def esperar(s, cond=lambda: False, cada=5.0):
        t, u = time.monotonic(), 0.0
        while time.monotonic() - t < s and not cond():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.monotonic() - u > cada:
                u = time.monotonic()
                print("  %4.0fs %-12s z=%s x=%s choques=%s | %s" % (time.monotonic() - t, esc.get("estado"), ver.get("z"),
                      ver.get("x"), ver.get("choques"), esc.get("info") or esc.get("log")), flush=True)

    try:
        cli.wait_for_service(timeout_sec=90)
        esperar(12)
        print(orden("escaleras").mensaje)
        print("%s: %s" % (orden_esc.upper(), orden(orden_esc).mensaje), flush=True)
        esperar(180, lambda: (esc.get("resultado") or {}).get("ok") is not None and not esc.get("activa"))
        esperar(3)
        res = esc.get("resultado") or {}
        print("FINAL %s: ok=%s | %s | z=%s | choques=%s" % (orden_esc, res.get("ok"), res.get("motivo"), ver.get("z"),
                                                           ver.get("choques")), flush=True)
        return bool(res.get("ok")) and not ver.get("choques")
    finally:
        os.killpg(os.getpgid(lanz.pid), signal.SIGINT)
        time.sleep(4)
        try:
            os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        n.destroy_node()


rclpy.init()
ok = True
if QUE in ("subir", "todo"):
    ok &= correr("subir")
if QUE in ("bajar", "todo"):
    ok &= correr("bajar", ARRIBA)
rclpy.try_shutdown()
print("RESULTADO:", "OK" if ok else "FALLA")
sys.exit(0 if ok else 1)
