"""guardar A -> explorar un rato -> ir A ; y calibrar con fantasmas."""
import json, os, signal, subprocess, sys, time, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from go2_nav_interfaces.srv import Comando
caso = sys.argv[1]
extra = ["fantasmas:=true"] if caso == "calibrar" else []
log = open("/tmp/fun_%s.log" % caso, "w")
lanz = subprocess.Popen(["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=dos_puertas"] + extra,
                        stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
rclpy.init(); n = Node("prueba_fun"); est, ver = {}, {}
n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
n.create_subscription(String, "/sim/verdad", lambda m: ver.update(json.loads(m.data)), 1)
cli = n.create_client(Comando, "/go2/comando")
def orden(o, *a):
    r = Comando.Request(); r.orden, r.args = o, list(a)
    f = cli.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=25); return f.result()
def esperar(s, cond=lambda: False):
    t = time.monotonic()
    while time.monotonic() - t < s and not cond():
        rclpy.spin_once(n, timeout_sec=0.1)
try:
    cli.wait_for_service(timeout_sec=60)
    esperar(30, lambda: est.get("pose") is not None and est.get("mapa_m2"))
    esperar(5)
    if caso == "puntos":
        print(orden("guardar", "A").mensaje); a = dict(ver)
        print(orden("explorar").mensaje); esperar(45)
        print(orden("parar").mensaje); esperar(2)
        print("lejos de A:", round(math.hypot(ver["x"] - a["x"], ver["y"] - a["y"]), 2), "m")
        print(orden("ir", "A").mensaje)
        t0 = time.monotonic(); esperar(90, lambda: est.get("modo") == "quieto" and time.monotonic() - t0 > 3)
        print("modo", est.get("modo"), "|", est.get("motivo"), "| error a A: %.2f m" % math.hypot(ver["x"] - a["x"], ver["y"] - a["y"]), "| choques", ver.get("choques"))
    else:
        L = lambda: est.get("limpiador", {})
        print("ANTES:", {k: L().get(k) for k in ("adelante", "izq", "der", "calibrado")})
        print(orden("calibrar").mensaje)
        esperar(130, lambda: (L().get("calibracion") or "calibrando").split(":")[0] != "calibrando")
        esperar(4)
        print("CALIBRACION:", L().get("calibracion"))
        print("DESPUES:", {k: L().get(k) for k in ("adelante", "izq", "der", "calibrado")})
        print(orden("explorar").mensaje); esperar(60)
        print("explorando 60 s con fantasmas calibrados: mapa", est.get("mapa_m2"), "m2, choques", ver.get("choques"), "|", est.get("motivo"))
finally:
    os.killpg(os.getpgid(lanz.pid), signal.SIGINT); time.sleep(3)
    try: os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
    except ProcessLookupError: pass
