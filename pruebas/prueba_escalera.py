"""Subir y bajar la escalera de la casa simulada a mano: go2 subir / go2 bajar.

    python3 prueba_escalera.py
"""
import json, math, os, signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from go2_nav_interfaces.srv import Comando

log = open("/tmp/esc.log", "w")
lanz = subprocess.Popen(["ros2", "launch", "go2_nav", "go2.launch.py", "sim:=true", "mundo:=casa",
                         "inicio_sim:=7.75,6.6,0.0"], stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
rclpy.init(); n = Node("prueba_esc"); est, ver = {}, {}
n.create_subscription(String, "/go2/estado", lambda m: est.update(json.loads(m.data)), 1)
n.create_subscription(String, "/sim/verdad", lambda m: ver.update(json.loads(m.data)), 1)
cli = n.create_client(Comando, "/go2/comando")


def orden(o, *a):
    r = Comando.Request(); r.orden, r.args = o, list(a)
    f = cli.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=25); return f.result()


def esperar(s, cond=lambda: False, cada=5.0):
    t = time.monotonic(); u = 0
    while time.monotonic() - t < s and not cond():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() - u > cada:
            u = time.monotonic()
            print("  %4.0fs piso %s z=%s pose=%s | %s | %s" % (time.monotonic() - t, est.get("piso"), ver.get("z"),
                  (ver.get("x"), ver.get("y"), ver.get("yaw")), est.get("escalera"), est.get("motivo")), flush=True)


try:
    cli.wait_for_service(timeout_sec=60)
    esperar(15)
    print("SUBIR:", orden("subir").mensaje)
    esperar(150, lambda: est.get("motivo", "").startswith("escalera") is False and est.get("piso") == 1 and est.get("escalera") is None)
    esperar(6)
    print("despues de subir: piso", est.get("piso"), "z", ver.get("z"), "choques", ver.get("choques"))
    # de frente al borde de la bajada (el perro quedo mirando +x en el entrepiso): media vuelta
    print("escaleras conocidas:\n" + orden("escaleras").mensaje)
    print("ir a la escalera de bajada: go2 casa (inicio esta en el piso 0)")
    print(orden("casa").mensaje)
    esperar(240, lambda: est.get("modo") in ("terminado", "quieto") and est.get("piso") == 0 and est.get("escalera") is None)
    print("FINAL: modo", est.get("modo"), "| piso", est.get("piso"), "| z", ver.get("z"), "| choques", ver.get("choques"),
          "|", est.get("motivo"))
    r = orden("log"); print("---- log\n" + (r.mensaje if r else ""))
finally:
    os.killpg(os.getpgid(lanz.pid), signal.SIGINT); time.sleep(3)
    try:
        os.killpg(os.getpgid(lanz.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
