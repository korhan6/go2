"""go2 - ordenes para go2_nav (ROS 2).

  go2 explorar          calibra los fantasmas (2 vueltas) y mapea todo solo (todos los pisos) y vuelve
  go2 explorar sin      igual, sin las 2 vueltas iniciales (los fantasmas igual los aprende caminando)
  go2 manejar           MAPEO A MANO: manejas al perro con el teclado (w a s d) mientras arma el mapa;
                        despues: go2 guardar A, lo llevas a otro lado, go2 guardar B, y go2 ir A / go2 ir B
  go2 ver               estado en vivo, una linea por segundo (Ctrl+C sale)
  go2 estado            estado completo
  go2 guardar A         guarda donde esta el perro como punto A
  go2 ir A              va al punto A (aunque este en otro piso)    go2 casa    vuelve al inicio
  go2 puntos            lista los puntos     go2 borrar A
  go2 pausa | seguir | parar
  go2 escaleras         escaleras vistas (portales), candidatas, y QUE VE EL LIDAR delante y por que
                        (no) lo toma por escalera de bajada
  go2 escaleras al_final|inmediato|off
                        al_final: termina el piso y despues toma las escaleras (por defecto)
                        inmediato: apenas ve una escalera, la toma | off: nunca
  go2 escaleras vision subida|requerida|off
                        subida (por defecto): la camara confirma solo las SUBIDAS; las bajadas las
                        confirma el lidar viendo el primer escalon (la camara no ve una bajada
                        desde arriba) | requerida: la camara confirma todo | off: solo el lidar
  go2 subir | bajar     toma a mano la escalera que tiene delante (subir = de frente;
                        bajar = gira 180 y baja SIEMPRE en reversa)
  go2 probar_mov        mide si el perro HACE lo que se le pide (gira 3 s y avanza 2 s; 1 m libre delante)
  go2 calibrar_mov      mide la respuesta real del perro (adelante/atras y giros, ~40 s, 1 m libre delante
                        y detras) y la COMPENSA: pide lo necesario para que haga lo que Nav2 quiere
  go2 camara            lo que ve la camara en vivo (YOLO en la GPU): fps, objetos, escalera
  go2 vision_probar [foto.jpg|camara]   prueba la GPU y el modelo sin ROS
  MAPEAR A MANO Y NAVEGAR SOLO (recomendado):
    go2 lanzar rviz:=true        1) mapear: manejalo con 'go2 manejar' o con el control remoto
    go2 guardar A                   por toda la casa, guardando puntos donde quieras ir
    go2 guardar_mapa casa        2) guardar el mapa (y los puntos)
    go2 lanzar mapa:=casa rviz:=true   3) otro dia: navegar en ese mapa (no lo cambia)
    go2 ir A | go2 casa | "2D Goal Pose" en RViz
  go2 mapas             mapas guardados
  go2 calibrar          gira 2 vueltas y aprende los puntos fantasma del propio perro
  go2 levantar          pone al perro de pie
  go2 log               ultimos mensajes
  go2 costmap [global]  lo que Nav2 ve alrededor del perro, en texto (3 x 3 m): para mandar en una foto
  go2 nav2 [segundos]   junta los avisos y errores de Nav2 (20 s) y los resume: por que no avanza
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_nav_interfaces.srv import Comando


def cam(e):
    V = e.get("vision")
    if not V:
        return "[camara: -]"
    E = V.get("escalera") or {}
    obj = " ".join(e.get("vision_objetos") or [])
    return "[camara %sfps%s%s]" % (V.get("fps", "?"), " ESCALERA %.0f%%" % (100 * E.get("conf", 0)) if E.get("vista") else "",
                                    (": " + obj) if obj else "")


def camara(nodo):
    print("camara + YOLO en vivo (Ctrl+C sale). Imagen: RViz 'Camara (YOLO)' o ~/.go2_nav/vision_ultima.jpg")
    ultimo = {"v": None}
    nodo.create_subscription(String, "/go2/vision", lambda m: ultimo.__setitem__("v", m.data), 1)
    t, t_ult = 0.0, time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(nodo, timeout_sec=0.1)
            if time.monotonic() - t < 1.0:
                continue
            t = time.monotonic()
            if ultimo["v"] is None:
                if time.monotonic() - t_ult > 5:
                    print("no llega /go2/vision: el nodo de vision no corre (go2 lanzar ...; revisa la terminal)")
                    t_ult = time.monotonic()
                continue
            v = json.loads(ultimo["v"])
            ultimo["v"] = None
            t_ult = time.monotonic()
            E = v.get("escalera") or {}
            objs = ", ".join("%s %.0f%%%s" % (o["nombre"], 100 * o["conf"],
                                               " %.1fm" % o["dist"] if o.get("dist") else "") for o in v.get("objetos", [])[:8])
            if v.get("error"):
                print("%s !! ERROR en la vision: %s" % (time.strftime("%H:%M:%S"), v["error"]), flush=True)
            print("%s %s %sfps (%sms) %s | %s%s" % (
                time.strftime("%H:%M:%S"), v.get("fuente"), v.get("fps", "?"), v.get("ms_max", "?"), v.get("gpu", ""),
                ("ESCALERA %.0f%% a %.0f grados%s | " % (100 * E.get("conf", 0), math.degrees(E.get("rumbo", 0)),
                                                         " %.1f m" % E["dist"] if E.get("dist") else "")) if E.get("vista") else "",
                objs or "(nada)"), flush=True)
    except KeyboardInterrupt:
        print()


def manejar(nodo):
    """Teleoperacion por teclado: el SLAM sigue mapeando mientras lo manejas."""
    import select
    import termios
    import tty
    from geometry_msgs.msg import Twist
    pub = nodo.create_publisher(Twist, "cmd_vel_prioridad", 10)
    H = {}

    def cb(m):
        try:
            d = json.loads(m.data)
            H.update({k: d.get(k) for k in ("adelante", "atras")})
        except (ValueError, AttributeError):
            pass
    nodo.create_subscription(String, "/go2/limpiador_estado", cb, 1)
    cli = nodo.create_client(Comando, "/go2/comando")
    if cli.wait_for_service(timeout_sec=3.0):
        r = Comando.Request()
        r.orden = "parar"           # que el gestor no mande metas mientras manejas
        f = cli.call_async(r)
        rclpy.spin_until_future_complete(nodo, f, timeout_sec=5.0)
    print(__doc__.split("\n")[0])
    print("MANEJAR (manten apretada la tecla; al soltar se para). Tambien puedes usar el control remoto.\n"
          "   w = adelante   s = atras   a / d = girar   q / e = de lado\n"
          "   1 / 2 / 3 = velocidad lenta / normal / rapida     espacio = parar    x = salir\n"
          "   En OTRA terminal:  go2 guardar A   (donde este parado)  ...  go2 ir A\n"
          "   Frena solo si tiene algo a menos de ~35 cm delante/atras (igual: miralo).")
    vel = {"1": (0.25, 0.5), "2": (0.40, 0.8), "3": (0.55, 1.0)}
    nivel = "2"
    teclas = {"w": (1, 0, 0), "s": (-0.6, 0, 0), "a": (0, 0, 1), "d": (0, 0, -1),
              "q": (0, 0.6, 0), "e": (0, -0.6, 0)}
    fd = sys.stdin.fileno()
    viejo = termios.tcgetattr(fd)
    orden, t_tecla, t_print = (0.0, 0.0, 0.0), 0.0, 0.0
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                k = sys.stdin.read(1).lower()
                if k in ("x", "\x03", "\x1b"):
                    break
                if k in vel:
                    nivel = k
                elif k in teclas:
                    v, w = vel[nivel]
                    a, b, c = teclas[k]
                    orden, t_tecla = (a * v, b * v, c * w), time.monotonic()
                elif k == " ":
                    orden = (0.0, 0.0, 0.0)
            if time.monotonic() - t_tecla > 0.6:       # tecla suelta
                orden = (0.0, 0.0, 0.0)
            vx, vy, wz = orden
            aviso = ""
            if vx > 0 and H.get("adelante") is not None and H["adelante"] < 0.40:
                vx, aviso = 0.0, "  ALGO DELANTE: no avanzo"
            if vx < 0 and H.get("atras") is not None and H["atras"] < 0.30:
                vx, aviso = 0.0, "  ALGO DETRAS: no retrocedo"
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.angular.z = vx, vy, wz
            pub.publish(tw)
            rclpy.spin_once(nodo, timeout_sec=0.0)
            if time.monotonic() - t_print > 0.5:
                t_print = time.monotonic()
                sys.stdout.write("\r  vel %s  orden x=%.2f y=%.2f giro=%.2f  libre adel=%s atras=%s%s          "
                                 % (nivel, vx, vy, wz, H.get("adelante", "-"), H.get("atras", "-"), aviso))
                sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, viejo)
        # con Ctrl+C rclpy ya cerro su contexto y no se puede publicar: el driver para solo al
        # perro si no le llegan ordenes en 0.4 s
        try:
            for _ in range(5):
                pub.publish(Twist())
                time.sleep(0.05)
        except Exception:
            pass
        print("\nlisto. Para navegar: go2 ir A")
    return 0


def probar_mov(nodo):
    """Manda giros y avances conocidos y mide con la odometria cuanto hizo de verdad el perro."""
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    od = {}

    def cb(m):
        q = m.pose.pose.orientation
        od["p"] = (m.pose.pose.position.x, m.pose.pose.position.y,
                   math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    nodo.create_subscription(Odometry, "/odom", cb, 10)
    pub = nodo.create_publisher(Twist, "cmd_vel_prioridad", 10)
    cli = nodo.create_client(Comando, "/go2/comando")
    if cli.wait_for_service(timeout_sec=3.0):
        r = Comando.Request()
        r.orden = "parar"
        f = cli.call_async(r)
        rclpy.spin_until_future_complete(nodo, f, timeout_sec=5.0)
    t = time.monotonic()
    while "p" not in od and time.monotonic() - t < 5:
        rclpy.spin_once(nodo, timeout_sec=0.1)
    if "p" not in od:
        print("no llega /odom: go2_nav no esta corriendo o el perro no manda odometria")
        return 1
    print("prueba de movimiento: deja ~1 m libre delante y alrededor. Empieza en 3 s (Ctrl+C cancela)")
    t = time.monotonic()
    while time.monotonic() - t < 3:
        rclpy.spin_once(nodo, timeout_sec=0.1)

    def mandar(vx, wz, dur):
        p0, giro, yaw_a = od["p"], 0.0, od["p"][2]
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            pub.publish(tw)
            rclpy.spin_once(nodo, timeout_sec=0.05)
            y = od["p"][2]
            giro += math.atan2(math.sin(y - yaw_a), math.cos(y - yaw_a))
            yaw_a = y
        pub.publish(Twist())
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
            pub.publish(Twist())
            rclpy.spin_once(nodo, timeout_sec=0.05)
        return math.hypot(od["p"][0] - p0[0], od["p"][1] - p0[1]), giro
    _, giro = mandar(0.0, 0.6, 3.0)
    pedido = 0.6 * 3.0
    print("GIRO:    pedi %.0f grados, giro %.0f grados  (%.0f %%)" % (math.degrees(pedido), math.degrees(abs(giro)),
                                                                   100 * abs(giro) / pedido))
    d, _ = mandar(0.25, 0.0, 2.0)
    print("AVANCE:  pedi %.2f m, avanzo %.2f m  (%.0f %%)" % (0.5, d, 100 * d / 0.5))
    if abs(giro) / pedido < 0.8 or d / 0.5 < 0.8:
        print("\n!! El perro hace menos de lo que se le pide. Corre 'go2 calibrar_mov' para medirlo y\n"
              "   compensarlo. Si da menos del 30 %, revisa el perro (la app abierta, otro programa\n"
              "   mandandole ordenes o la CPU del perro saturada: ver README).")
    else:
        print("\nOK: el perro obedece las ordenes de velocidad.")
    return 0


def calibrar_mov(nodo):
    """Mide cuanto avanza/gira de verdad el perro para varias ordenes y guarda la curva
    (~/.go2_nav/movimiento.json). El driver la usa para compensar."""
    import os
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    od = {}

    def cb(m):
        q = m.pose.pose.orientation
        od["p"] = (m.pose.pose.position.x, m.pose.pose.position.y,
                   math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    nodo.create_subscription(Odometry, "/odom", cb, 10)
    pub = nodo.create_publisher(Twist, "cmd_vel_prioridad", 10)
    pub_m = nodo.create_publisher(Bool, "/go2/midiendo_mov", 10)
    cli = nodo.create_client(Comando, "/go2/comando")
    if cli.wait_for_service(timeout_sec=3.0):
        r = Comando.Request()
        r.orden = "parar"
        f = cli.call_async(r)
        rclpy.spin_until_future_complete(nodo, f, timeout_sec=5.0)
    t = time.monotonic()
    while "p" not in od and time.monotonic() - t < 5:
        rclpy.spin_once(nodo, timeout_sec=0.1)
    if "p" not in od:
        print("no llega /odom: go2_nav no esta corriendo")
        return 1
    print("calibracion de movimiento (~40 s): avanza y retrocede hasta ~0.8 m y gira en el sitio.\n"
          "Deja ~1 m libre DELANTE y DETRAS. Empieza en 4 s (Ctrl+C cancela)")

    def esperar(dur, tw=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            pub_m.publish(Bool(data=True))
            pub.publish(tw if tw is not None else Twist())
            rclpy.spin_once(nodo, timeout_sec=0.05)

    def medir(vx, wz, dur, desde=1.0):
        """velocidad real en regimen (sin el arranque): de 'desde' s al final"""
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        t0 = time.monotonic()
        p_ini, yaw_a, giro = None, None, 0.0
        while time.monotonic() - t0 < dur:
            pub_m.publish(Bool(data=True))
            pub.publish(tw)
            rclpy.spin_once(nodo, timeout_sec=0.05)
            if time.monotonic() - t0 >= desde:
                if p_ini is None:
                    p_ini, yaw_a, t_ini = od["p"], od["p"][2], time.monotonic()
                y = od["p"][2]
                giro += math.atan2(math.sin(y - yaw_a), math.cos(y - yaw_a))
                yaw_a = y
        dt = time.monotonic() - t_ini
        p = od["p"]
        d = math.hypot(p[0] - p_ini[0], p[1] - p_ini[1])
        esperar(0.8)
        return d / dt, abs(giro) / dt

    try:
        esperar(4.0)
        curva_v, curva_w = [], []
        for c in (0.12, 0.20, 0.30, 0.45):
            dur = 2.6 if c < 0.4 else 2.2
            ad, _ = medir(c, 0.0, dur)
            at, _ = medir(-c, 0.0, dur)
            real = (ad + at) / 2
            curva_v.append([c, round(real, 3)])
            print("  avance  pedi %.2f m/s -> hace %.2f m/s  (%3.0f %%)" % (c, real, 100 * real / c))
        for i, c in enumerate((0.30, 0.50, 0.80, 1.10)):
            _, real = medir(0.0, c if i % 2 == 0 else -c, 3.0)
            curva_w.append([c, round(real, 3)])
            print("  giro    pedi %.2f rad/s -> hace %.2f rad/s (%3.0f %%)" % (c, real, 100 * real / c))
    except KeyboardInterrupt:
        print("cancelado")
        return 1
    finally:
        for _ in range(10):
            pub.publish(Twist())
            pub_m.publish(Bool(data=False))
            time.sleep(0.05)
    ruta = os.path.expanduser("~/.go2_nav/movimiento.json")
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    with open(ruta, "w") as f:
        json.dump({"v": curva_v, "w": curva_w, "fecha": time.strftime("%Y-%m-%d %H:%M")}, f)
    print("guardado en %s: el driver ya compensa (no hace falta reiniciar).\n"
          "Comprueba con: go2 probar_mov  (deberia dar ~90-100 %%)" % ruta)
    return 0


def costmap(nodo, args):
    """Dibuja el mapa de costos de Nav2 alrededor del perro (texto, para una foto)."""
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    cual = "global" if args and args[0] == "global" else "local"
    top = "/%s_costmap/costmap" % cual
    marco = "map" if cual == "global" else "odom"
    tfb = Buffer()
    TransformListener(tfb, nodo)
    m = {}
    nodo.create_subscription(OccupancyGrid, top, lambda msg: m.__setitem__("c", msg), 1)
    t = time.monotonic()
    pose = None
    while time.monotonic() - t < 8.0 and ("c" not in m or pose is None):
        rclpy.spin_once(nodo, timeout_sec=0.1)
        try:
            tr = tfb.lookup_transform(marco, "base_link", rclpy.time.Time())
            q = tr.transform.rotation
            pose = (tr.transform.translation.x, tr.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:
            pass
    if "c" not in m or pose is None:
        print("no llega %s o el TF %s->base_link (go2 lanzar corriendo?)" % (top, marco))
        return 1
    c = m["c"]
    W, H, r = c.info.width, c.info.height, c.info.resolution
    ox, oy = c.info.origin.position.x, c.info.origin.position.y
    x0, y0, yaw = pose
    cy, sy = math.cos(yaw), math.sin(yaw)
    print("%s alrededor del perro (arriba = hacia donde MIRA el perro; 5 cm por letra, 3 x 3 m)" % top)
    print("  # obstaculo   + el centro del perro no puede estar ahi   : costo alto   . bajo   ? sin ver")
    print("  o huella del perro   ^ hocico")
    paso, L = 0.05, 1.5
    for k in range(int(2 * L / paso), -1, -1):
        fila = ""
        for q in range(int(2 * L / paso) + 1):
            a = k * paso - L          # hacia adelante del perro
            b = L - q * paso          # hacia la izquierda
            x, y = x0 + cy * a - sy * b, y0 + sy * a + cy * b
            if abs(b) <= 0.17 and -0.38 <= a <= 0.46 and (abs(abs(b) - 0.16) < 0.026 or abs(a - 0.45) < 0.026
                                                           or abs(a + 0.37) < 0.026):
                fila += "^" if abs(a - 0.45) < 0.026 and abs(b) < 0.03 else "o"
                continue
            i, j = int(math.floor((y - oy) / r)), int(math.floor((x - ox) / r))
            v = c.data[i * W + j] if 0 <= i < H and 0 <= j < W else -1
            fila += "?" if v < 0 else "#" if v >= 100 else "+" if v >= 99 else ":" if v >= 50 else "." if v > 0 else " "
        print(fila)
    return 0


def nav2_avisos(nodo, args):
    """Resume los WARN/ERROR de Nav2 (y del SLAM) de los proximos segundos, con que significan."""
    from rcl_interfaces.msg import Log
    dur = float(args[0]) if args else 20.0
    cuenta = {}
    nodo.create_subscription(Log, "/rosout", lambda m: cuenta.__setitem__(
        (m.name, m.msg[:110]), cuenta.get((m.name, m.msg[:110]), 0) + 1) if m.level >= 30 else None, 100)
    print("escuchando los avisos de Nav2 %.0f s (deja al perro intentando lo que falla)..." % dur)
    t = time.monotonic()
    while time.monotonic() - t < dur:
        rclpy.spin_once(nodo, timeout_sec=0.1)
    significado = [
        ("Optimizer fail", "el controlador no encuentra NINGUNA trayectoria sin choque (la huella ya toca un obstaculo, "
                           "o hay obstaculos fantasma pegados al cuerpo): mira 'go2 costmap'"),
        ("Failed to create plan", "el planificador no encuentra ruta hasta la meta (puerta cerrada en el mapa de costos): "
                                  "mira 'go2 costmap global'"),
        ("Collision Ahead", "una maniobra de recuperacion (retroceder) no se hizo porque hay algo detras"),
        ("patience exceeded", "el controlador se rindio despues de fallar un rato seguido"),
        ("Failed to make progress", "el perro no se movio lo suficiente en 15 s (pisa en el sitio o esta trabado)"),
        ("malformed", "un mapa que llego a Nav2 estaba mal armado (avisame)"),
        ("extrapolation", "el TF llega atrasado (lidar cortado o PC lenta)"),
        ("missed its desired rate", "la PC no alcanza a calcular a tiempo (CPU)"),
    ]
    if not cuenta:
        print("sin avisos de Nav2 en %.0f s" % dur)
        return 0
    for (nombre, msg), k in sorted(cuenta.items(), key=lambda kv: -kv[1])[:15]:
        que = next((d for clave, d in significado if clave.lower() in msg.lower()), "")
        print("%4dx [%s] %s%s" % (k, nombre, msg, ("\n        -> " + que) if que else ""))
    return 0


def avisos(L, D):
    a = []
    if L and L.get("calibrado") is False:
        a.append("SIN CALIBRAR (go2 calibrar)")
    if D and D.get("evitacion_unitree"):
        a.append("EVASION UNITREE PRENDIDA")
    if D and D.get("lidar_mudo_s"):
        a.append("LIDAR CORTADO %.1f s (cable/bateria): quieto" % D["lidar_mudo_s"])
    elif D and D.get("pausas_lidar"):
        a.append("lidar con cortes")
    if D and D.get("ordenes_minimas"):
        a.append("ordenes minimas=%d" % D["ordenes_minimas"])
    return ("  !! " + " | ".join(a)) if a else ""


def escalera_delante(e):
    """Hay una caida delante: que dice el lidar de ella (por que no es / es escalera)."""
    d = e.get("escalera_delante")
    return ("\n         >> bajada delante: " + d) if d else ""


def linea(e):
    L = e.get("limpiador") or {}
    D = e.get("driver") or {}
    p = e.get("pose") or [0, 0, 0]
    f = lambda v: "-" if v is None else ("%.2f" % v if isinstance(v, float) else str(v))
    return ("%s %-10s piso %s x=%5.2f y=%5.2f %4.0f°  mapa %s m2  adel=%s izq=%s der=%s atras=%s  "
            "[lidar %sHz %sms | odom %sHz move %sHz orden %s] fantasmas=%s%s %s | %s"
            % (time.strftime("%H:%M:%S"), e.get("modo"), e.get("piso"), p[0], p[1], p[2] * 57.3, f(e.get("mapa_m2")),
               f(L.get("adelante")), f(L.get("izq")), f(L.get("der")), f(L.get("atras")),
               f(L.get("nubes_hz")), f(L.get("ms_max")), f(D.get("odom_hz")), f(D.get("move_hz")),
               ",".join("%.2f" % v for v in (D.get("orden") or [])) or "-",
               "%s (aprendidos %s)" % (f(L.get("fantasmas")), f(L.get("fantasmas_aprendidos"))), avisos(L, D), cam(e),
               e.get("motivo", "") + (("  [" + L["calibracion"] + "]") if L.get("calibracion") else "")
               + escalera_delante(e)))


def ver(nodo):
    print("go2_nav en vivo (Ctrl+C para salir)")
    ultimo = {"e": None, "log": ""}
    nodo.create_subscription(String, "/go2/estado", lambda m: ultimo.__setitem__("e", m.data), 1)
    t = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(nodo, timeout_sec=0.1)
            if time.monotonic() - t >= 1.0 and ultimo["e"]:
                t = time.monotonic()
                e = json.loads(ultimo["e"])
                print(linea(e), flush=True)
                if e.get("ultimo_log") and e["ultimo_log"] != ultimo["log"]:
                    ultimo["log"] = e["ultimo_log"]
                    print("         >> " + ultimo["log"], flush=True)
    except KeyboardInterrupt:
        print()


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help", "ayuda"):
        print(__doc__)
        return 0
    rclpy.init()
    nodo = Node("go2_cli")
    try:
        if args[0] == "ver":
            ver(nodo)
            return 0
        if args[0] == "camara":
            camara(nodo)
            return 0
        if args[0] == "probar_mov":
            return probar_mov(nodo)
        if args[0] == "manejar":
            return manejar(nodo)
        if args[0] == "calibrar_mov":
            return calibrar_mov(nodo)
        if args[0] == "costmap":
            return costmap(nodo, args[1:])
        if args[0] == "nav2":
            return nav2_avisos(nodo, args[1:])
        cli = nodo.create_client(Comando, "/go2/comando")
        if not cli.wait_for_service(timeout_sec=5.0):
            print("go2_nav no esta corriendo. Arrancalo con: ros2 launch go2_nav go2.launch.py")
            return 1
        req = Comando.Request()
        req.orden = args[0].replace("-", "_")
        req.args = args[1:]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(nodo, fut, timeout_sec=40.0 if req.orden == "guardar_mapa" else 10.0)
        r = fut.result()
        if r is None:
            print("sin respuesta")
            return 1
        if args[0] == "estado" and r.ok:
            e = json.loads(r.json)
            print(linea(e))
            print(json.dumps(e, indent=1, ensure_ascii=False))
        else:
            print(r.mensaje)
        return 0 if r.ok else 1
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
