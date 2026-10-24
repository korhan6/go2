"""go2_vision: vision por computadora con la camara del Go2, en la GPU de la laptop.

    camara del perro --(cable)--> laptop --> YOLO (GPU, CUDA) --> que hay y DONDE esta

Camara (por el cable, sin WebRTC):
  - "multicast": video H.264 que el perro manda a 230.1.1.1:1720 (30 fps). Lo decodifica GStreamer.
  - "videohub":  fotos JPEG pedidas por DDS al servicio videohub del perro (api 1001). Plan B.
  - "auto" (por defecto): prueba multicast y, si en 5 s no llega nada, videohub.

Modelo: YOLOv8 entrenado con Open Images V7 (600 clases, entre ellas "Stairs", "Person", "Chair",
"Door", "Table", "Bed"...). Corre en la GPU NVIDIA con media precision (FP16).

Lo que ve lo ubica en el mapa: la direccion sale de la imagen y la distancia del lidar (los
obstaculos limpios en esa direccion).

Publica:
  /go2/vision          String JSON: fps, fuente, objetos [{clase, conf, rumbo, dist, x, y}], escalera
  /go2/vision/imagen   sensor_msgs/Image con las detecciones dibujadas (para RViz), 10 Hz
  /go2/vision/marcas   MarkerArray: etiquetas de lo que ve, en su lugar del mapa (marco odom)
  ~/.go2_nav/vision_ultima.jpg  la ultima imagen con detecciones (cada 2 s)

Probar sin ROS:  go2 vision_probar [foto.jpg]
"""
import collections
import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

DATOS = os.path.expanduser("~/.go2_nav")
MODELOS = os.path.join(DATOS, "modelos")

ES = {"Stairs": "escalera", "Person": "persona", "Man": "persona", "Woman": "persona", "Boy": "nino",
      "Girl": "nina", "Chair": "silla", "Table": "mesa", "Door": "puerta", "Bed": "cama", "Couch": "sofa",
      "Sofa bed": "sofa", "Dog": "perro", "Cat": "gato", "Television": "tele", "Window": "ventana",
      "Houseplant": "planta", "Plant": "planta", "Bottle": "botella", "Laptop": "laptop",
      "Shelf": "estante", "Cabinetry": "mueble", "Desk": "escritorio", "Ladder": "escalera de mano",
      "Box": "caja", "Bicycle": "bici", "Car": "carro", "Toy": "juguete", "Bag": "bolsa",
      "Backpack": "mochila", "Kitchen appliance": "electrodomestico", "Refrigerator": "nevera",
      "Toilet": "inodoro", "Sink": "lavamanos", "Wheel": "rueda", "Tire": "llanta", "Stool": "banco",
      "Bench": "banca", "Wall clock": "reloj", "Lamp": "lampara", "Pillow": "almohada", "Curtain": "cortina",
      "Mobile phone": "celular", "Ball": "pelota", "Footwear": "zapato", "Fireplace": "chimenea"}
# clases de Open Images que salen en casi todo y no dicen nada
IGNORAR = {"Clothing", "Human face", "Human head", "Human hair", "Human hand", "Human arm", "Human body",
           "Human nose", "Human eye", "Human mouth", "Human ear", "Human leg", "Human foot",
           "Human beard", "Furniture", "Building", "House", "Tree", "Plant", "Floor", "Wall", "Window blind",
           "Land vehicle", "Vehicle", "Fashion accessory", "Tableware", "Food", "Animal", "Mammal",
           "Room", "Interior design", "Flooring", "Wood"}
ESCALERA = "Stairs"


# ------------------------------------------------------------------ camaras
class FuenteMulticast(object):
    """H.264 multicast del Go2 -> frames BGR, con gst-launch en un proceso aparte (sin colas: solo
    el ultimo frame)."""
    nombre = "multicast"

    def __init__(self, iface="", ancho=960, alto=540, grupo="230.1.1.1", puerto=1720):
        self.W, self.H = ancho, alto
        cmd = ["gst-launch-1.0", "-q", "udpsrc", "address=%s" % grupo, "port=%d" % puerto]
        if iface:
            cmd.append("multicast-iface=%s" % iface)
        cmd += ["!", "application/x-rtp,media=video,encoding-name=H264", "!", "rtph264depay", "!",
                "h264parse", "!", "avdec_h264", "!", "queue", "leaky=downstream", "max-size-buffers=1", "!",
                "videoconvert", "!", "videoscale", "!", "video/x-raw,format=BGR,width=%d,height=%d" % (ancho, alto),
                "!", "fdsink", "fd=1", "sync=false"]
        if shutil.which("gst-launch-1.0") is None:
            raise RuntimeError("falta GStreamer (sudo apt install gstreamer1.0-tools gstreamer1.0-libav "
                               "gstreamer1.0-plugins-good gstreamer1.0-plugins-bad)")
        self.p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        self.ultimo, self.t = None, 0.0
        self.n = 0
        self.vivo = True
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        tam = self.W * self.H * 3
        buf = bytearray(tam)
        mv = memoryview(buf)
        while self.vivo:
            k = 0
            while k < tam:
                r = self.p.stdout.readinto(mv[k:])
                if not r:
                    self.vivo = False
                    return
                k += r
            self.ultimo = np.frombuffer(bytes(buf), np.uint8).reshape(self.H, self.W, 3)
            self.t = time.monotonic()
            self.n += 1

    def cerrar(self):
        self.vivo = False
        try:
            self.p.kill()
        except Exception:
            pass


class FuenteVideohub(object):
    """Fotos JPEG del servicio 'videohub' del perro por DDS (GetImageSample, api 1001)."""
    nombre = "videohub"

    def __init__(self, nodo):
        import cv2
        from unitree_api.msg import Request, Response
        self.cv2 = cv2
        self.Request = Request
        self.pub = nodo.create_publisher(Request, "/api/videohub/request", 10)
        nodo.create_subscription(Response, "/api/videohub/response", self._resp, 10)
        self.id = int(time.time() * 1000) % 100000000
        self.esperando = None
        self.ev = threading.Event()
        self.ultimo, self.t, self.n = None, 0.0, 0
        self.vivo = True
        threading.Thread(target=self._lazo, daemon=True).start()

    def _resp(self, m):
        if m.header.identity.api_id != 1001 or m.header.identity.id != self.esperando:
            return
        if m.header.status.code == 0 and len(m.binary):
            img = self.cv2.imdecode(np.frombuffer(m.binary, np.uint8), self.cv2.IMREAD_COLOR)
            if img is not None:
                self.ultimo, self.t = img, time.monotonic()
                self.n += 1
        self.ev.set()

    def _lazo(self):
        while self.vivo:
            r = self.Request()
            self.id += 1
            self.esperando = self.id
            r.header.identity.id = self.id
            r.header.identity.api_id = 1001
            r.header.policy.noreply = False
            r.parameter = ""
            self.ev.clear()
            self.pub.publish(r)
            if not self.ev.wait(0.5):
                time.sleep(0.2)

    def cerrar(self):
        self.vivo = False


# ------------------------------------------------------------------ modelo
class Detector(object):
    def __init__(self, modelo="", imgsz=640, conf=0.25, dispositivo="auto"):
        import torch
        from ultralytics import YOLO
        cuda = torch.cuda.is_available() and dispositivo != "cpu"
        if not cuda:
            # sin GPU: 2 hilos y prioridad baja. La navegacion (Nav2, SLAM, limpiador) va primero
            torch.set_num_threads(2)
        self.dev = 0 if cuda else "cpu"
        self.half = cuda
        self.gpu = torch.cuda.get_device_name(0) if cuda else "CPU (sin GPU: mas lento)"
        if not modelo:
            modelo = "yolov8m-oiv7.pt" if cuda else "yolov8n-oiv7.pt"
        if not os.path.isabs(modelo):
            os.makedirs(MODELOS, exist_ok=True)
            modelo = os.path.join(MODELOS, modelo)
        # TensorRT (FP16, 'go2 vision_trt'): el mismo modelo 2-3 veces mas rapido en la RTX
        motor = os.path.splitext(modelo)[0] + ".engine"
        if cuda and os.path.exists(motor) and imgsz == 640:
            try:
                self.m = YOLO(motor, task="detect")
                self.archivo = motor
                self.gpu += " + TensorRT FP16"
            except Exception:
                self.m = None
        else:
            self.m = None
        if self.m is None:
            self.archivo = modelo
            self.m = YOLO(modelo)
        self.nombres = self.m.names
        self.modo = "GPU CUDA" if cuda else "CPU %d hilos" % torch.get_num_threads()
        self.imgsz, self.conf = imgsz, conf
        self.id_escalera = [k for k, v in self.nombres.items() if v == ESCALERA]
        self.m.predict(np.zeros((360, 640, 3), np.uint8), imgsz=imgsz, device=self.dev,
                       verbose=False)       # calentar (la primera inferencia en CUDA es lenta)

    def __call__(self, img):
        r = self.m.predict(img, imgsz=self.imgsz, conf=self.conf, device=self.dev,
                           verbose=False, max_det=60)[0]
        b = r.boxes
        if b is None or len(b) == 0:
            return np.zeros((0, 4)), np.zeros(0), np.zeros(0, int)
        return b.xyxy.cpu().numpy(), b.conf.cpu().numpy(), b.cls.cpu().numpy().astype(int)


def dibujar(img, dets, nombres):
    import cv2
    out = img.copy()
    for d in dets:
        x1, y1, x2, y2 = [int(v) for v in d["box"]]
        esc = d["clase"] == ESCALERA
        col = (0, 0, 255) if esc else (0, 200, 0)
        cv2.rectangle(out, (x1, y1), (x2, y2), col, 3 if esc else 2)
        txt = "%s %.0f%%" % (d["nombre"], 100 * d["conf"]) + (" %.1fm" % d["dist"] if d.get("dist") else "")
        cv2.putText(out, txt, (x1 + 3, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    return out


# ------------------------------------------------------------------ nodo ROS 2
def exportar_trt():
    """go2 vision_trt: convierte el modelo YOLO a TensorRT FP16 para la GPU (una vez, ~5-10 min)."""
    import torch
    from ultralytics import YOLO
    if not torch.cuda.is_available():
        print("sin GPU CUDA: TensorRT no aplica")
        return 1
    pt = os.path.join(MODELOS, "yolov8m-oiv7.pt")
    if not os.path.exists(pt):
        YOLO(pt)                                  # lo baja
    print("exportando %s a TensorRT FP16 en %s (tarda unos minutos)..." % (pt, torch.cuda.get_device_name(0)))
    ruta = YOLO(pt).export(format="engine", half=True, imgsz=640, device=0, workspace=2, verbose=False)
    print("listo: %s\nrelanza 'go2 lanzar': la vision lo usa sola ('go2 vision_probar' muestra la velocidad)" % ruta)
    return 0


def main():
    if "--trt" in sys.argv:
        return exportar_trt()
    if "--probar" in sys.argv:
        return probar(sys.argv[sys.argv.index("--probar") + 1:])
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    from .nube import a_cuerpo, leer_xyz, yaw_de_quat

    class Vision(Node):
        def __init__(self):
            super().__init__("go2_vision")
            p = self.declare_parameter
            self.fuente_pedida = p("fuente", "auto").value          # auto | multicast | videohub
            self.iface = p("iface", os.environ.get("GO2_IF", "")).value
            self.hfov = math.radians(p("hfov_grados", 120.0).value)  # campo de vision horizontal
            self.cam_x = p("camara_x", 0.32).value                   # camara: en el hocico
            self.umbral_esc = p("umbral_escalera", 0.35).value
            self.det = Detector(p("modelo", "").value, p("imgsz", 640).value, p("conf", 0.25).value,
                                p("dispositivo", "auto").value)
            self.get_logger().info("modelo %s en %s" % (os.path.basename(self.det.archivo), self.det.gpu))
            sd = qos_profile_sensor_data
            self.pub = self.create_publisher(String, "/go2/vision", 1)
            self.pub_img = self.create_publisher(Image, "/go2/vision/imagen", sd)
            self.pub_mk = self.create_publisher(MarkerArray, "/go2/vision/marcas", 1)
            self.create_subscription(PointCloud2, "/go2/obstaculos", self.cb_obst, sd)
            self.create_subscription(Odometry, "odom", self.cb_odom, sd)
            self.Image, self.Marker, self.MarkerArray, self.String = Image, Marker, MarkerArray, String
            self.a_cuerpo, self.leer_xyz, self.yaw_q = a_cuerpo, leer_xyz, yaw_de_quat
            self.obst, self.pose = None, None
            self.fuente, self.t_fuente = None, 0.0
            self.hist = collections.deque(maxlen=8)      # escalera en los ultimos 8 frames
            self.stats = {"n": 0, "ms": 0.0, "t0": time.monotonic()}
            self.estado = {}
            self.t_img = self.t_foto = 0.0
            self.t_ult_inf = 0.0
            self.vivo = True
            threading.Thread(target=self.lazo, daemon=True).start()

        def cb_obst(self, m):
            self.obst = m

        def cb_odom(self, m):
            p = m.pose.pose.position
            self.pose = (p.x, p.y, self.yaw_q(m.pose.pose.orientation))

        # -------------------------------------------------- camara
        def abrir(self, cual):
            if cual == "multicast":
                return FuenteMulticast(self.iface)
            return FuenteVideohub(self)

        def asegurar_fuente(self):
            f = self.fuente
            if f is not None and (time.monotonic() - f.t < 3.0 or time.monotonic() - self.t_fuente < 6.0):
                return f
            if f is not None:
                self.get_logger().warn("la camara (%s) no manda imagenes" % f.nombre)
                f.cerrar()
            orden = {"multicast": ["multicast"], "videohub": ["videohub"]}.get(
                self.fuente_pedida, ["multicast", "videohub"])
            if f is not None and f.nombre in orden and len(orden) > 1:
                orden.remove(f.nombre)
                orden.append(f.nombre)
            for cual in orden:
                try:
                    g = self.abrir(cual)
                except Exception as e:
                    self.get_logger().warn("camara %s: %s" % (cual, e))
                    continue
                t = time.monotonic()
                while time.monotonic() - t < 5.0 and g.ultimo is None:
                    time.sleep(0.05)
                if g.ultimo is not None:
                    self.get_logger().info("camara: %s (%dx%d)" % (cual, g.ultimo.shape[1], g.ultimo.shape[0]))
                    self.fuente, self.t_fuente = g, time.monotonic()
                    return g
                g.cerrar()
                self.get_logger().warn("camara %s: nada en 5 s" % cual)
            self.fuente = None
            return None

        # -------------------------------------------------- lazo de inferencia (siempre el ultimo frame)
        def lazo(self):
            """Nunca se muere: si algo falla (CUDA, una imagen rota, la camara), lo dice en
            /go2/vision ('error') y en la terminal, espera un poco y sigue."""
            import traceback
            self.visto_t = 0.0
            t_aviso = 0.0
            while self.vivo and rclpy.ok():
                try:
                    self.paso()
                    self.error = None
                except Exception as e:
                    self.error = "%s: %s" % (type(e).__name__, e)
                    if time.monotonic() - t_aviso > 10.0:
                        t_aviso = time.monotonic()
                        self.get_logger().error("vision: %s\n%s" % (self.error, traceback.format_exc()))
                    try:
                        self.publicar_estado([], None, getattr(self.fuente, "nombre", "ninguna"))
                    except Exception:
                        pass
                    time.sleep(0.5)

        def paso(self):
            if True:
                f = self.asegurar_fuente()
                if f is None:
                    self.publicar_estado([], None, "ninguna")
                    time.sleep(2.0)
                    return
                if f.t <= self.visto_t or (self.det.dev == "cpu" and time.monotonic() - self.t_ult_inf < 0.125):
                    time.sleep(0.003)
                    return
                self.t_ult_inf = time.monotonic()
                self.visto_t, img = f.t, f.ultimo
                t0 = time.perf_counter()
                xyxy, conf, cls = self.det(img)
                ms = (time.perf_counter() - t0) * 1000
                dets = self.ubicar(img, xyxy, conf, cls)
                self.stats["n"] += 1
                self.stats["ms"] = max(self.stats["ms"], ms)
                esc = self.escalera(dets)
                self.publicar_estado(dets, esc, f.nombre)
                self.publicar_vista(img, dets)

        def ubicar(self, img, xyxy, conf, cls):
            H, W = img.shape[:2]
            fpx = (W / 2.0) / math.tan(self.hfov / 2.0)
            pts = None
            if self.obst is not None and self.pose is not None:
                xyz = self.leer_xyz(self.obst)
                if len(xyz):
                    x, y, yaw = self.pose
                    fx, fy = self.a_cuerpo(xyz, x, y, yaw)
                    cx = fx - self.cam_x
                    ok = cx > 0.05
                    pts = (np.arctan2(fy[ok], cx[ok]), np.hypot(fx[ok], fy[ok]))
            dets = []
            for b, c, k in zip(xyxy, conf, cls):
                nombre = self.det.nombres.get(int(k), str(k))
                if nombre in IGNORAR:
                    continue
                # imagen: x a la derecha = rumbo negativo (ROS: angulos positivos a la izquierda)
                a1 = -math.atan((b[2] - W / 2.0) / fpx)
                a2 = -math.atan((b[0] - W / 2.0) / fpx)
                d = {"clase": nombre, "nombre": ES.get(nombre, nombre.lower()), "conf": round(float(c), 2),
                     "rumbo": round((a1 + a2) / 2, 3), "box": [round(float(v), 1) for v in b], "dist": None,
                     "abajo": round(float(b[3]) / H, 2)}
                if pts is not None:
                    sel = (pts[0] >= a1 - 0.03) & (pts[0] <= a2 + 0.03) & (pts[1] < 8.0)
                    if np.count_nonzero(sel) >= 3:
                        dist = float(np.percentile(pts[1][sel], 25))
                        x, y, yaw = self.pose
                        d["dist"] = round(dist, 2)
                        d["x"] = round(x + dist * math.cos(yaw + d["rumbo"]), 2)
                        d["y"] = round(y + dist * math.sin(yaw + d["rumbo"]), 2)
                dets.append(d)
            dets.sort(key=lambda d: -d["conf"])
            return dets

        def escalera(self, dets):
            e = [d for d in dets if d["clase"] == ESCALERA]
            mejor = e[0] if e else None
            self.hist.append(mejor if mejor and mejor["conf"] >= self.umbral_esc else None)
            buenos = [d for d in self.hist if d is not None]
            vista = len(buenos) >= 3                  # 3 de los ultimos 8 frames: no un frame suelto
            out = {"vista": vista, "frames": len(buenos), "conf": mejor["conf"] if mejor else 0.0}
            if buenos:
                out["rumbo"] = round(float(np.median([d["rumbo"] for d in buenos])), 3)
                con_dist = [d for d in buenos if d.get("dist")]
                if con_dist:
                    u = con_dist[-1]
                    out.update({"dist": u["dist"], "x": u["x"], "y": u["y"]})
            return out

        def publicar_estado(self, dets, esc, fuente):
            s = self.stats
            dt = time.monotonic() - s["t0"]
            if dt >= 1.0:
                self.estado = {"fps": round(s["n"] / dt, 1), "ms_max": round(s["ms"], 1)}
                self.stats = {"n": 0, "ms": 0.0, "t0": time.monotonic()}
            e = dict(self.estado)
            e.update({"t": time.time(), "fuente": fuente, "gpu": self.det.gpu, "hfov": round(self.hfov, 3),
                      "error": getattr(self, "error", None),
                      "objetos": [{k: d[k] for k in ("clase", "nombre", "conf", "rumbo", "dist", "x", "y") if k in d}
                                  for d in dets[:15]],
                      "escalera": esc or {"vista": False}})
            self.pub.publish(self.String(data=json.dumps(e)))

        def publicar_vista(self, img, dets):
            ahora = time.monotonic()
            if ahora - self.t_img < 0.1:
                return
            self.t_img = ahora
            import cv2
            vista = dibujar(img, dets, self.det.nombres)
            if ahora - self.t_foto > 2.0:
                self.t_foto = ahora
                try:
                    cv2.imwrite(os.path.join(DATOS, "vision_ultima.jpg"), vista)
                except Exception:
                    pass
            peq = cv2.resize(vista, (640, int(640 * vista.shape[0] / vista.shape[1])))
            m = self.Image()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "camara"
            m.height, m.width = peq.shape[:2]
            m.encoding, m.is_bigendian, m.step = "bgr8", 0, peq.shape[1] * 3
            m.data = peq.tobytes()
            self.pub_img.publish(m)
            # etiquetas en el mapa
            arr = self.MarkerArray()
            for i, d in enumerate([d for d in dets if d.get("dist")][:20]):
                esc = d["clase"] == ESCALERA
                for j, tipo in enumerate((self.Marker.TEXT_VIEW_FACING, self.Marker.CYLINDER)):
                    k = self.Marker()
                    k.header.frame_id, k.header.stamp = "odom", m.header.stamp
                    k.ns, k.id, k.type, k.action = "vision", 2 * i + j, tipo, self.Marker.ADD
                    k.pose.position.x, k.pose.position.y = d["x"], d["y"]
                    k.pose.position.z = 0.7 if j == 0 else 0.2
                    k.pose.orientation.w = 1.0
                    k.scale.x = k.scale.y = 0.18 if j else 0.0
                    k.scale.z = 0.25 if j == 0 else 0.4
                    k.color.r, k.color.g, k.color.b, k.color.a = (1.0, 0.1, 0.1, 0.9) if esc else (0.1, 0.9, 0.2, 0.9)
                    k.text = "%s %.0f%%" % (d["nombre"], 100 * d["conf"]) if j == 0 else ""
                    k.lifetime.nanosec = 600000000
                    arr.markers.append(k)
            if arr.markers:
                self.pub_mk.publish(arr)

    try:
        os.nice(10)          # la vision nunca le quita CPU a la navegacion
    except OSError:
        pass
    rclpy.init()
    n = Vision()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n.vivo = False
        if n.fuente is not None:
            n.fuente.cerrar()
        n.destroy_node()
        rclpy.try_shutdown()


# ------------------------------------------------------------------ prueba sin ROS
def probar(args):
    """go2 vision_probar [foto.jpg | camara]: GPU, velocidad del modelo y (opcional) una foto."""
    import cv2
    import torch
    print("torch %s | CUDA disponible: %s" % (torch.__version__, torch.cuda.is_available()))
    if torch.cuda.is_available():
        print("GPU: %s" % torch.cuda.get_device_name(0))
    else:
        print("!! sin GPU: revisa el driver NVIDIA (nvidia-smi). Uso el CPU con el modelo chico.")
    det = Detector()
    print("modelo: %s (%d clases, 'Stairs' = clase %s)" % (det.archivo, len(det.nombres), det.id_escalera))
    img = np.random.randint(0, 255, (540, 960, 3), np.uint8)
    t = time.perf_counter()
    for _ in range(60):
        det(img)
    dt = (time.perf_counter() - t) / 60
    print("velocidad: %.1f ms por imagen = %.0f fps  [%s]" % (dt * 1000, 1.0 / dt, det.modo))
    if args and args[0] == "camara":
        f = FuenteMulticast(os.environ.get("GO2_IF", ""))
        t = time.monotonic()
        while f.ultimo is None and time.monotonic() - t < 6:
            time.sleep(0.05)
        if f.ultimo is None:
            print("camara multicast: no llega video (230.1.1.1:1720 por %s)" % (os.environ.get("GO2_IF") or "?"))
            f.cerrar()
            return 1
        n0, t = f.n, time.monotonic()
        time.sleep(3.0)
        print("camara multicast: %dx%d a %.0f fps" % (f.W, f.H, (f.n - n0) / (time.monotonic() - t)))
        img = f.ultimo
        f.cerrar()
    elif args:
        img = cv2.imread(args[0])
        if img is None:
            print("no pude abrir %s" % args[0])
            return 1
    else:
        return 0
    xyxy, conf, cls = det(img)
    dets = []
    for b, c, k in zip(xyxy, conf, cls):
        nombre = det.nombres.get(int(k), str(k))
        if nombre not in IGNORAR:
            dets.append({"clase": nombre, "nombre": ES.get(nombre, nombre.lower()), "conf": float(c), "box": b})
    for d in sorted(dets, key=lambda d: -d["conf"]):
        print("  %-18s %3.0f%%" % (d["nombre"], 100 * d["conf"]))
    salida = os.path.join(DATOS, "vision_prueba.jpg")
    os.makedirs(DATOS, exist_ok=True)
    cv2.imwrite(salida, dibujar(img, dets, det.nombres))
    print("imagen con detecciones: %s" % salida)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
