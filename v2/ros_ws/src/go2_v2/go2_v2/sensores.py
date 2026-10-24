"""go2_sensores: la nube y la IMU del L1 en el marco 'body' que usa Point-LIO.

Reemplaza a transform_everything.py del stack de CMU, con la MISMA cuenta (go2_v2/transformacion.py,
probada contra la original en v2/pruebas/prueba_transformacion.py) pero:
  * vectorizada: ~1 ms por nube (la original iba punto por punto en Python: en una laptop sumaba
    mas de 1 s de demora a todo el sistema; CMU lo cuenta en su README);
  * la IMU no sale hasta conocer el desfase de relojes (la original mandaba las primeras con la
    hora del perro y despues saltaba ~3 s a la de la laptop: Point-LIO arrancaba torcido);
  * se suscribe "best effort" (sirve tanto si el perro publica confiable como si no);
  * publica su estado (/go2/sensores_estado, JSON 1 Hz): Hz, puntos, demora, ms por nube.

Entradas:  /utlidar/cloud (PointCloud2, marco del L1, reloj del perro), /utlidar/imu (Imu, ~250 Hz)
Salidas:   /utlidar/transformed_cloud, /utlidar/transformed_imu (giro; orientacion y aceleracion en
           cero, como CMU: Point-LIO usa solo el giro), /utlidar/transformed_raw_imu (completa)
Calibracion: ~/Desktop/imu_calib_data.yaml (la escribe 'go2v2 calibrar' = calibrate_imu de CMU).
"""
import array
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

from . import transformacion as T

FLOAT32 = 7


class Sensores(Node):
    def __init__(self):
        super().__init__("go2_sensores")
        P = self.declare_parameter
        ruta = os.path.expanduser(P("calibracion", "~/Desktop/imu_calib_data.yaml").value)
        self.calib, cargada = T.leer_calibracion(ruta)
        if cargada:
            self.get_logger().info("calibracion de la IMU cargada: %s" % ruta)
        else:
            self.get_logger().warn("SIN calibracion de la IMU (%s): uso los valores de CMU. "
                                   "Corre 'go2v2 calibrar' una vez (perro de pie)." % ruta)
        entrada = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_nube = self.create_publisher(PointCloud2, "/utlidar/transformed_cloud", 50)
        self.pub_imu = self.create_publisher(Imu, "/utlidar/transformed_imu", 50)
        self.pub_imu_raw = self.create_publisher(Imu, "/utlidar/transformed_raw_imu", 50)
        self.pub_est = self.create_publisher(String, "/go2/sensores_estado", 5)
        self.create_subscription(Imu, P("topico_imu", "/utlidar/imu").value, self.cb_imu, entrada)
        self.create_subscription(PointCloud2, P("topico_nube", "/utlidar/cloud").value, self.cb_nube, entrada)
        self.desfase_ns = None             # hora de la laptop - hora del perro (fija desde la 1a nube)
        self.est = {"nubes": 0, "imu": 0, "puntos": 0, "cuerpo": 0, "ms": 0.0, "demora_ms": 0.0,
                    "imu_descartadas": 0, "sin_xyz": 0}
        self.t_ult_nube = None
        self.t_ult_imu = None
        self.create_timer(1.0, self.estado)
        self.get_logger().info("go2_sensores listo: /utlidar/cloud + /utlidar/imu -> marco 'body' (Point-LIO)")

    # ------------------------------------------------------------------ nube
    def cb_nube(self, m):
        t0 = time.perf_counter()
        ahora_ns = self.get_clock().now().nanoseconds
        st_ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        if self.desfase_ns is None:
            self.desfase_ns = ahora_ns - st_ns
            self.get_logger().info("desfase de relojes laptop - perro: %.3f s" % (self.desfase_ns * 1e-9))
        n = m.width * m.height
        if n == 0:
            return
        campos = {f.name: f for f in m.fields}
        if not all(k in campos and campos[k].datatype == FLOAT32 for k in ("x", "y", "z")):
            self.est["sin_xyz"] += 1
            return
        paso = m.point_step
        datos, k = T.transformar_nube_bytes(m.data, n, paso, [campos[c].offset for c in ("x", "y", "z")])
        out = PointCloud2()
        out.header.frame_id = "body"
        st = st_ns + self.desfase_ns
        out.header.stamp.sec, out.header.stamp.nanosec = int(st // 1_000_000_000), int(st % 1_000_000_000)
        out.height, out.width = 1, k
        out.fields = m.fields
        out.is_bigendian = m.is_bigendian
        out.point_step = paso
        out.row_step = paso * k
        out.is_dense = m.is_dense
        # array('B') y no bytes: con bytes rclpy revisa byte por byte en Python (~20 ms por nube)
        buf = array.array("B")
        buf.frombytes(datos)
        out.data = buf
        self.pub_nube.publish(out)
        e = self.est
        e["nubes"] += 1
        e["puntos"] = k
        e["cuerpo"] = n - k
        e["ms"] = max(e["ms"], (time.perf_counter() - t0) * 1000.0)
        # demora = cuanto despues de medida llega la nube (con el desfase fijado en la 1a nube)
        e["demora_ms"] = max(e["demora_ms"], (ahora_ns - st) * 1e-6)
        self.t_ult_nube = time.monotonic()

    # ------------------------------------------------------------------ imu
    def cb_imu(self, m):
        if self.desfase_ns is None:
            self.est["imu_descartadas"] += 1          # todavia sin desfase: no mezclar relojes
            return
        g, a = T.transformar_imu((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z),
                                 (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z),
                                 self.calib)
        q = T.orientacion_imu((m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w))
        st = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec + self.desfase_ns
        i = Imu()
        i.header.stamp.sec, i.header.stamp.nanosec = int(st // 1_000_000_000), int(st % 1_000_000_000)
        i.header.frame_id = "body"
        i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w = q
        i.angular_velocity.x, i.angular_velocity.y, i.angular_velocity.z = g
        i.linear_acceleration.x, i.linear_acceleration.y, i.linear_acceleration.z = a
        self.pub_imu_raw.publish(i)
        # como CMU: a Point-LIO solo el giro (orientacion identidad, aceleracion cero)
        i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w = 0.0, 0.0, 0.0, 1.0
        i.linear_acceleration.x = i.linear_acceleration.y = i.linear_acceleration.z = 0.0
        self.pub_imu.publish(i)
        self.est["imu"] += 1
        self.t_ult_imu = time.monotonic()

    # ------------------------------------------------------------------ estado
    def estado(self):
        e = dict(self.est)
        e["ms"] = round(e["ms"], 2)
        e["demora_ms"] = round(e["demora_ms"], 1)
        ahora = time.monotonic()
        e["nube_hace_s"] = None if self.t_ult_nube is None else round(ahora - self.t_ult_nube, 2)
        e["imu_hace_s"] = None if self.t_ult_imu is None else round(ahora - self.t_ult_imu, 2)
        self.pub_est.publish(String(data=json.dumps(e)))
        if self.t_ult_nube is None:
            self.get_logger().warn("no llega /utlidar/cloud (cable? CycloneDDS por la interfaz del perro?)",
                                   throttle_duration_sec=10.0)
        elif ahora - self.t_ult_nube > 1.0:
            self.get_logger().warn("la nube del L1 no llega hace %.1f s" % (ahora - self.t_ult_nube),
                                   throttle_duration_sec=2.0)
        for k in ("nubes", "imu", "imu_descartadas", "sin_xyz"):
            self.est[k] = 0
        self.est["ms"] = 0.0
        self.est["demora_ms"] = 0.0


def main():
    rclpy.init()
    n = Sensores()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
