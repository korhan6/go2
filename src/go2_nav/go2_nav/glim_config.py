"""Config de GLIM para el Go2 (la genera 'go2 lanzar slam:=glim' en ~/.go2_nav/glim/<modo>).

Parte de la config oficial de GLIM (share/glim/config) y cambia lo del Go2:
  * nube:  /go2/l1_sin_cuerpo -- la nube CRUDA del L1 (marco del sensor, sello y 'time' por punto
           originales) sin los puntos del propio perro (se mueven con el: frenarian la odometria)
  * IMU:   /go2/imu -- la del CUERPO (go2_imu), mismo reloj del perro que la nube
  * T_lidar_imu del URDF de Unitree (base->imu y base->L1); NO la IMU interna del L1 (girada 122 grados)
  * GPU (CUDA) si GLIM esta instalado con CUDA; si no, CPU
  * mapeo: odometria + submapas + mapa global con cierre de lazos | odometria: solo odometria
  * su /tf va a /glim/tf: el arbol de TF lo arma go2_nav (driver: odom->base_link, slam3d: map->odom)
"""
import json
import math
import os
import re

T_BASE_IMU = (-0.02557, 0.0, 0.04232)                  # URDF Go2: imu_joint (sin giro)
T_BASE_L1 = (0.28945, 0.0, -0.046825)                  # URDF Go2: radar_joint
RPY_BASE_L1 = (0.0, 2.8782, 0.0)


def leer_jsonc(ruta):
    txt = open(ruta).read()
    out, i, n, en_txt = [], 0, len(txt), False
    while i < n:
        c = txt[i]
        if en_txt:
            out.append(c)
            if c == "\\":
                out.append(txt[i + 1])
                i += 1
            elif c == '"':
                en_txt = False
        elif c == '"':
            en_txt = True
            out.append(c)
        elif txt.startswith("//", i):
            i = txt.find("\n", i)
            i = n if i < 0 else i
            continue
        elif txt.startswith("/*", i):
            i = txt.find("*/", i) + 2
            continue
        else:
            out.append(c)
        i += 1
    return json.loads(re.sub(r",(\s*[}\]])", r"\1", "".join(out)))


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def quat(R):
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [(R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, 0.25 * s]
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i][i] - R[j][j] - R[k][k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j][i] + R[i][j]) / s
    q[k] = (R[k][i] + R[i][k]) / s
    q[3] = (R[k][j] - R[j][k]) / s
    return q


def t_lidar_imu():
    """[x y z qx qy qz qw] que lleva un punto del marco de la IMU al del L1 (convencion de GLIM)."""
    R = rot_rpy(*RPY_BASE_L1)                                   # R_imu_lidar (la IMU no gira)
    t_il = [T_BASE_L1[i] - T_BASE_IMU[i] for i in range(3)]      # el L1 en el marco de la IMU
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]          # R_lidar_imu = R^T
    t = [-sum(Rt[i][j] * t_il[j] for j in range(3)) for i in range(3)]
    return [round(v, 6) for v in t] + [round(v, 8) for v in quat(Rt)]


def hay_gpu(prefijo_glim):
    for d in ("lib", "lib/glim"):
        if os.path.exists(os.path.join(prefijo_glim, d, "libodometry_estimation_gpu.so")):
            return True
    return False


def generar(origen, destino, modo="mapeo", gpu=False):
    """origen: share/glim/config. Devuelve la carpeta generada."""
    os.makedirs(destino, exist_ok=True)
    cfg = {f: leer_jsonc(os.path.join(origen, f)) for f in os.listdir(origen) if f.endswith(".json")}
    dev = "gpu" if gpu else "cpu"
    g = cfg["config.json"]["global"]
    for k, base in (("config_odometry", "config_odometry_%s.json"), ("config_sub_mapping", "config_sub_mapping_%s.json"),
                    ("config_global_mapping", "config_global_mapping_%s.json")):
        if base % dev in cfg:
            g[k] = base % dev

    r = cfg["config_ros.json"]["glim_ros"]
    r.update({
        "enable_local_mapping": modo == "mapeo",
        "enable_global_mapping": modo == "mapeo",
        "imu_topic": "/go2/imu", "points_topic": "/go2/l1_sin_cuerpo", "image_topic": "",
        "imu_frame_id": "imu", "lidar_frame_id": "utlidar_lidar", "base_frame_id": "",
        "odom_frame_id": "odom", "map_frame_id": "map", "publish_imu2lidar": False,
        "acc_scale": 1.0,                                       # la IMU del cuerpo ya da m/s2
        "extension_modules": ["libmemory_monitor.so", "librviz_viewer.so"],
    })

    s = cfg["config_sensors.json"]["sensors"]
    s["T_lidar_imu"] = t_lidar_imu()
    s["intensity_field"] = "intensity"
    s["ring_field"] = ""
    for k, v in (("autoconf_perpoint_times", True), ("perpoint_relative_time", True), ("perpoint_time_scale", 1.0)):
        if k in s:
            s[k] = v

    p = cfg["config_preprocess.json"]["preprocess"]
    p.update({"distance_near_thresh": 0.3, "distance_far_thresh": 30.0, "use_random_grid_downsampling": False,
              "downsample_resolution": 0.10, "enable_outlier_removal": True, "outlier_removal_k": 10,
              "outlier_std_mul_factor": 2.0, "k_correspondences": 10, "num_threads": 4})

    o = cfg["config_odometry_%s.json" % dev]["odometry_estimation"]
    o["num_threads"] = 4
    if "initialization_mode" in o:
        o["initialization_mode"] = "NAIVE"                   # arranca QUIETO: velocidad inicial cero
    if dev == "cpu":
        for k, v in (("ivox_resolution", 0.5), ("ivox_min_dist", 0.05), ("vgicp_resolution", 0.25), ("max_iterations", 10)):
            if k in o:
                o[k] = v

    sm = cfg.get("config_sub_mapping_%s.json" % dev, {}).get("sub_mapping")
    if sm is not None:
        for k, v in (("keyframe_update_interval_trans", 0.5), ("keyframe_update_min_points", 300),
                     ("keyframe_voxel_resolution", 0.15), ("submap_downsample_resolution", 0.08),
                     ("submap_voxel_resolution", 0.25)):
            if k in sm:
                sm[k] = v
    gm = cfg.get("config_global_mapping_%s.json" % dev, {}).get("global_mapping")
    if gm is not None:
        for k, v in (("submap_voxel_resolution", 0.25), ("max_implicit_loop_distance", 30.0)):
            if k in gm:
                gm[k] = v

    for f, d in cfg.items():
        with open(os.path.join(destino, f), "w") as fh:
            json.dump(d, fh, indent=2)
    return destino
