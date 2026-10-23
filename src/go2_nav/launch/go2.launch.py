"""Todo go2_nav con un comando:

    ros2 launch go2_nav go2.launch.py              # perro real por cable (Ethernet)
    ros2 launch go2_nav go2.launch.py rviz:=true   # + RViz con mapa, ruta y puntos fantasma
    ros2 launch go2_nav go2.launch.py sim:=true mundo:=pasillo   # sin perro (simulado)
    ros2 launch go2_nav go2.launch.py vision:=false  # sin la camara/YOLO
    ros2 launch go2_nav go2.launch.py mapa:=casa     # NAVEGAR con un mapa guardado (go2 guardar_mapa casa):
                                                     # el perro solo se ubica en el mapa (no lo cambia)
    ros2 launch go2_nav go2.launch.py slam:=toolbox  # el SLAM 2D de antes (slam_toolbox) en vez del 3D
    ros2 launch go2_nav go2.launch.py control:=false # sin el control PS4 (por defecto: si, si esta el paquete joy)

Vision (go2_vision): corre con el Python de ~/.go2_nav/venv_vision (PyTorch + YOLO con CUDA, lo
instala instalar.sh), aparte del Python del sistema que usa ROS.

Cadena de velocidad: controller(MPPI) -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
                     -> collision_monitor -> cmd_vel -> go2_driver -> Move del perro (20 Hz)
"""
import functools
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


VENV_PY = os.path.expanduser("~/.go2_nav/venv_vision/bin/python")
MAPAS = os.path.expanduser("~/.go2_nav/mapas")


def _slam(context):
    """SLAM: mapeando (sin mapa:=) o ubicandose en un mapa guardado (mapa:=NOMBRE).
    slam:=go2 (por defecto) = go2_slam3d (nube 3D completa contra un mapa 3D; mapa 2D sin rayos sueltos)
    slam:=toolbox = slam_toolbox 2D (el de antes) + go2_mapa3d."""
    mapa = LaunchConfiguration("mapa").perform(context).strip()
    if LaunchConfiguration("slam").perform(context) != "toolbox":
        if mapa and not os.path.exists(os.path.join(MAPAS, mapa + ".slam3d.npz")):
            hay = sorted(f[:-11] for f in os.listdir(MAPAS) if f.endswith(".slam3d.npz")) if os.path.isdir(MAPAS) else []
            return [LogInfo(msg="!! NO EXISTE el mapa '%s' (%s.slam3d.npz). Mapas guardados: %s. "
                                "Lanza sin mapa:= y guardalo con: go2 guardar_mapa %s"
                                % (mapa, os.path.join(MAPAS, mapa), hay, mapa))]
        return [Node(package="go2_nav", executable="slam3d", name="go2_slam3d", output="screen",
                     parameters=[{"mapa": mapa}] + _ajustes()[0])]
    return _slam_toolbox(context) + [
        Node(package="go2_nav", executable="mapa3d", name="go2_mapa3d", output="screen",
             parameters=[{"mapa": mapa}] + _ajustes()[0])]


def _slam_toolbox(context):
    import yaml
    share = get_package_share_directory("go2_nav")
    slam = os.path.join(share, "config", "slam.yaml")
    st = get_package_share_directory("slam_toolbox")
    mapa = LaunchConfiguration("mapa").perform(context).strip()
    if not mapa:
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(st, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam, "use_sim_time": "false"}.items())]
    base = os.path.join(MAPAS, mapa)
    if not os.path.exists(base + ".posegraph"):
        return [LogInfo(msg="!! NO EXISTE el mapa '%s' (%s.posegraph). Mapas guardados: %s. "
                            "Lanza sin mapa:= y guardalo con: go2 guardar_mapa %s"
                            % (mapa, base, sorted(f[:-10] for f in os.listdir(MAPAS) if f.endswith(".posegraph"))
                               if os.path.isdir(MAPAS) else [], mapa))]
    with open(slam) as f:
        p = yaml.safe_load(f)
    q = p["slam_toolbox"]["ros__parameters"]
    q.update({"mode": "localization", "map_file_name": base,
              # arranca donde EMPEZO el mapeo; si el perro esta en otro lado: "2D Pose Estimate" en RViz
              "map_start_at_dock": True,
              # localizacion: pocas nubes en el buffer y cierre de lazo rapido (como el ejemplo oficial)
              "scan_buffer_size": 3, "loop_match_minimum_chain_size": 3})
    tmp = "/tmp/go2_slam_localizacion.yaml"
    with open(tmp, "w") as f:
        yaml.safe_dump(p, f)
    return [LogInfo(msg="go2_nav: NAVEGANDO en el mapa guardado '%s' (no se modifica)" % mapa),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(st, "launch", "localization_launch.py")),
                launch_arguments={"slam_params_file": tmp, "use_sim_time": "false"}.items())]


AJUSTES = os.path.expanduser("~/.go2_nav/ajustes.yaml")


def _controlador(context):
    """controller_server. obstaculos:=stvl -> el costmap LOCAL usa Spatio-Temporal Voxel Layer
    (voxeles 3D que se olvidan solos en 4 s: hecho para lidares ralos como el L1) en vez de la capa
    2D. Si el paquete no esta instalado, avisa y usa la capa 2D."""
    import yaml
    share = get_package_share_directory("go2_nav")
    nav2 = os.path.join(share, "config", "nav2.yaml")
    extra = _ajustes()[0]
    acc = []
    if LaunchConfiguration("obstaculos").perform(context) == "stvl":
        try:
            get_package_share_directory("spatio_temporal_voxel_layer")
            with open(nav2) as f:
                d = yaml.safe_load(f)
            lc = d["local_costmap"]["local_costmap"]["ros__parameters"]
            lc["plugins"] = ["stvl_layer", "inflation_layer"]
            lc["stvl_layer"] = {
                "plugin": "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer",
                "enabled": True, "voxel_decay": 4.0, "decay_model": 0, "voxel_size": 0.05,
                "track_unknown_space": True, "mark_threshold": 0, "update_footprint_enabled": True,
                "combination_method": 1, "origin_z": -1.0, "publish_voxel_map": True,
                "transform_tolerance": 0.3, "mapping_mode": False, "map_save_duration": 60.0,
                "observation_sources": "obstaculos",
                "obstaculos": {"data_type": "PointCloud2", "topic": "/go2/obstaculos", "marking": True,
                               "sensor_frame": "lidar",
                               "clearing": True, "obstacle_range": 3.4, "min_obstacle_height": -1.0,
                               "max_obstacle_height": 2.0, "expected_update_rate": 0.0,
                               "observation_persistence": 0.0, "inf_is_valid": False, "filter": "voxel",
                               "voxel_min_points": 0, "clear_after_reading": True, "model_type": 1,
                               "vertical_fov_angle": 1.57, "vertical_fov_padding": 0.05,
                               "horizontal_fov_angle": 6.28, "decay_acceleration": 5.0,
                               "max_z": 2.0, "min_z": 0.1}}
            nav2 = "/tmp/go2_nav2_stvl.yaml"
            with open(nav2, "w") as f:
                yaml.safe_dump(d, f)
            acc.append(LogInfo(msg="go2_nav: obstaculos con STVL (voxeles 3D, se olvidan en 4 s)"))
        except Exception as e:
            acc.append(LogInfo(msg="!! STVL no disponible (%s): uso la capa 2D. Instalar: "
                                   "sudo apt install ros-$ROS_DISTRO-spatio-temporal-voxel-layer" % e))
    acc.append(Node(package="nav2_controller", executable="controller_server", output="screen",
                    parameters=[nav2] + extra, remappings=[("cmd_vel", "cmd_vel_nav")],
                    respawn=True, respawn_delay=2.0))
    return acc


def _nube(context):
    extra = _ajustes()[0]
    sim = LaunchConfiguration("sim").perform(context) == "true"
    if LaunchConfiguration("nube").perform(context) == "unitree" or sim:
        return [Node(package="go2_nav", executable="limpiador", name="go2_limpiador", output="screen",
                     parameters=extra)]
    return [Node(package="go2_nav", executable="nube", name="go2_nube", output="screen", parameters=extra),
            Node(package="go2_nav", executable="limpiador", name="go2_limpiador", output="screen",
                 parameters=[{"topico_nube": "/go2/nube_odom"}] + extra)]


@functools.lru_cache(maxsize=None)
def _ajustes():
    """Tus ajustes (~/.go2_nav/ajustes.yaml, ver config/ajustes_ejemplo.yaml): van DESPUES de los
    valores del paquete, asi que los pisan. Ese archivo no se borra al actualizar el codigo.
    Se revisa antes (go2_nav/ajustes.py): un error de tipeo ahi tiraba TODOS los nodos; ahora lo
    que no sirve se avisa y se ignora."""
    import yaml
    from go2_nav import ajustes
    from go2_nav.nucleo import config
    with open(os.path.join(get_package_share_directory("go2_nav"), "config", "nav2.yaml")) as f:
        paquete = ajustes.nodos(yaml.safe_load(f))[0]
    paquete["go2_escaleras"] = dict(config.DEFECTO)
    salida = os.path.join(tempfile.gettempdir(), "go2_nav_ajustes_%d.yaml" % os.getuid())
    return ajustes.leer(AJUSTES, paquete, salida)


def _vision_y_gestor(context):
    sim = LaunchConfiguration("sim").perform(context) == "true"
    vision = LaunchConfiguration("vision").perform(context) == "true"
    acciones = []
    # subida: la camara confirma (y puede vetar) SOLO las subidas; una bajada la confirma el lidar
    # viendo el primer escalon (la camara, a 30 cm del suelo, no ve una bajada desde arriba).
    # En la simulacion no hay camara: solo lidar.
    esc_vision = LaunchConfiguration("escaleras_vision").perform(context) or ("off" if sim else "subida")
    acciones.append(Node(package="go2_nav", executable="gestor", name="go2_gestor", output="screen",
                         parameters=[{"escaleras": LaunchConfiguration("escaleras").perform(context),
                                      "escaleras_vision": esc_vision,
                                      "mapa": LaunchConfiguration("mapa").perform(context).strip(),
                                      "slam": LaunchConfiguration("slam").perform(context)}] + _ajustes()[0]))
    # control PS4: joy (lee el DualShock por USB/Bluetooth; espera si aun no esta conectado) + go2_ps4
    control = LaunchConfiguration("control").perform(context)
    if control == "true" or (control == "auto" and not sim):
        try:
            get_package_share_directory("joy")
            acciones += [Node(package="joy", executable="joy_node", name="joy_node", output="log",
                              parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}]),
                         Node(package="go2_nav", executable="ps4", name="go2_ps4", output="screen")]
        except Exception:
            acciones.append(LogInfo(msg="control PS4 NO arranca: falta el paquete joy "
                                        "(sudo apt install ros-$ROS_DISTRO-joy)"))
    if vision and not sim:
        if os.path.exists(VENV_PY):
            acciones.append(ExecuteProcess(
                cmd=[VENV_PY, "-m", "go2_nav.vision", "--ros-args", "-r", "__node:=go2_vision",
                     "-p", "fuente:=" + LaunchConfiguration("camara").perform(context)],
                name="vision", output="screen", respawn=True, respawn_delay=5.0))
        else:
            acciones.append(LogInfo(msg="go2_vision NO arranca: falta %s (corre instalar.sh)" % VENV_PY))
    return acciones


def generate_launch_description():
    share = get_package_share_directory("go2_nav")
    nav2 = os.path.join(share, "config", "nav2.yaml")
    bt = os.path.join(share, "config", "ruta_comprometida.xml")
    rviz_cfg = os.path.join(share, "rviz", "go2.rviz")
    with open(os.path.join(share, "urdf", "go2.urdf")) as f:
        urdf = f.read()
    extra, aviso = _ajustes()

    sim = LaunchConfiguration("sim")
    rviz = LaunchConfiguration("rviz")
    mundo = LaunchConfiguration("mundo")
    fantasmas = LaunchConfiguration("fantasmas")
    nav2_nodos = ["controller_server", "planner_server", "behavior_server", "bt_navigator",
                  "velocity_smoother", "collision_monitor"]
    remap_vel = [("cmd_vel", "cmd_vel_nav")]

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("mundo", default_value="casa"),
        DeclareLaunchArgument("fantasmas", default_value="false"),
        DeclareLaunchArgument("inicio_sim", default_value=""),
        DeclareLaunchArgument("puntos_sim", default_value="0"),        # 2500 = lidar ralo como el L1
        # al_final: termina el piso y despues toma las escaleras | inmediato | off
        DeclareLaunchArgument("escaleras", default_value="al_final"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("camara", default_value="auto"),          # auto | multicast | videohub
        DeclareLaunchArgument("escaleras_vision", default_value=""),    # subida | requerida | off
        DeclareLaunchArgument("nube", default_value="cruda"),
        DeclareLaunchArgument("obstaculos", default_value="2d"),        # 2d | stvl (voxeles 3D que se olvidan)           # cruda (go2_nube) | unitree (cloud_deskewed)
        DeclareLaunchArgument("slam", default_value="go2"),             # go2 (SLAM 3D) | toolbox (slam_toolbox 2D)
        DeclareLaunchArgument("mapa", default_value=""),                # NOMBRE = navegar en ~/.go2_nav/mapas/NOMBRE
        DeclareLaunchArgument("control", default_value="auto"),         # control PS4: auto (no en sim) | true | false

        Node(package="go2_nav", executable="perro_sim", name="go2_perro_sim", output="screen",
             parameters=[{"mundo": mundo, "fantasmas": ParameterValue(fantasmas, value_type=bool),
                          "inicio": LaunchConfiguration("inicio_sim"),
                          "puntos_por_nube": ParameterValue(LaunchConfiguration("puntos_sim"), value_type=int)}],
             condition=IfCondition(sim)),
        LogInfo(msg=aviso or "go2_nav: sin ajustes propios (%s no existe): valores del paquete" % AJUSTES),
        Node(package="go2_nav", executable="driver", name="go2_driver", output="screen", parameters=extra),
        # donde esta el L1 en el perro (URDF del Go2: 0.29 m adelante del centro, en la cabeza).
        # Los costmaps borran obstaculos con rayos desde el sensor: desde base_link, cada rayo a un
        # punto detras de un marco de puerta "atravesaba" el marco y lo borraba
        Node(package="tf2_ros", executable="static_transform_publisher", name="tf_lidar",
             arguments=["--x", "0.29", "--frame-id", "base_link", "--child-frame-id", "lidar"]),
        # el CUERPO del perro en RViz (modelo de Unitree; las patas se mueven con /joint_states del driver)
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="go2_modelo",
             parameters=[{"robot_description": urdf}]),
        # nube: cruda (por defecto) = /utlidar/cloud del sensor llevada a 'odom' en ROS 2 (go2_nube);
        #       unitree = /utlidar/cloud_deskewed procesada por el perro ('go2 escala' la midio moviendose
        #       con el perro: no sirve para mapear)
        OpaqueFunction(function=_nube),
        Node(package="go2_nav", executable="escaleras", name="go2_escaleras", output="screen", parameters=extra),
        # launch oficial de slam_toolbox (en Jazzy es un nodo lifecycle: su launch lo activa solo):
        # mapeando, o ubicandose en un mapa guardado con mapa:=NOMBRE
        OpaqueFunction(function=_slam),

        OpaqueFunction(function=_controlador),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[nav2] + extra, respawn=True, respawn_delay=2.0),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[nav2] + extra, remappings=remap_vel, respawn=True, respawn_delay=2.0),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[nav2, {"default_nav_to_pose_bt_xml": bt}] + extra, respawn=True, respawn_delay=2.0),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="screen", parameters=[nav2] + extra, remappings=remap_vel, respawn=True, respawn_delay=2.0),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             output="screen", parameters=[nav2] + extra, respawn=True, respawn_delay=2.0),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"autostart": True, "node_names": nav2_nodos, "bond_timeout": 8.0,
                                      "attempt_respawn_reconnection": True, "bond_respawn_max_duration": 15.0}]),

        OpaqueFunction(function=_vision_y_gestor),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_cfg],
             condition=IfCondition(rviz)),
    ])
