"""Todo go2_nav con un comando:

    ros2 launch go2_nav go2.launch.py              # perro real por cable (Ethernet)
    ros2 launch go2_nav go2.launch.py rviz:=true   # + RViz con mapa, ruta y puntos fantasma
    ros2 launch go2_nav go2.launch.py sim:=true mundo:=pasillo   # sin perro (simulado)
    ros2 launch go2_nav go2.launch.py vision:=false  # sin la camara/YOLO
    ros2 launch go2_nav go2.launch.py mapa:=casa     # NAVEGAR con un mapa guardado (go2 guardar_mapa casa):
                                                     # el perro solo se ubica en el mapa (no lo cambia)

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
    """SLAM: mapeando (sin mapa:=) o ubicandose en un mapa guardado (mapa:=NOMBRE)."""
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
                                      "mapa": LaunchConfiguration("mapa").perform(context).strip()}] + _ajustes()[0]))
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
        DeclareLaunchArgument("mapa", default_value=""),                # NOMBRE = navegar en ~/.go2_nav/mapas/NOMBRE

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
        Node(package="go2_nav", executable="limpiador", name="go2_limpiador", output="screen", parameters=extra),
        Node(package="go2_nav", executable="escaleras", name="go2_escaleras", output="screen", parameters=extra),
        # launch oficial de slam_toolbox (en Jazzy es un nodo lifecycle: su launch lo activa solo):
        # mapeando, o ubicandose en un mapa guardado con mapa:=NOMBRE
        OpaqueFunction(function=_slam),

        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[nav2] + extra, remappings=remap_vel, respawn=True, respawn_delay=2.0),
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
