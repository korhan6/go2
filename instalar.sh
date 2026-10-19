#!/bin/bash
# go2_nav - instalador para la LAPTOP (Ubuntu 24.04 -> ROS 2 Jazzy; Ubuntu 22.04 -> Humble)
#   tar xzf go2_nav_ros2.tar.gz && cd go2_nav_ros2 && bash instalar.sh
#   SIN_VISION=1 bash instalar.sh    -> sin la vision por computadora (camara + YOLO en la GPU)
# Deja todo en ~/go2_ws y el comando 'go2'. Pide la clave de sudo (apt).
set -e
AQUI="$(cd "$(dirname "$0")" && pwd)"
WS="$HOME/go2_ws"

if [ -n "$VIRTUAL_ENV" ]; then
  echo "Tienes un entorno virtual de Python activo ($VIRTUAL_ENV)."
  echo "ROS 2 usa el Python del sistema: corre 'deactivate' y vuelve a ejecutar: bash instalar.sh"
  exit 1
fi

. /etc/os-release
case "$VERSION_CODENAME" in
  noble) DISTRO=jazzy ;;
  jammy) DISTRO=humble ;;
  *) echo "Ubuntu $VERSION_ID no soportado (usa 24.04 o 22.04)"; exit 1 ;;
esac
echo "== Ubuntu $VERSION_ID -> ROS 2 $DISTRO"

if [ ! -f /opt/ros/$DISTRO/setup.bash ]; then
  echo "== Instalando ROS 2 $DISTRO (repositorio oficial)"
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi
echo "== Paquetes (Nav2, slam_toolbox, CycloneDDS, RViz)"
sudo apt-get update
sudo apt-get install -y ros-$DISTRO-ros-base ros-$DISTRO-navigation2 ros-$DISTRO-slam-toolbox \
  ros-$DISTRO-rmw-cyclonedds-cpp ros-$DISTRO-rosidl-generator-dds-idl ros-$DISTRO-rviz2 ros-$DISTRO-nav2-rviz-plugins \
  python3-colcon-common-extensions python3-numpy python3-scipy git

echo "== Red: buffers grandes para las nubes del lidar"
printf 'net.core.rmem_max=2147483647\nnet.core.rmem_default=8388608\nnet.ipv4.ipfrag_time=3\nnet.ipv4.ipfrag_high_thresh=134217728\n' | sudo tee /etc/sysctl.d/60-go2-dds.conf >/dev/null
sudo sysctl -p /etc/sysctl.d/60-go2-dds.conf >/dev/null || true

echo "== Espacio de trabajo $WS"
mkdir -p "$WS/src"
rm -rf "$WS/src/go2_nav" "$WS/src/go2_nav_interfaces"
cp -r "$AQUI/src/go2_nav" "$AQUI/src/go2_nav_interfaces" "$WS/src/"
if [ ! -d "$WS/src/unitree_ros2" ]; then
  # mensajes oficiales de Unitree (unitree_api, unitree_go) para hablar con el perro por DDS
  git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2.git "$WS/src/unitree_ros2"
fi
touch "$WS/src/unitree_ros2/example/COLCON_IGNORE"
[ -d "$WS/src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg" ] && touch "$WS/src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg/COLCON_IGNORE"

echo "== Compilando"
set +u
source /opt/ros/$DISTRO/setup.bash
cd "$WS"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

cp "$AQUI/go2_env.sh" "$WS/go2_env.sh"
sed -i "s/__DISTRO__/$DISTRO/" "$WS/go2_env.sh"
mkdir -p "$HOME/.local/bin"
cat > "$HOME/.local/bin/go2" <<EOF
#!/bin/bash
source "$WS/go2_env.sh" >/dev/null
case "\$1" in
  lanzar)  shift
           # sin IP en el cable, ROS arranca por otra interfaz y no ve al perro: esperar el cable
           for i in \$(seq 90); do
             ip -o -4 addr show | grep -q " 192\\.168\\.123\\." && break
             [ "\$i" = 1 ] && echo "esperando el cable al perro (IP 192.168.123.x)... revisa el cable"
             sleep 1
           done
           ip -o -4 addr show | grep -q " 192\\.168\\.123\\." || { echo "sin cable al perro: no lanzo (para simular: go2 sim)"; exit 1; }
           source "$WS/go2_env.sh"
           exec ros2 launch go2_nav go2.launch.py "\$@" ;;
  sim)     shift; exec ros2 launch go2_nav go2.launch.py sim:=true rviz:=true "\$@" ;;
  rviz)    exec rviz2 -d "$WS/install/go2_nav/share/go2_nav/rviz/go2.rviz" ;;
  vision_probar) shift; exec "\$HOME/.go2_nav/venv_vision/bin/python" -m go2_nav.vision --probar "\$@" ;;
  *)       exec "$WS/install/go2_nav/lib/go2_nav/go2" "\$@" ;;
esac
EOF
chmod +x "$HOME/.local/bin/go2"
if ! grep -q "go2_ws/go2_env.sh" "$HOME/.bashrc"; then
  echo "source $WS/go2_env.sh" >> "$HOME/.bashrc"
fi

# ---------------------------------------------------------------- vision por computadora
# Camara del perro (H.264 por el cable) + YOLO en la GPU NVIDIA. PyTorch va en un entorno aparte
# (~/.go2_nav/venv_vision) que VE los paquetes de ROS pero no los toca.
if [ -z "$SIN_VISION" ]; then
  echo "== Vision por computadora (camara del perro + YOLO en la GPU)"
  sudo apt-get install -y python3-venv python3-pip gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
  if nvidia-smi -L >/dev/null 2>&1; then
    nvidia-smi -L
  else
    echo "!! No veo la GPU NVIDIA (falta el driver). Instalalo con:"
    echo "     sudo ubuntu-drivers install     y REINICIA la laptop; despues corre otra vez: bash instalar.sh"
    echo "   Mientras tanto la vision corre en el CPU (mas lenta)."
  fi
  VENV="$HOME/.go2_nav/venv_vision"
  mkdir -p "$HOME/.go2_nav"
  [ -x "$VENV/bin/python" ] || python3 -m venv --system-site-packages "$VENV"
  # numpy 1.x: el mismo que usa ROS 2 (con numpy 2 los mensajes de ROS fallan)
  RESTR="$HOME/.go2_nav/restricciones.txt"
  printf 'numpy<2\nopencv-python<4.12\nopencv-python-headless<4.12\n' > "$RESTR"
  # sin la configuracion de pip del usuario (un extra-index "cpu" mete el PyTorch SIN CUDA)
  export PIP_CONFIG_FILE=/dev/null
  "$VENV/bin/pip" install -q --upgrade pip || true
  if nvidia-smi -L >/dev/null 2>&1 && \
     ! "$VENV/bin/python" -c "import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)" 2>/dev/null; then
    echo "   PyTorch CON CUDA para tu GPU (~3 GB, tarda un rato)"
    "$VENV/bin/pip" install -c "$RESTR" --force-reinstall torch torchvision \
      --index-url https://download.pytorch.org/whl/cu128 || echo "!! no pude instalar PyTorch con CUDA"
  fi
  if "$VENV/bin/pip" install -c "$RESTR" "ultralytics>=8.3,<9"; then
    set +u
    source "$WS/go2_env.sh" >/dev/null
    "$VENV/bin/python" -m go2_nav.vision --probar || echo "!! la prueba de la vision fallo (mira el error arriba)"
  else
    echo "!! no pude instalar PyTorch/YOLO (revisa internet). El resto de go2_nav funciona igual."
  fi
fi
echo
echo "LISTO. Abre una terminal NUEVA y:"
echo "  go2 sim                      # prueba sin perro (abre RViz)"
echo "  go2 lanzar rviz:=true        # con el perro por cable"
echo "  go2 explorar | go2 ver | go2 guardar A | go2 ir A"
echo "  go2 camara                   # lo que ve la camara (YOLO en la GPU)"
