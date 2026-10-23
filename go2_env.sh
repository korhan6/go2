# go2_nav: entorno ROS 2 para hablar con el Go2 por el cable (DDS directo, como la mochila EDU)
source /opt/ros/__DISTRO__/setup.bash
[ -f "__WS__/install/setup.bash" ] && source "__WS__/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # el Go2 usa CycloneDDS 0.10.x
export ROS_DOMAIN_ID=0                         # el dominio del perro

# la interfaz del cable: la que tenga una IP 192.168.123.x (el perro es 192.168.123.161)
export GO2_IF=$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')
if [ -n "$GO2_IF" ]; then
  # buffer de recepcion grande: las nubes del lidar (~300 KB) llegan en muchos pedazos UDP y,
  # con el buffer chico, se pierde uno y se pierde la nube entera (lidar a 0-8 Hz)
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_IF\" priority=\"default\" multicast=\"default\"/></Interfaces></General><Internal><SocketReceiveBufferSize min=\"default\" max=\"10MB\"/></Internal></Domain></CycloneDDS>"
  echo "go2_nav: perro por $GO2_IF (DDS directo)"
else
  unset CYCLONEDDS_URI
  echo "go2_nav: sin cable al perro (192.168.123.x). Para simular: go2 sim"
fi
