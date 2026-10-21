#!/bin/bash
# Arranca la simulación completa dentro del contenedor:
#   1) genera la ciudad, 2) Gazebo (servidor sin ventana, dibujo por software),
#   3) puente Gazebo ↔ ROS 2, 4) núcleo del laboratorio.
# Si cualquiera de los tres procesos termina, el contenedor se reinicia solo.
source /opt/ros/jazzy/setup.bash

CFG=${LAB_CONFIG:-/lab/config/lab.yaml}
DATA=${LAB_DATA:-/data}

python3 /lab/gz/generar_mundo.py "$CFG" "$DATA" || exit 1

echo "[sim] Iniciando Gazebo (puede tardar ~30 s la primera vez)..."
gz sim -s -r --headless-rendering -v 2 "$DATA/ciudad.sdf" &
PID_GZ=$!
sleep 8

echo "[sim] Iniciando puente Gazebo <-> ROS 2..."
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$DATA/puente.yaml" &
PID_PUENTE=$!
sleep 2

python3 /lab/src/sim/nucleo_gz.py &
PID_NUCLEO=$!

terminar() {
  echo "[sim] Deteniendo..."
  kill -TERM $PID_NUCLEO $PID_PUENTE $PID_GZ 2>/dev/null
  wait
  exit 0
}
trap terminar TERM INT

wait -n $PID_GZ $PID_PUENTE $PID_NUCLEO
echo "[sim] Un proceso terminó inesperadamente; se reinicia el contenedor."
kill -TERM $PID_NUCLEO $PID_PUENTE $PID_GZ 2>/dev/null
exit 1
