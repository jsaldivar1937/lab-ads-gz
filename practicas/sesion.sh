#!/bin/bash
# Sesión de terminal: entra a la carpeta personal del alumno (la misma donde
# la página guarda los archivos que sube). Las prácticas se copian sin
# sobrescribir lo que el alumno ya modificó.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=8
if [[ "$1" =~ ^[a-z0-9_]{1,40}$ ]]; then
  export LAB_ALUMNO=$1
  DIR=/trabajo/alumnos/$1
else
  export LAB_ALUMNO=anonimo
  DIR=/trabajo/sesion_$(date +%m%d_%H%M%S)_$RANDOM
fi
mkdir -p "$DIR"
cp -n /lab/practicas/*.py /lab/practicas/LEEME.md "$DIR"/ 2>/dev/null
rm -f "$DIR/generar_puente.py" "$DIR/puente_lectura.py"
cd "$DIR"
cat <<'BANNER'

  ┌──────────────────────────────────────────────────────────────┐
  │  Laboratorio Virtual ADS · Terminal de prácticas (ROS 2)     │
  │                                                              │
  │  ros2 topic list                  ver todos los tópicos      │
  │  ros2 topic echo /auto/imu/data   ver un sensor en crudo     │
  │  python3 leer_imu.py              ejecutar una práctica      │
  │  ls                               ver tus archivos           │
  │  python3 mi_codigo.py             correr un archivo subido   │
  │  python3 procesar_camara.py       publica tu propio tópico   │
  │  cat LEEME.md                     lista de prácticas         │
  │                                                              │
  │  Puedes leer todos los sensores; el vehículo solo lo mueve   │
  │  quien tenga permiso del profesor en la página.              │
  └──────────────────────────────────────────────────────────────┘

BANNER
echo "  Tu carpeta: $DIR"
echo "  Tus tópicos: /alumnos/$LAB_ALUMNO/...  (se ven en la página → Datos crudos)"
echo
exec bash --rcfile <(echo "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=8 LAB_ALUMNO=$LAB_ALUMNO; PS1='\[\e[33m\]ads\[\e[0m\]:\w\$ '")
