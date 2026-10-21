# Prácticas de laboratorio · sensores del vehículo autónomo

Todos los datos vienen del simulador en tiempo real. Ejecuta con `python3 <archivo>`;
detén con `Ctrl + C`. Edita con `nano <archivo>`.

| # | Archivo | Sensor / tema | Tópico |
|---|---|---|---|
| 1 | leer_imu.py | IMU, sesgo del giroscopio | /auto/imu/data |
| 2 | leer_gnss.py | GNSS a coordenadas locales y error | /auto/gnss/fix |
| 3 | leer_lidar.py | Nube de puntos por sectores | /auto/lidar/points |
| 4 | leer_camara.py | Imagen cruda e intrínsecos; guarda PNG | /auto/camara_frontal/image |
| 5 | leer_ultrasonido.py | 8 sensores de estacionamiento | /auto/ultrasonido_*/range |
| 6 | leer_radar.py | Distancia, velocidad relativa y TTC | /auto/radar_frontal/points |
| 7 | leer_encoders.py | Odometría por encoders vs posición real | /auto/encoders/joint_states |
| 8 | sniffer_can.py | Bus CAN crudo y decodificado (DBC) | /auto/can/rx |
| 9 | lector_obd.py | Diagnóstico OBD-II (modo 01 y 03) | /auto/can/rx |
| 10 | procesar_camara.py | Filtra la cámara (bordes, gris, umbral, carril) y **publica tu tópico** | /alumnos/<tu_nombre>/camara_procesada |
| – | eco_crudo.py | Cualquier tópico en crudo | (el que indiques) |

**Publicar tus propios tópicos:** todo lo que publiques bajo `/alumnos/<tu_nombre>/...`
aparece en la página (Terminal → Datos crudos); si es una imagen, se ve como video.
Tu nombre de carpeta está en la variable `$LAB_ALUMNO`. Ejemplo:

    python3 procesar_camara.py carril
    ros2 topic pub /alumnos/$LAB_ALUMNO/saludo std_msgs/msg/String "{data: hola}"

Comandos útiles de ROS 2:

    ros2 topic list                              # todos los tópicos
    ros2 topic echo /auto/imu/data               # mensajes en crudo
    ros2 topic hz /auto/lidar/points             # frecuencia real
    ros2 topic info /auto/can/rx -v              # quién publica, QoS
    ros2 interface show sensor_msgs/msg/Range    # estructura del mensaje
    ros2 run tf2_ros tf2_echo map auto/base_link # pose real del vehículo

Las fallas que inyecta el profesor (P0118, C0035, B1A01…) también aparecen aquí:
úsalas para practicar diagnóstico y detección de datos anómalos.
