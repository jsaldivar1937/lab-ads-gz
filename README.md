# Laboratorio Virtual de Conducción Autónoma (ADS) · versión Gazebo

Simulador para el diplomado en línea: **Gazebo Harmonic + ROS 2 Jazzy + visualizador web**,
todo dentro de Docker (ya no se usa CARLA ni ningún programa nativo en Windows).
Un vehículo con todos sus sensores en una ciudad con tráfico; **todos los alumnos lo ven**,
**solo conduce quien autorice el profesor**, y cada alumno tiene una **terminal** para
leer los sensores en crudo y hacer prácticas.

```
 WINDOWS 10 + Docker Desktop
 ┌──────────────────────────── red "lab" (dominio ROS 7) ───────────────────────────┐
 │ sim       Gazebo (ciudad, autos, sensores) + puente gz↔ROS 2 + núcleo:           │
 │           dinámica, autopiloto, tráfico, ultrasonido, radar, encoders, ECU/CAN    │
 │ web       Flask: página de alumnos y profesor, video, datos crudos               │
 │ puente_practicas  copia de SOLO LECTURA de los sensores → dominio 8 ─────────┐    │
 └───────────────────────────────────────────────────────────────────────────────│───┘
 ┌──────────────────────────── red "practicas" (dominio ROS 8) ──────────────────▼───┐
 │ terminal  ttyd: una terminal bash con ROS 2 por alumno (no puede mover el auto)   │
 └───────────────────────────────────────────────────────────────────────────────────┘
 nginx :8080  →  /  (alumnos) · /profesor · /terminal/          → ngrok para internet
```

## 1. Instalación (una sola vez)

Requisitos: Docker Desktop con WSL2 (ya instalado). **CARLA ya no se necesita.**

1. Descomprime este proyecto en `E:\lab-ads-gz`.
2. *(Opcional)* cambia usuario/clave del profesor en `.env` (por defecto `profesor` / `profesor`).
3. Construye la imagen (tarda de 10 a 20 minutos la primera vez; descarga ~2 GB):
   ```powershell
   cd E:\lab-ads-gz
   docker compose build
   ```

## 2. Uso en cada sesión

Doble clic en `windows\iniciar_laboratorio.bat`, o en PowerShell:
```powershell
cd E:\lab-ads-gz
docker compose up -d
docker compose logs -f sim        # espera "[nucleo] Listo." y sal con Ctrl+C
```
- Profesor: <http://localhost:8080/profesor>
- Alumnos: <http://localhost:8080/>
- Internet: `E:\ngrok\ngrok.exe http 8080`
- Detener: `windows\detener_laboratorio.bat` o `docker compose down`

## 3. La ciudad

No existe un mapa urbano mantenido para el Gazebo actual (el *citysim* de Open Robotics
era para Gazebo Classic, ya sin soporte), así que la ciudad se **genera** al arrancar
(`gz/generar_mundo.py`), ligera para que Gazebo la dibuje sin GPU dentro de Docker:

- Cuadrícula de 3×3 calles de doble sentido (120 × 120 m), carriles de 3.5 m,
  línea central amarilla, bordes blancos y líneas de alto en cada cruce.
- 12 edificios, banquetas, señales de alto y de límite de velocidad.
- Estacionamiento con cajones y autos estacionados (prácticas de ultrasonido).
- 5 autos de tráfico que circulan solos por distintas rutas y ceden el paso.
- Coordenadas geográficas reales: el origen del GNSS está en la UAZ, Zacatecas.

Parámetros en `config/lab.yaml` → `ciudad` y `trafico`.

## 4. Pestañas de la página

| Pestaña | Contenido |
|---|---|
| **Conducción** | Cámara, LiDAR/radar 3D, mapa, telemetría y mandos (con permiso) |
| **Sensores** | Cámara adicional (profundidad, segmentación semántica o flujo óptico; la enciende el profesor), ultrasonido, IMU, GNSS, radar, LiDAR, encoders, estado y Hz de cada sensor |
| **ECU / CAN** | Tablero de la ECU, testigo MIL, DTC, mensajes CAN decodificados con el DBC, tramas en vivo, descarga del DBC; el profesor inyecta fallas |
| **Terminal** | Terminal bash con ROS 2 para cada alumno, lista de comandos y prácticas (clic para copiar) y **visor de datos crudos** de cualquier tópico |

## 5. Terminal y prácticas

Cada alumno tiene su carpeta personal `/trabajo/alumnos/<su_nombre>` con copias de las
prácticas (`practicas/`). En **Mis archivos** (pestaña Terminal) sube su propio código
(arrastrar o botón, máx. 10 MB) y lo corre en la terminal: `python3 mi_codigo.py`.
También puede descargar lo que genere (por ejemplo, las imágenes de `leer_camara.py`). Los alumnos **leen todos los sensores pero no pueden mover el vehículo**:
la terminal está en otra red y otro dominio ROS, alimentada por un puente de solo lectura.

| Práctica | Tema |
|---|---|
| `leer_imu.py` | IMU cruda y sesgo del giroscopio |
| `leer_gnss.py` | GNSS → coordenadas locales y error contra la posición real |
| `leer_lidar.py` | Nube de puntos por sectores |
| `leer_camara.py` | Imagen cruda, intrínsecos K, guarda PNG |
| `leer_ultrasonido.py` | 8 sensores de estacionamiento |
| `leer_radar.py` | Distancia, velocidad relativa y tiempo a colisión |
| `leer_encoders.py` | Odometría por encoders vs posición real |
| `sniffer_can.py` | Bus CAN crudo y decodificado con DBC |
| `lector_obd.py` | OBD-II modo 01 y modo 03 |
| `procesar_camara.py` | Filtra la cámara y publica `/alumnos/<nombre>/camara_procesada` |
| `eco_crudo.py` | Cualquier tópico en crudo |

Lo que un alumno publique bajo `/alumnos/<su_nombre>/...` sube a la página (Datos crudos;
las imágenes se ven como video). El puente solo deja pasar ese prefijo (máx. 15 Hz,
4 MB por mensaje), así que desde la terminal no se puede mover el vehículo.

## 6. Tópicos ROS 2 (iguales que en la versión CARLA)

| Tópico | Tipo |
|---|---|
| `/auto/camara_frontal/image` (+ `camera_info`) | `sensor_msgs/Image` rgb8 |
| `/auto/camara_profundidad/image` · `/auto/camara_segmentacion/image`, `/labels` · `/auto/camara_flujo/image`, `/flow` | bajo demanda |
| `/auto/lidar/points` | `sensor_msgs/PointCloud2` (x, y, z, intensity) |
| `/auto/radar_frontal/points` | `sensor_msgs/PointCloud2` (x, y, z, velocity) |
| `/auto/gnss/fix` · `/auto/imu/data` | `NavSatFix` · `Imu` |
| `/auto/ultrasonido_<pos>/range` (8) | `sensor_msgs/Range` (inf = libre) |
| `/auto/encoders/joint_states` · `/ticks` | `JointState` · `Int32MultiArray` |
| `/auto/ground_truth/odom` | `nav_msgs/Odometry` |
| `/auto/can/rx` · `/auto/ecu/estado` | `can_msgs/Frame` · JSON |
| `/auto/vehiculo/estado` · `/auto/eventos` | JSON |
| `/auto/cmd/ackermann` · `/cmd/directo` · `/cmd/autopilot` · `/reset` | comandos (solo dominio 7) |

Qué es nativo de Gazebo y qué está emulado: cámaras, profundidad, segmentación, LiDAR,
IMU, GNSS y contacto son sensores de Gazebo; el flujo óptico se calcula de la cámara
(OpenCV); ultrasonido, radar y encoders se calculan con la geometría de la ciudad y el
movimiento real del vehículo.

## 7. Rendimiento

Gazebo dibuja por **software** (sin GPU) dentro de Docker en Windows. Si el factor de
tiempo real que muestra la página baja de ~0.8:
- baja `horizontal_samples` o `channels` del LiDAR,
- baja la resolución de `camara_frontal`,
- baja `trafico.vehiculos`.
Después: `docker compose restart sim`.

## 8. Solución de problemas

| Síntoma | Qué hacer |
|---|---|
| La página dice "Esperando el mapa" | `docker compose logs sim`: Gazebo tarda ~30 s en arrancar |
| No hay video ni LiDAR | `docker compose logs sim`; busca errores de "render" u "ogre" |
| La terminal no abre | `docker compose logs terminal` |
| Los alumnos no ven datos en la terminal | `docker compose logs puente_practicas` |
| Usuario o clave del profesor incorrectos | Revisa `.env`; después `docker compose up -d` |
