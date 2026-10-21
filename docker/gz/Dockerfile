# Imagen del laboratorio: ROS 2 Jazzy + Gazebo Harmonic (servidor sin ventana)
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GZ_PARTITION=lab_ads

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-can-msgs \
        ros-jazzy-ackermann-msgs \
        ros-jazzy-tf2-ros \
        ros-jazzy-sensor-msgs-py \
        python3-pip python3-numpy python3-opencv python3-flask python3-yaml \
        libegl1 libegl-mesa0 libgl1-mesa-dri mesa-utils \
        ttyd nano vim less htop \
    && rm -rf /var/lib/apt/lists/*

# Bus CAN: codificar/decodificar con el archivo DBC
RUN pip3 install --no-cache-dir --break-system-packages "cantools<40"

WORKDIR /lab
