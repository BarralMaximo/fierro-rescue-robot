# Fierro listo para correr: ROS 2 Humble, las dependencias de package.xml y el paquete compilado.
#
#   docker build -t fierro .
#   docker run --rm fierro python3 -m pytest -q src/fierro/test                                   # tests
#   docker run --rm --net=host fierro ros2 launch fierro fierro.launch.py victims:=4              # misión
#
# La misión necesita el robot o el simulador en la misma red (--net=host comparte la red de ROS 2).

# La misma imagen base que usa el CI del Challenge
FROM ros:humble-ros-base-jammy@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19

SHELL ["/bin/bash", "-c"]
WORKDIR /ws

# Primero solo package.xml: la capa de dependencias se reusa mientras no cambien. Los paquetes de Ubuntu salen del
# mirror de Azure, el que usan los runners de GitHub, porque desde ahí archive.ubuntu.com corta conexiones a mitad de
# la descarga; y apt reintenta lo que falle.
COPY package.xml src/fierro/package.xml
RUN sed -i 's|http://archive.ubuntu.com/|http://azure.archive.ubuntu.com/|' /etc/apt/sources.list \
    && echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries \
    && apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

COPY . src/fierro
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select fierro \
    && sed -i 's|^exec "\$@"|source /ws/install/setup.bash\nexec "$@"|' /ros_entrypoint.sh

CMD ["ros2", "launch", "fierro", "fierro.launch.py"]
