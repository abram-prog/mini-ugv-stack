# ---------- build stage ----------
FROM ros:humble-ros-base AS builder
SHELL ["/bin/bash", "-lc"]

# базовые инструменты
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    build-essential cmake git \
 && rm -rf /var/lib/apt/lists/*

# рабочее пространство
WORKDIR /ros2_ws
COPY src ./src

# зависимости ROS (apt через rosdep)
RUN rosdep update || true
RUN rosdep install --rosdistro humble --from-paths src -y --ignore-src || true

# python-зависимости для моста (нужны в build для генерации, тестов и т.п.)
RUN pip install --no-cache-dir paho-mqtt fastapi uvicorn

# сборка
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# ---------- runtime stage ----------
FROM ros:humble-ros-base
SHELL ["/bin/bash", "-lc"]
WORKDIR /ros2_ws

# сюда копируем собранный воркспейс
COPY --from=builder /ros2_ws /ros2_ws

# python-зависимости нужны И в рантайме
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/* && \
    pip install --no-cache-dir paho-mqtt fastapi uvicorn

# удобное окружение
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# ... всё как было выше ...
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42
ENV PYTHONUNBUFFERED=1

# Никакого ENTRYPOINT
# Оставим дефолтную команду просто «ждать», чтобы контейнер не падал,
# её переопределим в compose.
CMD ["bash","-lc","tail -f /dev/null"]
