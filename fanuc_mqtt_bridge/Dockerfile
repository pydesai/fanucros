# syntax=docker/dockerfile:1.7

FROM ros:humble-ros-base AS builder

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-paho-mqtt \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/fanuc_ws/src/fanuc_mqtt_bridge
COPY . /opt/fanuc_ws/src/fanuc_mqtt_bridge

WORKDIR /opt/fanuc_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --packages-select fanuc_mqtt_bridge


FROM ros:humble-ros-base AS runtime

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV BRIDGE_CONFIG_TEMPLATE=/opt/fanuc/config/bridge_config.yaml
ENV BRIDGE_CONFIG_OUT=/tmp/bridge_config.generated.yaml

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-paho-mqtt \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/fanuc_ws/install /opt/fanuc_ws/install
COPY config/bridge_config.yaml /opt/fanuc/config/bridge_config.yaml
COPY docker/entrypoint.sh /opt/fanuc/entrypoint.sh
COPY docker/entrypoint.py /opt/fanuc/entrypoint.py

RUN chmod +x /opt/fanuc/entrypoint.sh /opt/fanuc/entrypoint.py

ENTRYPOINT ["/opt/fanuc/entrypoint.sh"]
CMD []
