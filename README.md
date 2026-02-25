<!-- SPDX-FileCopyrightText: 2025 FANUC America Corp.
     SPDX-FileCopyrightText: 2025 FANUC CORPORATION

     SPDX-License-Identifier: Apache-2.0
-->
<!-- markdownlint-disable MD013 -->
# fanuc_driver

![FANUC ROS 2 Control Driver](/images/FANUC_ros2_ControlDriver.jpg "FANUC ROS 2 Control Driver")

## About

This repository hosts the source code of the FANUC ROS 2 Driver project, a ros2_control high-bandwidth streaming driver.
This project will allow users to develop a ROS 2 application to control a FANUC virtual or real robot.

## Installation

See the [FANUC ROS 2 Driver Documentation](https://fanuc-corporation.github.io/fanuc_driver_doc/) for instructions.

## fanuc_mqtt_bridge Quick Start

`fanuc_mqtt_bridge` reads FANUC telemetry and publishes JSON to MQTT topics.
Package path: `fanuc_mqtt_bridge/`.

### Prerequisites

- ROS 2 Humble available in your shell.
- MQTT broker available (local or remote).

### Option A: Run simulator with local ROS 2

1. Start MQTT broker:

```bash
docker run --rm -d --name fanuc-mqtt-sim --network host eclipse-mosquitto:2
```

2. Build and source the workspace:

```bash
colcon build --packages-select fanuc_mqtt_bridge
source install/setup.bash
```

3. Configure simulator backend in `fanuc_mqtt_bridge/config/bridge_config.yaml`:

- `robot.backend: sim`
- `mqtt.host: 127.0.0.1` (or your broker host)

4. Launch:

```bash
ros2 launch fanuc_mqtt_bridge fanuc_mqtt_bridge.launch.py
```

Or:

```bash
ros2 run fanuc_mqtt_bridge fanuc_mqtt_bridge --ros-args -p config_file:=/absolute/path/to/bridge_config.yaml
```

5. Stop broker when done (optional):

```bash
docker rm -f fanuc-mqtt-sim
```

### Option B: Run with Docker

1. Build image:

```bash
docker build -t fanuc-mqtt-bridge:local ./fanuc_mqtt_bridge
```

2. Run in simulator mode:

```bash
docker run --rm --network host \
  -e ROBOT_BACKEND=sim \
  -e ROBOT_IP=127.0.0.1 \
  -e MQTT_HOST=127.0.0.1 \
  -e MQTT_PORT=1883 \
  -e MQTT_CLIENT_ID=fanuc-mqtt-bridge-sim \
  -e PUBLISH_ROBOT_ID=crx10ia_sim_01 \
  fanuc-mqtt-bridge:local
```

3. Run against a real FANUC controller:

```bash
docker run --rm --network host \
  -e ROBOT_IP=192.168.1.100 \
  -e ROBOT_RMI_PORT=8193 \
  -e ROBOT_BACKEND=fanuc_rmi \
  -e MQTT_HOST=127.0.0.1 \
  -e MQTT_PORT=1883 \
  -e MQTT_USERNAME=fanuc \
  -e MQTT_PASSWORD=fanuc \
  -e PUBLISH_ROBOT_ID=crx10ia_prod_01 \
  -e PUBLISH_BASE_TOPIC=fanuc/crx10ia_prod_01 \
  fanuc-mqtt-bridge:local
```

If `fanuc_rmi` is not already installed in the image, mount a wheel and set:

```bash
-v /absolute/path/to/fanuc_rmi-<version>-py3-none-any.whl:/opt/fanuc/ext/fanuc_rmi.whl:ro \
-e FANUC_RMI_PIP_SPEC=/opt/fanuc/ext/fanuc_rmi.whl
```

Notes:

- `fanuc_rmi` Python binding is required for `ROBOT_BACKEND=fanuc_rmi` and is not provided by this repo.
- `ROBOT_RMI_PORT` is the preferred port variable; `ROBOT_PORT` is accepted as a legacy alias.
- On Docker Desktop (Windows/macOS), use `MQTT_HOST=host.docker.internal` to reach a broker on the host.
- If `BRIDGE_CONFIG_FILE` is set, the container uses that file directly.
- Full env-var reference is in `fanuc_mqtt_bridge/README.md`.

## Licensing

The original FANUC ROS 2 Driver source code and associated documentation
including these web pages are Copyright (C) 2025 FANUC America Corporation
and FANUC CORPORATION.

Any modifications or additions to source code or documentation
contributed to this project are Copyright (C) the contributor,
and should be noted as such in the comments section of the modified file(s).

FANUC ROS 2 Driver is licensed under
     [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)

Exceptions:

- The sockpp library is licensed under the terms of the [BSD 3-Clause License](https://opensource.org/license/BSD-3-Clause).

- The readwriterqueue library is licensed under the terms of
  the [Simplified BSD License](https://opensource.org/license/BSD-2-Clause).

- The reflect-cpp and yaml-cpp libraries are licensed under the
  terms of the [MIT License](https://opensource.org/license/mit).

Please see the LICENSE folder in the root directory for the full texts of these licenses.
