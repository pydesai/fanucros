# fanuc_mqtt_bridge

`fanuc_mqtt_bridge` is a ROS 2 Python package that reads FANUC controller telemetry via RMI polling and publishes JSON payloads to an MQTT broker.

## Features

- Reads joints, status, extended status, IO, registers, and configured variables.
- Publishes category-specific JSON payloads to MQTT topics.
- Optional write actions guarded by `writes.enabled`.
- Reconnect handling for MQTT and FANUC RMI.

## Important backend note

This package expects a Python RMI backend module named `fanuc_rmi` that exposes `RMIConnection`.
That binding is not provided by this repository.
For local testing without hardware, set `robot.backend: sim` in config.

## MQTT topics

Using `publish.base_topic` (for example `fanuc/crx10ia_prod_01`):

- `<base_topic>/connection`
- `<base_topic>/joints`
- `<base_topic>/status`
- `<base_topic>/status_ext`
- `<base_topic>/io/digital`
- `<base_topic>/io/analog`
- `<base_topic>/io/group`
- `<base_topic>/registers/num`
- `<base_topic>/registers/pos`
- `<base_topic>/variables`
- `<base_topic>/errors`

## Run

### 1) Build

```bash
colcon build --packages-select fanuc_mqtt_bridge
source install/setup.bash
```

### 2) Configure

Edit:

- `fanuc_mqtt_bridge/config/bridge_config.yaml`

### 3) Launch

```bash
ros2 launch fanuc_mqtt_bridge fanuc_mqtt_bridge.launch.py
```

Or run directly:

```bash
ros2 run fanuc_mqtt_bridge fanuc_mqtt_bridge --ros-args -p config_file:=/absolute/path/to/bridge_config.yaml
```

## Docker

### Build

```bash
docker build -t fanuc-mqtt-bridge:local ./fanuc_mqtt_bridge
```

### Run with env vars

Example for an external host connecting to a FANUC controller and MQTT broker:

```bash
docker run --rm --network host \
  -e ROBOT_IP=192.168.1.100 \
  -e ROBOT_BACKEND=fanuc_rmi \
  -e MQTT_HOST=127.0.0.1 \
  -e MQTT_PORT=1883 \
  -e MQTT_USERNAME=fanuc \
  -e MQTT_PASSWORD=fanuc \
  -e PUBLISH_ROBOT_ID=crx10ia_prod_01 \
  -e PUBLISH_BASE_TOPIC=fanuc/crx10ia_prod_01 \
  fanuc-mqtt-bridge:local
```

If `BRIDGE_CONFIG_FILE` is set, the container uses that file directly.
Otherwise it starts from the template config and applies env var overrides.

Supported env vars:

- `ROBOT_IP`, `ROBOT_RMI_PORT`, `ROBOT_TIMEOUT_SEC`, `ROBOT_BACKEND`
- `MQTT_HOST`, `MQTT_PORT`, `MQTT_USERNAME`, `MQTT_PASSWORD`, `MQTT_CLIENT_ID`, `MQTT_KEEPALIVE_SEC`, `MQTT_QOS`, `MQTT_RETAIN`
- `PUBLISH_ROBOT_ID`, `PUBLISH_BASE_TOPIC`, `PUBLISH_RATE_HZ`, `PUBLISH_INCLUDE_TIMESTAMP`
- `TELEMETRY_JOINTS`, `TELEMETRY_STATUS`, `TELEMETRY_STATUS_EXT`
- `TELEMETRY_DIGITAL_INPUTS`, `TELEMETRY_DIGITAL_OUTPUTS`, `TELEMETRY_FLAGS`
- `TELEMETRY_ANALOG_INPUTS`, `TELEMETRY_ANALOG_OUTPUTS`
- `TELEMETRY_GROUP_INPUTS`, `TELEMETRY_GROUP_OUTPUTS`
- `TELEMETRY_NUM_REGISTERS`, `TELEMETRY_POS_REGISTERS`, `TELEMETRY_VARIABLES`
- `WRITES_ENABLED`, `WRITES_STARTUP_ACTIONS_JSON`, `WRITES_CYCLIC_ACTIONS_JSON`
- `BRIDGE_CONFIG_FILE`, `BRIDGE_CONFIG_TEMPLATE`, `BRIDGE_CONFIG_OUT`

Notes:

- List env vars use comma-separated values, for example: `TELEMETRY_DIGITAL_INPUTS=81,82,83`.
- `WRITES_STARTUP_ACTIONS_JSON` and `WRITES_CYCLIC_ACTIONS_JSON` must be JSON arrays.
- `fanuc_rmi` Python binding is not included in this repository. For real-controller polling (`ROBOT_BACKEND=fanuc_rmi`), provide a compatible `fanuc_rmi` module in your image/runtime. With `ROBOT_BACKEND=auto`, the bridge tries `fanuc_rmi` and falls back to simulator if unavailable.

## Config schema

See the example in `fanuc_mqtt_bridge/config/bridge_config.yaml`.

Required keys:

- `robot.ip`
- `mqtt.host`
- `publish.robot_id`

Optional key:

- `robot.backend`: `auto` (default), `fanuc_rmi`, or `sim`

## Testing

```bash
python3 -m pytest fanuc_mqtt_bridge/test -q
```
