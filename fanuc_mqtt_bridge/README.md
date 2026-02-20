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
