#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts read unset variables in a few code paths.
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source /opt/fanuc_ws/install/setup.bash
set -u

if [[ $# -gt 0 ]]; then
  exec "$@"
fi

exec python3 /opt/fanuc/entrypoint.py
