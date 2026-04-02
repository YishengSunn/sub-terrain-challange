#!/usr/bin/env bash

set -euo pipefail

ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

set +u
source "${ws_root}/install/setup.bash"
set -u

# Avoid FastDDS shared-memory transport lock conflicts on some systems (overridable).
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

rviz_config="$(ros2 pkg prefix mapping)/share/mapping/rviz/default.rviz"
exec ros2 launch simulation simulation.launch.py rviz_config:="${rviz_config}" "$@"
