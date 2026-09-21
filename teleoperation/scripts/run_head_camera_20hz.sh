#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
# Tested Gemini435Le RGB+depth profile. This script sends no head motor commands.
exec ros2 launch orbbec_camera gemini435_le.launch.py \
  preset_resolution_config:='640,400,1,1' \
  color_width:=640 color_height:=400 color_fps:=20 \
  depth_width:=640 depth_height:=400 depth_fps:=20 \
  enable_point_cloud:=false enable_colored_point_cloud:=false
