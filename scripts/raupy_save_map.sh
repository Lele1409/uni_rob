#!/bin/bash
# raupy_save_map.sh — save the current map while raupy_explore.sh is still running.
#   scripts/raupy_save_map.sh [name]        # default name: raupy_map
# Writes ~/raupy_maps/<name>_<YYYYmmdd_HHMMSS>.{yaml,pgm} (Nav2 map, loadable by map_server)
# and .{posegraph,data} (slam_toolbox pose graph, to continue mapping later).
# Needs slam_toolbox and map_saver alive: run it BEFORE Ctrl+C on the launch.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/raupy_env.sh" >/dev/null
raupy_use_stack_domain  # the stack runs on its own domain, behind domain_bridge

name="${1:-raupy_map}"
dir="$HOME/raupy_maps"
base="$dir/${name}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$dir"

# Same format as the supervisor: trinary, 0.25/0.65 thresholds.
timeout 20 ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: /map, map_url: '$base', image_format: pgm, map_mode: trinary,
    free_thresh: 0.25, occupied_thresh: 0.65}" | tail -1

timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$base'}" | tail -1

ls -l "$base".* 2>/dev/null || { echo "raupy_save_map: no files written" >&2; exit 1; }
