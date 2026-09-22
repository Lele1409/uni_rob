#!/bin/bash
# robot_explore.sh — start autonomous frontier exploration (SLAM + Nav2 + explorer +
# supervisor + RViz) on the laptop. THE ROBOT STARTS DRIVING ~15 s AFTER LAUNCH.
#   scripts/robot_explore.sh                         # defaults: 600 s budget, map "<robot>_map"
#   scripts/robot_explore.sh max_duration_s:=300.0 map_name:=room1
#   RVIZ_FPS=2 scripts/robot_explore.sh        # RViz frame rate (default 5)
#   scripts/robot_explore.sh use_rviz:=false         # no RViz at all (lightest)
# Any extra arguments are passed to exploration.launch.py.
#
# The supervisor ends the run by itself (no frontiers left / map stopped growing / time up):
# it stops the explorer, cancels Nav2, stops the robot and saves the map to ~/raupy_maps.
#
# Only domain_bridge talks to the robot; everything else runs on a laptop-only domain
# (STACK_DOMAIN, default 57). Too many DDS participants over Wi-Fi saturated the link.
#
# laser_yaw_fix: Raupy's URDF mounts the lidar at yaw -90 deg, measured it is 180 deg
# (LASER_YAW_FIX in robot_env.sh, empty = trust the URDF).
# Robot: scripts/robot_select.sh raupy|bisasam, or ROBOT=bisasam scripts/robot_explore.sh
#
# To stop early:  scripts/robot_stop.sh, then scripts/robot_save_map.sh, then Ctrl+C here.
# Emergency:      Ctrl+C here (robot halts within 0.5 s, unsaved map is lost),
#                 or lift the robot / power switch.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Own copy: robot_env.sh unsets _ws_root when it's done.
rviz_src="$_ws_root/ros2_ws/src/raupy_exploration/rviz/exploration.rviz"
source "$_ws_root/scripts/robot_env.sh" || exit 1

if ! ros2 pkg prefix domain_bridge >/dev/null 2>&1; then
  echo "robot_explore: domain_bridge missing: sudo apt install ros-jazzy-domain-bridge" >&2
  exit 1
fi

# The box filter needs laser_filters on the laptop; without it the launch would abort.
if ros2 pkg prefix laser_filters >/dev/null 2>&1; then
  filter=true
else
  filter=false
  echo "robot_explore: laser_filters not installed, relaying /scan unfiltered to /scan_clean" >&2
fi

# RViz renders at 30 fps by default; on this VM (software OpenGL) that alone took ~2 cores
# and starved Nav2. Use a copy of the layout with a lower frame rate (RVIZ_FPS).
rviz_fps="${RVIZ_FPS:-5}"
rviz_cfg="$(mktemp --suffix=.rviz)"
sed "s/^\( *Frame Rate:\).*/\1 $rviz_fps/" "$rviz_src" > "$rviz_cfg"
if ! grep -q "Fixed Frame" "$rviz_cfg"; then
  echo "robot_explore: could not build the RViz layout copy, using the original (30 fps)" >&2
  rviz_cfg="$rviz_src"
fi

robot_domain="$ROBOT_DOMAIN"
use_stack_domain
echo "robot_explore: $ROBOT, robot domain $robot_domain <-> bridge <-> stack domain $ROS_DOMAIN_ID"
echo "robot_explore: robot starts moving in ~15 s. Ctrl+C to abort."
exec ros2 launch raupy_exploration exploration.launch.py use_scan_filter:=$filter \
  use_bridge:=true robot_domain:="$robot_domain" rviz_config:="$rviz_cfg" \
  laser_yaw_fix:="$LASER_YAW_FIX" map_name:="${ROBOT}_map" "$@"
