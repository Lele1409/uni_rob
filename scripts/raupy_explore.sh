#!/bin/bash
# raupy_explore.sh — start autonomous frontier exploration (SLAM + Nav2 + explorer +
# supervisor + RViz) on the laptop. THE ROBOT STARTS DRIVING ~15 s AFTER LAUNCH.
#   scripts/raupy_explore.sh                         # defaults: 600 s budget, map "bisasam_map"
#   scripts/raupy_explore.sh max_duration_s:=300.0 map_name:=room1
#   RAUPY_RVIZ_FPS=2 scripts/raupy_explore.sh        # RViz frame rate (default 5)
#   RAUPY_LASER_YAW=3.14159 scripts/raupy_explore.sh # override the URDF lidar yaw (see below)
#   scripts/raupy_explore.sh use_rviz:=false         # no RViz at all (lightest)
# Any extra arguments are passed to exploration.launch.py.
#
# The supervisor ends the run by itself (no frontiers left / map stopped growing / time up):
# it stops the explorer, cancels Nav2, stops the robot and saves the map to ~/raupy_maps.
#
# Only domain_bridge talks to the robot; everything else runs on a laptop-only domain
# (RAUPY_STACK_DOMAIN, default 57). Too many DDS participants over Wi-Fi saturated the link,
# and on Bisasam domain 30 is shared with another group's nodes as well.
#
# BISASAM BRANCH. Two differences from Raupy:
#   - the lidar is part of the robot's container stack, so there is nothing to start over SSH.
#     If /scan is missing, the stack on the robot is down.
#   - laser_yaw_fix is NOT hardcoded. Raupy's URDF mounted the lidar at yaw -90 deg while it
#     really sits at 180 deg, so every run passed 3.14159. Bisasam's URDF comes from a
#     different stack and may well be right. Run scripts/raupy_check_interfaces.sh: it derives
#     the true yaw from where the antennas block the scan and prints the value to set here.
#
# To stop early:  scripts/raupy_stop.sh, then scripts/raupy_save_map.sh, then Ctrl+C here.
# Emergency:      Ctrl+C here (robot halts within 0.5 s, unsaved map is lost),
#                 or lift the robot / power switch.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Own copy: raupy_env.sh unsets _ws_root when it's done.
rviz_src="$_ws_root/ros2_ws/src/raupy_exploration/rviz/exploration.rviz"
source "$_ws_root/scripts/raupy_env.sh"

if ! ros2 pkg prefix domain_bridge >/dev/null 2>&1; then
  echo "raupy_explore: domain_bridge missing: sudo apt install ros-jazzy-domain-bridge" >&2
  exit 1
fi

# Without /scan, SLAM never publishes a map and the whole run hangs silently waiting for it
# (seen on Raupy 2026-09-22), so check before anything starts moving.
if ! timeout 20 python3 - <<'EOF'
import sys, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init()
n = rclpy.create_node('explore_scan_check')
got = []
n.create_subscription(LaserScan, '/scan', lambda m: got.append(1), qos_profile_sensor_data)
for _ in range(60):
    if got:
        break
    rclpy.spin_once(n, timeout_sec=0.2)
sys.exit(0 if got else 1)
EOF
then
  echo "raupy_explore: no /scan on domain $ROS_DOMAIN_ID. Symptom if you start anyway: the" >&2
  echo "  supervisor waits for the first map forever and the costmaps log 'timestamp earlier" >&2
  echo "  than all the data in the transform cache' for frame 'map'." >&2
  echo "  Check the robot's containers: ssh $RAUPY_USER@$RAUPY_HOST docker ps" >&2
  echo "  Then run scripts/raupy_preflight.sh." >&2
  exit 1
fi

# The box filter needs laser_filters on the laptop; without it the launch would abort.
if ros2 pkg prefix laser_filters >/dev/null 2>&1; then
  filter=true
else
  filter=false
  echo "raupy_explore: laser_filters not installed, relaying /scan unfiltered to /scan_clean" >&2
fi

# RViz renders at 30 fps by default; on this VM (software OpenGL) that alone took ~2 cores
# and starved Nav2. Use a copy of the layout with a lower frame rate (RAUPY_RVIZ_FPS).
rviz_fps="${RAUPY_RVIZ_FPS:-5}"
rviz_cfg="$(mktemp --suffix=.rviz)"
sed "s/^\( *Frame Rate:\).*/\1 $rviz_fps/" "$rviz_src" > "$rviz_cfg"
if ! grep -q "Fixed Frame" "$rviz_cfg"; then
  echo "raupy_explore: could not build the RViz layout copy, using the original (30 fps)" >&2
  rviz_cfg="$rviz_src"
fi

# Empty unless RAUPY_LASER_YAW is set, so the launch file trusts the robot's own URDF.
yaw_arg=()
if [ -n "${RAUPY_LASER_YAW:-}" ]; then
  yaw_arg=("laser_yaw_fix:=$RAUPY_LASER_YAW")
  echo "raupy_explore: overriding the lidar yaw with $RAUPY_LASER_YAW rad"
fi

robot_domain="$RAUPY_ROBOT_DOMAIN"
raupy_use_stack_domain
echo "raupy_explore: robot domain $robot_domain <-> bridge <-> stack domain $ROS_DOMAIN_ID"
echo "raupy_explore: robot starts moving in ~15 s. Ctrl+C to abort."
exec ros2 launch raupy_exploration exploration.launch.py use_scan_filter:=$filter \
  use_bridge:=true robot_domain:="$robot_domain" rviz_config:="$rviz_cfg" \
  "${yaw_arg[@]}" "$@"
