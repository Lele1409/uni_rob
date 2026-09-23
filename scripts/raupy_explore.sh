#!/bin/bash
# raupy_explore.sh — start autonomous frontier exploration (SLAM + Nav2 + explorer +
# supervisor + RViz) on the laptop. THE ROBOT STARTS DRIVING ~15 s AFTER LAUNCH.
#   scripts/raupy_explore.sh                         # defaults: 600 s budget, map "raupy_map"
#   scripts/raupy_explore.sh max_duration_s:=300.0 map_name:=room1
#   RAUPY_RVIZ_FPS=2 scripts/raupy_explore.sh        # RViz frame rate (default 5)
#   scripts/raupy_explore.sh use_rviz:=false         # no RViz at all (lightest)
# Any extra arguments are passed to exploration.launch.py.
#
# The supervisor ends the run by itself (no frontiers left / map stopped growing / time up):
# it stops the explorer, cancels Nav2, stops the robot and saves the map to ~/raupy_maps.
#
# Only domain_bridge talks to the robot; everything else runs on a laptop-only domain
# (RAUPY_STACK_DOMAIN, default 57). Too many DDS participants over Wi-Fi saturated the link.
#
# laser_yaw_fix: Raupy's URDF mounts the lidar at yaw -90 deg, measured it is 180 deg.
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

# The lidar is off by default (rplidar.service). Without /scan, SLAM never publishes a map
# and the whole run hangs silently waiting for it (seen 2026-09-22), so start it here when
# the preflight was skipped.
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
  echo "raupy_explore: no /scan, starting the lidar on the robot ..." >&2
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$RAUPY_USER@$RAUPY_HOST" 'rosbot-lidar.sh start' >&2 \
    || { echo "raupy_explore: could not start the lidar, run scripts/raupy_preflight.sh" >&2; exit 1; }
  sleep 5
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

robot_domain="$RAUPY_ROBOT_DOMAIN"
raupy_use_stack_domain
echo "raupy_explore: robot domain $robot_domain <-> bridge <-> stack domain $ROS_DOMAIN_ID"
echo "raupy_explore: robot starts moving in ~15 s. Ctrl+C to abort."
exec ros2 launch raupy_exploration exploration.launch.py use_scan_filter:=$filter \
  use_bridge:=true robot_domain:="$robot_domain" rviz_config:="$rviz_cfg" \
  laser_yaw_fix:=3.14159 "$@"
