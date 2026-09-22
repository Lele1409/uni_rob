#!/bin/bash
# robot_preflight.sh — run on the laptop before every exploration session.
#   scripts/robot_preflight.sh
# 1. robot side (over SSH): microros/rosbot services, starts the lidar, clock offset
# 2. laptop side: /scan, /odometry/filtered and TF odom->base_link actually arrive here
# It does not move the robot.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/robot_env.sh" || exit 1

ok()   { echo "  [ OK ] $*"; }
warn() { echo "  [WARN] $*"; }
fail() { echo "  [FAIL] $*"; FAILED=1; }
FAILED=0
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $ROBOT_USER@$ROBOT_HOST"

echo "== Robot ($ROBOT_HOST)"
if ! $SSH true 2>/dev/null; then
  fail "SSH to $ROBOT_USER@$ROBOT_HOST failed"; exit 1
fi
for svc in microros rosbot; do
  state="$($SSH systemctl is-active "$svc" 2>/dev/null)"
  [ "$state" = active ] && ok "$svc.service active" || fail "$svc.service is '$state'"
done

# The lidar is off by default (rplidar.service, started via rosbot-lidar.sh).
if $SSH 'systemctl is-active --quiet rplidar' 2>/dev/null; then
  ok "lidar already running"
else
  echo "  starting lidar ..."
  $SSH 'rosbot-lidar.sh start' && ok "lidar started" || fail "rosbot-lidar.sh start failed"
fi

echo "== Laptop"
if ros2 pkg prefix laser_filters >/dev/null 2>&1; then
  ok "laser_filters installed (chassis box filter will be used)"
else
  warn "laser_filters missing: robot_explore.sh will relay /scan unfiltered."
  warn "  install with: sudo apt install ros-jazzy-laser-filters"
fi

if ros2 pkg prefix domain_bridge >/dev/null 2>&1; then
  ok "domain_bridge installed"
else
  fail "domain_bridge missing: sudo apt install ros-jazzy-domain-bridge"
fi

# Topic rates. Wi-Fi discovery can take a few seconds, so give it time.
hz() {  # hz <topic> <min_hz>
  local rate
  rate=$(timeout 12 ros2 topic hz "$1" --window 20 2>/dev/null \
         | awk '/average rate/ {r=$3} END {print r}')
  if [ -z "$rate" ]; then
    fail "$1: nothing received"
  elif python3 -c "import sys; sys.exit(0 if $rate >= $2 else 1)"; then
    ok "$1 at ${rate} Hz"
  else
    warn "$1 at only ${rate} Hz (expected >= $2)"
  fi
}
echo "  measuring topic rates (~12 s each) ..."
hz /scan 7
hz /odometry/filtered 8

# Clock offset + Wi-Fi latency, measured as the age of /scan stamps on arrival here.
# (Timing `ssh date` is useless: the SSH handshake dominates it.) Nav2/collision_monitor
# drop scans older than 2 s, SLAM waits 0.5 s for TF.
age=$(timeout 30 python3 - <<'EOF' 2>/dev/null
import rclpy, statistics
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('preflight_scan_age'); d = []
def cb(m):
    d.append((n.get_clock().now().nanoseconds
              - (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) / 1e9)
n.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)
while len(d) < 50:
    rclpy.spin_once(n, timeout_sec=1.0)
print(f'{statistics.median(d):.3f} {max(d):.3f}')
EOF
)
if [ -z "$age" ]; then
  fail "/scan age: could not measure"
else
  read -r med mx <<< "$age"
  if python3 -c "import sys; sys.exit(0 if abs($med) < 0.3 else 1)"; then
    ok "/scan age median ${med}s, max ${mx}s (clocks in sync)"
  else
    warn "/scan age median ${med}s, max ${mx}s: clocks out of sync (check timedatectl on both)"
  fi
fi

if timeout 10 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q Translation; then
  ok "TF odom -> base_link available"
else
  fail "TF odom -> base_link not received"
fi

echo
if [ "$FAILED" = 0 ]; then
  echo "Preflight passed. Next: scripts/robot_explore.sh"
else
  echo "Preflight FAILED, fix the items above first."
  exit 1
fi
