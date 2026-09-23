#!/bin/bash
# raupy_preflight.sh — run on the laptop before every exploration session.
#   scripts/raupy_preflight.sh
# 1. robot side (over SSH): the stack's containers are up, and nothing is obviously wedged
# 2. laptop side: /scan, /odometry/filtered and TF odom->base_link actually arrive here
# It does not move the robot.
#
# BISASAM BRANCH. Bisasam runs its stack as Docker containers, not as the microros/rosbot
# systemd units Raupy used, and it has no rplidar.service to start -- its lidar container
# comes up with the rest. The robot-side checks are therefore best-effort and never fatal on
# their own: what counts is whether the data arrives, which the laptop-side checks measure.
#
# Run scripts/raupy_check_interfaces.sh once before the first session of the day; it verifies
# the topic names, types and frames that this preflight takes for granted.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/raupy_env.sh"

ok()   { echo "  [ OK ] $*"; }
warn() { echo "  [WARN] $*"; }
fail() { echo "  [FAIL] $*"; FAILED=1; }
FAILED=0
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $RAUPY_USER@$RAUPY_HOST"

echo "== Robot ($RAUPY_HOST)"
if ! $SSH true 2>/dev/null; then
  warn "SSH to $RAUPY_USER@$RAUPY_HOST failed, skipping the robot-side checks."
  warn "  The laptop-side checks below are what actually decide whether a run can start."
else
  # Bisasam is shared with another group, so report what is running rather than insisting
  # on a fixed set of container names.
  running="$($SSH 'docker ps --format "{{.Names}}\t{{.Status}}"' 2>/dev/null)"
  if [ -z "$running" ]; then
    fail "no containers are running (or $RAUPY_USER cannot reach docker)."
    fail "  Bring the stack up on the robot before starting here."
  else
    ok "$(echo "$running" | wc -l) container(s) running:"
    echo "$running" | sed 's/^/         /'
    # A container that keeps restarting looks "up" in a casual glance but delivers nothing.
    if echo "$running" | grep -qi 'restarting'; then
      fail "at least one container is restarting in a loop, see above."
    fi
  fi
fi

echo "== Laptop"
if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  ok "rmw_cyclonedds_cpp installed (matches Bisasam's DDS)"
else
  fail "rmw_cyclonedds_cpp missing: sudo apt install ros-jazzy-rmw-cyclonedds-cpp"
  fail "  Without it the laptop sees an empty topic list, not an error."
fi

if ros2 pkg prefix laser_filters >/dev/null 2>&1; then
  ok "laser_filters installed (chassis box filter will be used)"
else
  warn "laser_filters missing: raupy_explore.sh will relay /scan unfiltered."
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

# The MCU can come up half dead: the driver activates, topics flow, but the firmware repeats
# one frozen sample forever (seen on Raupy 2026-09-22: identical IMU quaternion/acceleration,
# encoders stuck, motors silent). The EKF then diverges on those constant inputs and invents
# motion, so the map drifts away mid-run. A live IMU always jitters, so identical samples are
# a reliable giveaway. Same CORE2 board on Bisasam, so the same failure is possible; its stack
# may publish the IMU elsewhere, hence "no data" is only a warning here.
imu=$(timeout 20 python3 - <<'EOF' 2>/dev/null
import rclpy
from sensor_msgs.msg import Imu
rclpy.init(); n = rclpy.create_node('preflight_imu_alive'); s = []
n.create_subscription(Imu, '/imu/data', lambda m: s.append(
    (m.orientation.z, m.angular_velocity.z, m.linear_acceleration.x)), 10)
for _ in range(100):
    if len(s) >= 50:
        break
    rclpy.spin_once(n, timeout_sec=0.2)
print(f'{len(s)} {len(set(s))}')
EOF
)
read -r n_imu n_uniq <<< "${imu:-0 0}"
if [ "${n_imu:-0}" -lt 10 ]; then
  warn "IMU: no /imu/data. Check the topic name with scripts/raupy_check_interfaces.sh;"
  warn "  until then a frozen MCU would only show up in raupy_drive_test.sh."
elif [ "${n_uniq:-0}" -le 1 ]; then
  fail "IMU frozen: $n_imu identical samples. The MCU is half dead (encoders and motors too)."
  fail "  Power-cycle the robot, then check with scripts/raupy_drive_test.sh."
else
  ok "IMU alive ($n_uniq distinct samples out of $n_imu)"
fi

echo
if [ "$FAILED" = 0 ]; then
  echo "Preflight passed. Next: scripts/raupy_drive_test.sh, then scripts/raupy_explore.sh"
else
  echo "Preflight FAILED, fix the items above first."
  exit 1
fi
