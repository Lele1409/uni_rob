#!/bin/bash
# raupy_check_interfaces.sh — verify every assumption this branch makes about Bisasam.
#   scripts/raupy_check_interfaces.sh
# Read-only: it does not move the robot and does not start or stop anything.
#
# Run this ONCE on the first Bisasam session, before raupy_preflight.sh. The configs on this
# branch were adapted from the 2026-09-22 inspection notes (Humble in Docker, CycloneDDS,
# domain 30, plain Twist on /cmd_vel, no ToF topics) plus everything measured on Raupy's
# identical chassis. This script checks each of those against the robot and, where something
# differs, names the file and the value to change.
#
# It talks to the ROBOT's domain directly (no bridge), so a couple of extra DDS participants
# are on the Wi-Fi for its duration. Don't run it while an exploration run is going.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/raupy_env.sh"

ok()   { echo "  [ OK ] $*"; }
warn() { echo "  [WARN] $*"; }
bad()  { echo "  [DIFF] $*"; DIFFS=$((DIFFS + 1)); }
DIFFS=0

echo
echo "== Robot side ($RAUPY_HOST)"
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $RAUPY_USER@$RAUPY_HOST"
if $SSH true 2>/dev/null; then
  ok "SSH works as $RAUPY_USER"
  echo "  containers:"
  $SSH 'docker ps --format "    {{.Names}}  ({{.Image}})  {{.Status}}"' 2>/dev/null \
    || warn "  could not list containers (is $RAUPY_USER in the docker group?)"
  echo "  ROS env inside the containers:"
  $SSH 'docker ps -q | xargs -r docker inspect --format "{{range .Config.Env}}{{println .}}{{end}}"' \
    2>/dev/null | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_DISTRO)=' | sort -u | sed 's/^/    /'
else
  warn "SSH to $RAUPY_USER@$RAUPY_HOST failed. Everything below still works over DDS,"
  warn "  but raupy_preflight.sh and raupy_mcu_diag.sh need SSH."
fi

echo
echo "== Topics on domain $ROS_DOMAIN_ID (RMW $RMW_IMPLEMENTATION)"
# Discovery over Wi-Fi is slow, and on a shared domain the list is long.
mapfile -t topics < <(timeout 20 ros2 topic list 2>/dev/null | sort)
if [ "${#topics[@]}" -eq 0 ]; then
  echo "  [FAIL] no topics at all. Either the domain is wrong, or the RMW does not match"
  echo "         (Bisasam uses CycloneDDS; a Fast DDS laptop sees an empty list, not an error)."
  exit 1
fi
ok "${#topics[@]} topics visible"
printf '    %s\n' "${topics[@]}"

topic_type() { timeout 10 ros2 topic type "$1" 2>/dev/null | head -1; }
has_topic() { printf '%s\n' "${topics[@]}" | grep -qx "$1"; }

echo
echo "== Interfaces this branch depends on"

# 1. /cmd_vel type. This is the single most important value: a wrong type is not an error
#    anywhere, the robot simply never receives a command.
t="$(topic_type /cmd_vel)"
case "$t" in
  geometry_msgs/msg/Twist)
    ok "/cmd_vel is $t (matches this branch)" ;;
  geometry_msgs/msg/TwistStamped)
    bad "/cmd_vel is $t, this branch assumes geometry_msgs/msg/Twist."
    bad "  Set enable_stamped_cmd_vel: true in config/nav2_raupy.yaml (5 places),"
    bad "  cmd_vel_stamped: true in config/supervisor.yaml and config/stuck_monitor.yaml,"
    bad "  and the cmd_vel type in config/domain_bridge.yaml." ;;
  '') bad "/cmd_vel not advertised. Nobody is subscribing on the robot; the driver is down." ;;
  *)  bad "/cmd_vel is '$t', which is neither Twist nor TwistStamped." ;;
esac

# 2. Odometry topic. Nav2 reads it by name in three places in nav2_raupy.yaml.
if has_topic /odometry/filtered; then
  ok "/odometry/filtered exists ($(topic_type /odometry/filtered))"
else
  bad "no /odometry/filtered. Candidates seen:"
  printf '%s\n' "${topics[@]}" | grep -iE 'odom' | sed 's/^/         /'
  bad "  Put the right one in config/nav2_raupy.yaml (odom_topic, 3x) and config/domain_bridge.yaml."
fi

# 3. Scan.
if has_topic /scan; then
  ok "/scan exists ($(topic_type /scan))"
else
  bad "no /scan. Candidates seen:"
  printf '%s\n' "${topics[@]}" | grep -iE 'scan|laser' | sed 's/^/         /'
fi

# 4. ToF sensors. This branch assumes they are absent; if they came back, turn them on.
tof=$(printf '%s\n' "${topics[@]}" | grep -cE '^/range/(fl|fr|rl|rr)$')
if [ "$tof" -eq 0 ]; then
  ok "no /range/* ToF topics (as assumed: range_layer off, use_range_sensors false)"
else
  warn "$tof /range/* topics exist after all. This branch ignores them; to use them set"
  warn "  use_range_sensors:=true and put range_layer back into the plugin lists and the"
  warn "  collision monitor sources in config/nav2_raupy.yaml."
fi

# 5. Everything below needs live data, so one subscriber node does the rest in one go.
echo
echo "== Live data (takes about 40 s)"
timeout 90 python3 - <<'EOF'
import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

SCANS = 60          # ~6 s of an RPLIDAR at 10 Hz
NO_RETURN = 0.70    # a beam is "blind" if it fails this often
SECTOR_MIN_DEG = 5  # ignore shorter blind runs (single dead beams)

rclpy.init()
node = rclpy.create_node('check_interfaces')
scans = []
node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
buf = Buffer()
TransformListener(buf, node)

deadline = node.get_clock().now().nanoseconds * 1e-9 + 40.0
while len(scans) < SCANS and node.get_clock().now().nanoseconds * 1e-9 < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

if not scans:
    print('  [DIFF] no /scan data arrived; nothing below could be measured')
    raise SystemExit(0)

s = scans[-1]
print(f'  [ OK ] /scan: {len(s.ranges)} beams, frame "{s.header.frame_id}", '
      f'{math.degrees(s.angle_max - s.angle_min):.0f} deg span, '
      f'range {s.range_min:.2f}-{s.range_max:.1f} m')

# TF base_link -> the scan's own frame. The chassis box filter needs this transform to exist
# at all, and the rear_shadow angles in scan_filter.yaml assume the lidar sits at yaw 180 deg.
urdf_yaw = None
try:
    tf = buf.lookup_transform('base_link', s.header.frame_id, Time())
    q = tf.transform.rotation
    urdf_yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    t = tf.transform.translation
    print(f'  [ OK ] TF base_link -> {s.header.frame_id}: '
          f'xyz ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), yaw {urdf_yaw:.1f} deg')
except TransformException as exc:
    print(f'  [DIFF] no TF base_link -> {s.header.frame_id} ({exc}).')
    print('         The box filter and the costmaps need it. Check the base frame name:')
    print('         Raupy had base_link and no base_footprint.')

# Blind sectors: the Wi-Fi antennas behind the lidar block beams straight back on every
# ROSbot 2 PRO. Confirm the sector cut by scan_filter.yaml's rear_shadow matches this unit.
n = min(len(x.ranges) for x in scans)
misses = [0] * n
for x in scans:
    for i in range(n):
        r = x.ranges[i]
        if not math.isfinite(r) or r < x.range_min or r > x.range_max:
            misses[i] += 1
blind = [m / len(scans) >= NO_RETURN for m in misses]
step = math.degrees(s.angle_increment)
start_deg = math.degrees(s.angle_min)

sectors, run_start = [], None
for i in range(n + 1):
    is_blind = blind[i] if i < n else False
    if is_blind and run_start is None:
        run_start = i
    elif not is_blind and run_start is not None:
        if (i - run_start) * step >= SECTOR_MIN_DEG:
            sectors.append((start_deg + run_start * step, start_deg + (i - 1) * step))
        run_start = None

def wrap(deg):
    """Fold an angle into (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


print(f'  ---- blind sectors over {len(scans)} scans (>= {NO_RETURN:.0%} beams with no return),'
      ' in the lidar frame:')
for lo, hi in sectors:
    print(f'         {lo:+7.1f} .. {hi:+7.1f} deg  ({hi - lo:.0f} deg wide)')
if not sectors:
    print('         none. If that holds up, the rear_shadow filter in scan_filter.yaml is')
    print('         cutting live beams: shrink it and raise goal_preemption_lidar_fov_deg.')
print('         scan_filter.yaml currently cuts -35.0 .. +20.0 deg (55 deg) in this frame.')
print('         Widen or move it if a blind sector above sticks out of that range.')

# The widest blind sector is the Wi-Fi antennas, and they sit straight back on the chassis.
# That makes them a physical reference for where the lidar really points: whatever beam
# direction they block IS 180 deg in base_link. Comparing that with the yaw the robot's URDF
# publishes tells us whether the URDF is right -- on Raupy it was 90 deg off, which is what
# laser_yaw_fix corrects.
if sectors:
    lo, hi = max(sectors, key=lambda s_: s_[1] - s_[0])
    centre = wrap((lo + hi) / 2.0)
    true_yaw = wrap(180.0 - centre)
    print(f'  ---- widest blind sector centred at {centre:+.1f} deg in the lidar frame.')
    print('         Those are the antennas, which sit at 180 deg on the chassis, so the lidar')
    print(f'         frame really points at yaw {true_yaw:+.1f} deg in base_link.')
    if urdf_yaw is None:
        print('         No TF to compare it against (see above).')
    elif abs(wrap(true_yaw - urdf_yaw)) <= 15.0:
        print(f'         The URDF says {urdf_yaw:+.1f} deg, which agrees. Leave laser_yaw_fix unset.')
    else:
        print(f'  [DIFF] The URDF says {urdf_yaw:+.1f} deg, off by '
              f'{wrap(true_yaw - urdf_yaw):+.1f} deg. Run the stack with')
        print(f'         laser_yaw_fix:={math.radians(true_yaw):.5f}')
        print('         (add it to the ros2 launch line in scripts/raupy_explore.sh), as Raupy')
        print('         needed 3.14159. Without it SLAM builds the map with the scan rotated.')
EOF

echo
if [ "$DIFFS" -eq 0 ]; then
  echo "No differences found against this branch's assumptions."
  echo "Next: scripts/raupy_preflight.sh"
else
  echo "$DIFFS difference(s) found. Fix the files named above before running the stack,"
  echo "then re-run this script."
  exit 1
fi
