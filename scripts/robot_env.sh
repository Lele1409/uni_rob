# robot_env.sh — SOURCE this (don't execute) before every robot session on the laptop.
#   source scripts/robot_env.sh
#   ROBOT=bisasam source scripts/robot_env.sh   # one-off; scripts/robot_select.sh saves it
# Sets ROS Jazzy + this workspace, the same RMW/transport as the robot, and the
# robot's current ROS_DOMAIN_ID (derived from its IP, changes with the DHCP lease).

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Which robot: $ROBOT if set, else the choice saved by scripts/robot_select.sh, else raupy.
# Both are identical ROSbot 2 PRO builds, only host and fallback domain differ.
if [ -z "$ROBOT" ] && [ -f "$_ws_root/scripts/.robot" ]; then
  ROBOT="$(tr -d '[:space:]' < "$_ws_root/scripts/.robot")"
fi
export ROBOT="${ROBOT:-raupy}"
case "$ROBOT" in
  raupy)   _host=raupy.roblab.cs.hs-fulda.de;   _domain_fallback=103 ;;  # 10.33.130.103
  bisasam) _host=bisasam.roblab.cs.hs-fulda.de; _domain_fallback=101 ;;  # 10.33.130.101
  *) echo "robot_env: unknown ROBOT '$ROBOT' (raupy | bisasam)" >&2; return 1 2>/dev/null || exit 1 ;;
esac
ROBOT_HOST="${ROBOT_HOST:-$_host}"
ROBOT_USER="${ROBOT_USER:-husarion}"
# URDF says lidar yaw -90 deg, measured 180 deg on Raupy. Bisasam is the same build, so the
# same fix is assumed (not measured there yet). Empty = trust the robot's URDF.
export LASER_YAW_FIX="${LASER_YAW_FIX-3.14159}"

source /opt/ros/jazzy/setup.bash
[ -f "$_ws_root/install/setup.bash" ] && source "$_ws_root/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Read the domain live from the robot; fall back to the last known value.
_domain="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_HOST" \
  'cut -d= -f2 /etc/default/rosbot-domain' 2>/dev/null)"
if [ -n "$_domain" ]; then
  export ROS_DOMAIN_ID="$_domain"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FALLBACK:-$_domain_fallback}"
  echo "robot_env: robot not reachable, using ROS_DOMAIN_ID=$ROS_DOMAIN_ID (fallback)" >&2
fi
export ROBOT_DOMAIN="$ROS_DOMAIN_ID"
# The exploration stack runs on its own laptop-only domain; domain_bridge is the only process
# on the robot's domain (see ros2_ws/src/raupy_exploration/config/domain_bridge.yaml).
export STACK_DOMAIN="${STACK_DOMAIN:-57}"
# Switch this shell to the stack's domain (explore / stop / save_map scripts use it).
use_stack_domain() {
  export ROS_DOMAIN_ID="$STACK_DOMAIN"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ros2 daemon stop >/dev/null 2>&1
}
ros2 daemon stop >/dev/null 2>&1
echo "robot_env: robot $ROBOT ($ROBOT_HOST), ROS_DOMAIN_ID=$ROS_DOMAIN_ID (robot), stack domain $STACK_DOMAIN, RMW=$RMW_IMPLEMENTATION"
unset _ws_root _domain _host _domain_fallback
