# raupy_env.sh — SOURCE this (don't execute) before every Raupy session on the laptop.
#   source scripts/raupy_env.sh
# Sets ROS Jazzy + this workspace, the same RMW/transport as the robot, and the
# robot's current ROS_DOMAIN_ID (derived from its IP, changes with the DHCP lease).

RAUPY_HOST="${RAUPY_HOST:-raupy.roblab.cs.hs-fulda.de}"
RAUPY_USER="${RAUPY_USER:-husarion}"
_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
[ -f "$_ws_root/install/setup.bash" ] && source "$_ws_root/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Read the domain live from the robot; fall back to the last known value.
_domain="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$RAUPY_USER@$RAUPY_HOST" \
  'cut -d= -f2 /etc/default/rosbot-domain' 2>/dev/null)"
if [ -n "$_domain" ]; then
  export ROS_DOMAIN_ID="$_domain"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FALLBACK:-103}"
  echo "raupy_env: robot not reachable, using ROS_DOMAIN_ID=$ROS_DOMAIN_ID (fallback)" >&2
fi
ros2 daemon stop >/dev/null 2>&1
echo "raupy_env: ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
unset _ws_root _domain
