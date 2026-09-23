# raupy_env.sh — SOURCE this (don't execute) before every Bisasam session on the laptop.
#   source scripts/raupy_env.sh
# Sets ROS Jazzy + this workspace, the RMW the robot uses, and the robot's ROS_DOMAIN_ID.
#
# BISASAM BRANCH. The script and variable names keep the raupy_ prefix so this branch stays
# a small, readable diff against wf-materials; only the values behind them are Bisasam's.
# Differences from Raupy:
#   - host bisasam instead of raupy
#   - rmw_cyclonedds_cpp instead of rmw_fastrtps_cpp (Bisasam's Humble containers use
#     CycloneDDS, and DDS only talks to its own implementation)
#   - a fixed domain 30 set inside the container, not 100 + the last two digits of the IP
#     recomputed at every service start

RAUPY_HOST="${RAUPY_HOST:-bisasam.roblab.cs.hs-fulda.de}"
RAUPY_USER="${RAUPY_USER:-husarion}"
_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
[ -f "$_ws_root/install/setup.bash" ] && source "$_ws_root/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Two interfaces at once (Wi-Fi for the robot domain, loopback for the stack domain); see
# the comments in the file itself. Read from the source tree so this works before a build.
export CYCLONEDDS_URI="file://$_ws_root/ros2_ws/src/raupy_exploration/config/cyclonedds.xml"
if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  echo "raupy_env: rmw_cyclonedds_cpp is NOT installed. The laptop cannot talk to Bisasam." >&2
  echo "raupy_env:   sudo apt install ros-jazzy-rmw-cyclonedds-cpp" >&2
fi

# Domain 30, measured 2026-09-22. Unlike Raupy's it is a fixed value in the container's
# environment, so it does not follow the DHCP lease. Read it from the robot anyway when we
# can reach it, because another group also edits that compose file.
_domain=""
if _ssh_env="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$RAUPY_USER@$RAUPY_HOST" \
    'docker ps -q | xargs -r docker inspect --format "{{range .Config.Env}}{{println .}}{{end}}"' \
    2>/dev/null)"; then
  _domain="$(echo "$_ssh_env" | sed -n 's/^ROS_DOMAIN_ID=\([0-9]\+\)$/\1/p' | head -1)"
fi
if [ -n "$_domain" ]; then
  export ROS_DOMAIN_ID="$_domain"
else
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_FALLBACK:-30}"
  echo "raupy_env: could not read the domain from the robot, using ROS_DOMAIN_ID=$ROS_DOMAIN_ID (fallback)" >&2
fi
export RAUPY_ROBOT_DOMAIN="$ROS_DOMAIN_ID"
# The exploration stack runs on its own laptop-only domain; domain_bridge is the only process
# on the robot's domain (see ros2_ws/src/raupy_exploration/config/domain_bridge.yaml).
# On Bisasam that also keeps us off the other group's nodes, which share domain 30.
export RAUPY_STACK_DOMAIN="${RAUPY_STACK_DOMAIN:-57}"
# Switch this shell to the stack's domain (explore / stop / save_map scripts use it).
raupy_use_stack_domain() {
  export ROS_DOMAIN_ID="$RAUPY_STACK_DOMAIN"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ros2 daemon stop >/dev/null 2>&1
}
ros2 daemon stop >/dev/null 2>&1
echo "raupy_env: ROS_DOMAIN_ID=$ROS_DOMAIN_ID (robot), stack domain $RAUPY_STACK_DOMAIN, RMW=$RMW_IMPLEMENTATION"
unset _ws_root _domain _ssh_env
