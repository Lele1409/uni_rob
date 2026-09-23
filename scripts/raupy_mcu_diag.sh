#!/bin/bash
# raupy_mcu_diag.sh — why is the MCU dead? Read-only, does not move the robot.
#   scripts/raupy_mcu_diag.sh
#
# Both robots are ROSbot 2 PRO, so the CORE2/STM32 failure modes are the same; only where the
# evidence lives differs. Raupy logged to the microros/rosbot systemd units, Bisasam runs the
# same two pieces as Docker containers, so this reads container logs instead.
#
# LR2 (yellow) means two very different things — "STM32 in bootloader" or "STM32
# malfunctioning" — and the remedy differs. This separates them from the logs:
#
#   A) no "firmware started"  -> the MCU never left its bootloader. The micro-ROS agent's
#      bootloader GO is what jumps it out, and it can lose that race at boot.
#      Fix: restart the micro-ROS agent container, then the driver container. No power-cycle.
#   B) "firmware started" + entities, but no data -> the firmware runs and the failure is
#      below it, on the sensor/motor side. That is the hardware suspicion from 2026-09-22
#      (on Raupy: four power-cycles in one evening, three of them dead, nothing in the logs
#      telling them apart). Fix: power-cycle, and retry once if the first one doesn't take.
#
# NEVER run core2-go.sh (or any bootloader handshake) by hand against a live agent: it fires
# 57600 8E1 handshake bytes into the running 576000 8N1 session and corrupts it. Restart the
# container instead, which does it in the right order.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/raupy_env.sh"

SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $RAUPY_USER@$RAUPY_HOST"
$SSH true 2>/dev/null || { echo "SSH to $RAUPY_USER@$RAUPY_HOST failed"; exit 1; }

echo "== Containers"
$SSH 'docker ps --format "  {{.Names}}\t{{.Status}}\t{{.Image}}"' 2>/dev/null \
  || { echo "  could not reach docker as $RAUPY_USER"; exit 1; }

# The image and container names are the other group's to choose, so find them by what they
# are rather than by a hardcoded name.
agent="$($SSH 'docker ps --format "{{.Names}} {{.Image}}" | grep -iE "micro.?ros|agent" | head -1 | cut -d" " -f1')"
driver="$($SSH 'docker ps --format "{{.Names}} {{.Image}}" | grep -iE "rosbot|bringup|driver|controller" | grep -viE "micro.?ros|agent" | head -1 | cut -d" " -f1')"
echo
echo "  micro-ROS agent container: ${agent:-<not found>}"
echo "  driver container:          ${driver:-<not found>}"
if [ -z "$agent" ]; then
  echo "  Without the agent container the checks below cannot run. List the containers above"
  echo "  and pass the right name: RAUPY_AGENT=<name> scripts/raupy_mcu_diag.sh"
  agent="${RAUPY_AGENT:-}"
fi
agent="${RAUPY_AGENT:-$agent}"
driver="${RAUPY_DRIVER:-$driver}"

go_ok=0
if [ -n "$agent" ]; then
  echo
  echo "== Did the firmware leave the bootloader? ($agent)"
  go_ok=$($SSH "docker logs '$agent' 2>&1 | grep -c 'firmware started'")
  # The agent logs one line per XRCE object ("topic created", "datawriter created", ...),
  # never the word "entity" — a working boot registers ~51 of them.
  entities=$($SSH "docker logs '$agent' 2>&1 | grep -c ' created  *|'")
  echo "  'firmware started' lines: $go_ok"
  echo "  entity-creation lines:    $entities"
  $SSH "docker logs --tail 15 '$agent' 2>&1 | sed 's/^/  | /'"
fi

if [ -n "$driver" ]; then
  echo
  echo "== Did the driver get motor feedback? ($driver)"
  $SSH "docker logs '$driver' 2>&1 | grep -iE 'activation failed|feedback|error' | tail -10 | sed 's/^/  | /'"
fi

echo
echo "== Battery"
# Read it over DDS rather than on the robot: no ROS environment to source inside a container.
batt=$(timeout 15 ros2 topic echo /battery --once 2>/dev/null | grep -E "voltage|percentage|present")
if [ -n "$batt" ]; then
  echo "$batt" | sed 's/^/  /'
else
  echo "  no /battery data — the firmware publishes nothing (half-dead MCU), or this stack"
  echo "  names the topic differently (scripts/raupy_check_interfaces.sh lists the topics)."
fi

echo
echo "== Verdict"
if [ -z "$agent" ]; then
  echo "  Could not identify the micro-ROS agent container, so no verdict."
  echo "  Re-run with RAUPY_AGENT=<name> once you see it in the list above."
elif [ "${go_ok:-0}" -eq 0 ]; then
  echo "  A) The firmware never started: the MCU is sitting in its bootloader."
  echo "     The bootloader GO lost its race, and pressing RST cannot fix this — a reset"
  echo "     just drops the STM32 back into the bootloader with no GO to follow."
  echo "     Fix (restart the agent first, the driver only once the agent is up):"
  echo "       ssh $RAUPY_USER@$RAUPY_HOST"
  echo "       docker restart $agent"
  echo "       sleep 20"
  echo "       docker restart ${driver:-<driver container>}"
  echo "     Then: scripts/raupy_preflight.sh && scripts/raupy_drive_test.sh"
else
  echo "  B) The firmware started and registered entities, so the bootloader GO worked."
  echo "     The failure is below the firmware, on the sensor/motor side."
  echo "     Restarting the containers will NOT help. Power-cycle (twice if needed), and if it"
  echo "     persists on a full battery it is a hardware fault worth reporting."
fi
echo
echo "  Bisasam is shared with another group: check with them before restarting anything."
