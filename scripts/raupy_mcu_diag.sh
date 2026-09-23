#!/bin/bash
# raupy_mcu_diag.sh — why is the MCU dead? Read-only, does not move the robot.
#   scripts/raupy_mcu_diag.sh
#
# LR2 (yellow) means two very different things — "STM32 in bootloader" or "STM32
# malfunctioning" — and the remedy differs. This separates them from the logs:
#
#   A) no "firmware started"  -> the MCU never left its bootloader. On 24.04 the BOOT0 pin
#      floats high, so the STM32 boots into the system bootloader EVERY time and only
#      microros.service's bootloader GO jumps it out (it retries for ~4 s and can lose that
#      race). Fix: restart microros, then rosbot. No power-cycle needed.
#   B) "firmware started" + entities, but no data -> the firmware runs and the failure is
#      below it, on the sensor/motor side. That is the hardware suspicion from 2026-09-22.
#
# NEVER run core2-go.sh by hand to fix A: it fires 57600 8E1 handshake bytes into the live
# 576000 8N1 session and corrupts it. Restart the unit instead, which does it in the right order.

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/raupy_env.sh"

SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 $RAUPY_USER@$RAUPY_HOST"
$SSH true 2>/dev/null || { echo "SSH to $RAUPY_USER@$RAUPY_HOST failed"; exit 1; }

echo "== Services"
for svc in microros rosbot; do
  echo "  $svc: $($SSH systemctl is-active "$svc" 2>/dev/null)"
done

echo
echo "== Did the firmware leave the bootloader? (microros, this boot)"
go_ok=$($SSH 'journalctl -u microros -b --no-pager | grep -c "firmware started"')
# The agent logs one line per XRCE object ("topic created", "datawriter created", ...),
# never the word "entity" — a working boot registers ~51 of them.
entities=$($SSH 'journalctl -u microros -b --no-pager | grep -c " created  *|"')
echo "  'firmware started' lines: $go_ok"
echo "  entity-creation lines:    $entities"
$SSH 'journalctl -u microros -b --no-pager | tail -15 | sed "s/^/  | /"'

echo
echo "== Did the driver get motor feedback? (rosbot, this boot)"
$SSH 'journalctl -u rosbot -b --no-pager | grep -iE "activation failed|feedback|error" | tail -10 | sed "s/^/  | /"'

echo
echo "== Battery"
# Piping straight into sed would hide the exit status behind sed's, so capture first.
batt=$($SSH 'source /usr/local/bin/rosbot-env.sh >/dev/null 2>&1; timeout 10 ros2 topic echo /battery --once 2>/dev/null | grep -E "voltage|percentage|present"')
if [ -n "$batt" ]; then
  echo "$batt" | sed 's/^/  /'
else
  echo "  no /battery data — the firmware publishes nothing (half-dead MCU)"
fi

echo
echo "== Verdict"
if [ "${go_ok:-0}" -eq 0 ]; then
  echo "  A) The firmware never started: the MCU is sitting in its bootloader."
  echo "     The bootloader GO lost its race, and pressing RST cannot fix this — a reset"
  echo "     just drops the STM32 back into the bootloader with no GO to follow."
  echo "     Fix (needs your password, husarion has no passwordless sudo for these):"
  echo "       ssh $RAUPY_USER@$RAUPY_HOST"
  echo "       sudo systemctl restart microros   # jumps the firmware, reattaches the agent"
  echo "       sleep 20"
  echo "       sudo systemctl restart rosbot     # driver, after microros is up"
  echo "     Then: scripts/raupy_preflight.sh && scripts/raupy_drive_test.sh"
else
  echo "  B) The firmware started and registered entities, so the bootloader GO worked."
  echo "     The failure is below the firmware, on the sensor/motor side."
  echo "     Restarting microros will NOT help. Power-cycle (twice if needed), and if it"
  echo "     persists on a full battery it is a hardware fault worth reporting."
fi
