#!/bin/bash
# robot_select.sh — choose the robot all scripts in scripts/ talk to (saved in scripts/.robot).
#   scripts/robot_select.sh            # show the current choice
#   scripts/robot_select.sh bisasam    # raupy | bisasam
# $ROBOT in the environment still overrides the saved choice for a single command.

f="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.robot"
if [ -z "$1" ]; then
  echo "robot: $(cat "$f" 2>/dev/null || echo 'raupy (default)')"
  exit 0
fi
case "$1" in
  raupy|bisasam) echo "$1" > "$f" && echo "robot: $1" ;;
  *) echo "robot_select: unknown robot '$1' (raupy | bisasam)" >&2; exit 1 ;;
esac
