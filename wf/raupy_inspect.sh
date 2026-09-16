#!/bin/bash
# Read-only inspection of Raupy. Run from the dev machine:
#   ssh robot@raupy.roblab.cs.hs-fulda.de 'bash -s' < wf/raupy_inspect.sh > wf/raupy_info.txt 2>&1
s() { echo; echo "=================== $* ==================="; }

s SYSTEM
hostname; uname -a; cat /etc/os-release | head -3; ip -4 addr | grep inet; uptime
s ROS
ls /opt/ros; env | grep -E 'ROS|RMW|CYCLONE|FASTRTPS|AMENT' | sort
grep -nE 'source|ROS_|RMW|export' ~/.bashrc
s HISTORY
tail -n 300 ~/.bash_history
s HOME
ls -la ~
s WORKSPACES
for w in $(find ~ -maxdepth 3 -type d -name src -not -path '*/.*' 2>/dev/null); do
  echo "--- $w"; ls "$w"
  for p in $(find "$w" -maxdepth 3 -name package.xml 2>/dev/null); do echo "  pkg: $(grep -oP '(?<=<name>)[^<]+' "$p") ($p)"; done
done
s LAUNCH_FILES
find ~ -path '*/src/*' \( -name '*launch*.py' -o -name '*.launch.xml' -o -name '*.launch' \) -not -path '*/.git/*' 2>/dev/null
s CONFIGS_AND_URDF
for f in $(find ~ -path '*/src/*' \( -name '*.yaml' -o -name '*.xacro' -o -name '*.urdf' \) -not -path '*/.git/*' -size -40k 2>/dev/null | grep -viE 'test|gazebo|\.github|rviz' | head -60); do
  echo "----- $f"; cat "$f"
done
s LAUNCH_CONTENTS
for f in $(find ~ -path '*/src/*' \( -name '*launch*.py' -o -name '*.launch.xml' \) -not -path '*/.git/*' -size -30k 2>/dev/null | grep -viE 'test|gazebo' | head -40); do
  echo "----- $f"; cat "$f"
done
s SYSTEMD_AUTOSTART
systemctl list-units --type=service --state=running --no-pager | grep -viE 'systemd|dbus|getty|snap|network|polkit|udisks|cron|rsyslog|ssh|avahi|cups|ModemManager|wpa|accounts|power|thermal|kerneloops|rtkit|upower|colord|gdm|switcheroo|fwupd|packagekit|whoopsie|bluetooth'
crontab -l 2>/dev/null; ls /etc/systemd/system | grep -viE 'wants|target|dbus|sshd|syslog'
s DEVICES
ls -la /dev/ttyUSB* /dev/ttyACM* /dev/input/js* 2>/dev/null; ls -la /dev/serial/by-id 2>/dev/null
lsusb; groups
cat /etc/udev/rules.d/*.rules 2>/dev/null
s ROS_GRAPH_LIVE
source /opt/ros/*/setup.bash 2>/dev/null
for w in ~/*/install/setup.bash ~/install/setup.bash; do [ -f "$w" ] && echo "sourcing $w" && source "$w"; done
ps aux | grep -E 'ros|launch|lidar|volks|driver|epos|ekf' | grep -v grep
timeout 10 ros2 node list; timeout 10 ros2 topic list -t
