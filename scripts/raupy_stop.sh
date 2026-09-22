#!/bin/bash
# raupy_stop.sh — stop exploration early but keep SLAM running, so the map can still be saved.
#   scripts/raupy_stop.sh
# Same as the supervisor's first steps: pause the explorer, cancel all Nav2 goals,
# send zero velocity. Resume with:
#   ros2 service call /control_exploration frontier_exploration_ros2/srv/ControlExploration "{action: 1}"

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/raupy_env.sh" >/dev/null
raupy_use_stack_domain  # the stack runs on its own domain, behind domain_bridge

# action 2 = STOP. quit_after_stop false keeps the node alive for a resume.
timeout 10 ros2 service call /control_exploration frontier_exploration_ros2/srv/ControlExploration \
  "{action: 2, delay_seconds: 0.0, quit_after_stop: false}" \
  || echo "raupy_stop: explorer control service did not answer" >&2

# Empty request (zero goal id + zero stamp) = cancel all goals.
timeout 10 ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}" \
  || echo "raupy_stop: Nav2 cancel did not answer" >&2

# 1 s of zeros overrides any last command from the controller/smoother.
# pub first waits for the robot's /cmd_vel subscriber; discovery over Wi-Fi takes a few
# seconds, so the timeout must be generous or it gets killed before sending anything.
timeout 20 ros2 topic pub -w 1 -r 10 -t 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}}" >/dev/null \
  || echo "raupy_stop: zero velocity not sent (no /cmd_vel subscriber found)" >&2

echo "raupy_stop: done. Save the map with scripts/raupy_save_map.sh before closing the launch."
