#!/bin/bash
# raupy_drive_test.sh — does the robot react to /cmd_vel at all? Bypasses the whole stack:
# no Nav2, no bridge, no collision monitor, just a velocity command on the robot's own domain.
#   scripts/raupy_drive_test.sh            # 2 s forward at 0.08 m/s
#   scripts/raupy_drive_test.sh -0.08      # backward
#   scripts/raupy_drive_test.sh 0.0 0.4    # turn on the spot
# THE ROBOT MOVES (~16 cm). Keep the area clear and stay ready to lift it.
#
# It reads the wheel encoders (/joint_states) before and after, so it separates the two
# cases that look identical from the laptop:
#   wheels turned  -> the motors are fine, the problem is in the stack (Nav2 / bridge / timing)
#   wheels frozen  -> commands arrive but the MCU does not drive; power-cycle the robot.
#
# BISASAM BRANCH: /cmd_vel is a plain geometry_msgs/Twist here. Run this right after
# scripts/raupy_check_interfaces.sh has confirmed that type -- with the wrong one the wheels
# stay still and the test blames the MCU for what is really a type mismatch.

lin="${1:-0.08}"
ang="${2:-0.0}"
secs="${3:-2.0}"

_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_ws_root/scripts/raupy_env.sh" >/dev/null  # robot domain, no stack domain switch

echo "raupy_drive_test: domain $ROS_DOMAIN_ID, linear ${lin} m/s, angular ${ang} rad/s, ${secs} s"

python3 - "$lin" "$ang" "$secs" <<'EOF'
import sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

lin, ang, secs = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])

rclpy.init()
node = rclpy.create_node('raupy_drive_test')
state = {}


def on_js(msg):
    state['last'] = dict(zip(msg.name, msg.position))
    state.setdefault('names', list(msg.name))


node.create_subscription(JointState, '/joint_states', on_js, 10)
pub = node.create_publisher(Twist, '/cmd_vel', 10)

deadline = time.time() + 10.0
while 'last' not in state and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
if 'last' not in state:
    print('  no /joint_states: the driver is not publishing, nothing to test')
    sys.exit(2)

before = dict(state['last'])
print('  waiting for the robot to subscribe to /cmd_vel ...')
deadline = time.time() + 10.0
while pub.get_subscription_count() == 0 and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if pub.get_subscription_count() == 0:
    print('  nobody subscribes to /cmd_vel: the driver is not running')
    sys.exit(2)


def send(v, w):
    # An unstamped Twist carries no timestamp, so unlike on Raupy the robot cannot reject
    # this as "too old"; it stops on its own cmd_vel_timeout once we stop publishing.
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    pub.publish(msg)


print('  driving ...')
end = time.time() + secs
while time.time() < end:
    send(lin, ang)
    rclpy.spin_once(node, timeout_sec=0.05)
for _ in range(10):  # stop
    send(0.0, 0.0)
    rclpy.spin_once(node, timeout_sec=0.05)
for _ in range(10):  # let the last encoder values in
    rclpy.spin_once(node, timeout_sec=0.1)

after = dict(state['last'])
print('  wheel movement (rad):')
moved = 0.0
for name in state['names']:
    d = after.get(name, 0.0) - before.get(name, 0.0)
    moved = max(moved, abs(d))
    print(f'    {name}: {d:+.3f}')
if moved < 0.05:
    print('  RESULT: wheels did NOT turn. Commands are accepted but the MCU does not drive'
          ' the motors -> power-cycle the robot (not reboot), then rerun this test.')
else:
    print('  RESULT: wheels turned. The motors are fine, so a failed run is a stack problem'
          ' (Nav2 / bridge / timing), not the robot.')
node.destroy_node()
rclpy.try_shutdown()
EOF
