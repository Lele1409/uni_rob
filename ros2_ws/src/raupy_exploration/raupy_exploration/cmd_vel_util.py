"""Velocity command type selection (no ROS runtime logic beyond the message classes).

The two ROSbot 2 PRO units we can use are physically identical but run different ROS
stacks, and they disagree on the /cmd_vel message type:

- Bisasam (Humble, Docker image): plain ``geometry_msgs/Twist``. A TwistStamped is not
  just ignored, it is a different topic type, so publishing one means the robot never
  sees a command at all.
- Raupy (Jazzy, native rosbot_ros): ``geometry_msgs/TwistStamped``.

Every node of ours that writes velocities takes a ``cmd_vel_stamped`` parameter and goes
through the helpers here, so the same code runs on either robot. Nav2 has its own switch
for this (``enable_stamped_cmd_vel`` in nav2_raupy.yaml) and domain_bridge needs the type
spelled out in domain_bridge.yaml -- all three must agree.
"""

from geometry_msgs.msg import Twist, TwistStamped


def cmd_vel_type(stamped):
    """Message class for /cmd_vel: TwistStamped if ``stamped`` else Twist."""
    return TwistStamped if stamped else Twist


def make_cmd_vel(stamped, linear_x=0.0, angular_z=0.0, stamp=None, frame_id=''):
    """Build a velocity command of the matching type.

    ``stamp`` and ``frame_id`` are only used for the stamped variant; the robot's diff
    drive controller rejects commands whose stamp is older than its cmd_vel_timeout, so
    the caller passes the current time there.
    """
    if not stamped:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    msg = TwistStamped()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.twist.linear.x = linear_x
    msg.twist.angular.z = angular_z
    return msg


def cmd_vel_twist(msg):
    """The Twist part of either message type, for readers that only need the velocity."""
    return msg.twist if isinstance(msg, TwistStamped) else msg
