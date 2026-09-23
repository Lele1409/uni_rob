"""Tests for raupy_exploration.cmd_vel_util.

The point of these is the branch difference: Bisasam takes a plain Twist, Raupy a
TwistStamped, and picking the wrong one is silent -- the robot simply never moves. So the
tests assert the exact message classes, not just that "something" comes back.
"""

from geometry_msgs.msg import Twist, TwistStamped

from raupy_exploration.cmd_vel_util import cmd_vel_twist, cmd_vel_type, make_cmd_vel


def test_type_follows_the_flag():
    assert cmd_vel_type(False) is Twist
    assert cmd_vel_type(True) is TwistStamped


def test_unstamped_carries_the_velocity_and_has_no_header():
    msg = make_cmd_vel(False, linear_x=0.25, angular_z=-0.4)
    assert isinstance(msg, Twist)
    assert msg.linear.x == 0.25
    assert msg.angular.z == -0.4
    assert not hasattr(msg, 'header')


def test_stamped_carries_the_velocity_under_twist():
    msg = make_cmd_vel(True, linear_x=0.1, angular_z=0.2, frame_id='base_link')
    assert isinstance(msg, TwistStamped)
    assert msg.twist.linear.x == 0.1
    assert msg.twist.angular.z == 0.2
    assert msg.header.frame_id == 'base_link'


def test_stamp_is_applied_when_given():
    # The diff drive controller drops stamped commands older than cmd_vel_timeout, so a
    # caller-supplied stamp has to survive into the message.
    stamp = TwistStamped().header.stamp
    stamp.sec, stamp.nanosec = 1234, 5678
    msg = make_cmd_vel(True, stamp=stamp)
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1234, 5678)


def test_zero_command_is_the_default():
    for stamped in (False, True):
        twist = cmd_vel_twist(make_cmd_vel(stamped))
        assert twist.linear.x == 0.0
        assert twist.angular.z == 0.0


def test_cmd_vel_twist_unwraps_either_type():
    plain = Twist()
    plain.linear.x = 0.3
    assert cmd_vel_twist(plain) is plain
    assert cmd_vel_twist(plain).linear.x == 0.3

    stamped = TwistStamped()
    stamped.twist.linear.x = 0.3
    assert cmd_vel_twist(stamped) is stamped.twist
    assert cmd_vel_twist(stamped).linear.x == 0.3
