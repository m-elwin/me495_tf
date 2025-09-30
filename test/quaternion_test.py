"""Test quaternion math."""

from math import cos, pi, sin, sqrt


from geometry_msgs.msg import Quaternion
from me495_tf.in_out import quatToMsg
from me495_tf.quaternion import angle_axis_to_quaternion
import pytest


def test_identity():
    """Test that angle_axis_to_quaternion correctly produces the identity quaternion."""
    assert angle_axis_to_quaternion(0, [1.0, 0.0, 0.0]) == Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


def test_quat_to_msg():
    """Test that quatToMsg successfully converts a list to a geometry_msgs.msg.Quaternion."""
    # Construct a unit quaternion with different angles
    w = cos(pi/6.0)
    x = 0
    y = sin(pi/6.0) * 1/sqrt(5.0)
    z = sin(pi/6.0) * 2/sqrt(5.0)
    assert w**2 + x**2 + y**2 + z**2 == pytest.approx(1.0)
    assert quatToMsg([w, x, y, z]) == Quaternion(x=x, y=y, z=z, w=w)
