"""Test quaternion math."""

from geometry_msgs.msg import Quaternion
from me495_tf.quaternion import angle_axis_to_quaternion


def test_identity():
    assert angle_axis_to_quaternion(0, [1.0, 0.0, 0.0]) == Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
