import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@pytest.mark.rostest
def generate_test_description():
    in_out_action = Node(package="me495_tf",
                         executable="in_out",
                         )
    return (
        LaunchDescription([
            in_out_action,
            launch_testing.actions.ReadyToTest()
            ]),
        # These are extra parameters that get passed to the test functions
        {
            'in_out': in_out_action
        }
    )


class TestME495Tf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_static_transform(self, launch_service, in_out, proc_output):
        buffer = Buffer()
        _ = TransformListener(buffer, self.node)
        proc_output.assertWaitFor("Static Transform: world->base", process=in_out, timeout=3.0)
        rclpy.spin_once(self.node)
        xform = buffer.lookup_transform("world", "base", rclpy.time.Time())
        assert xform.transform.translation.x == 0.0
        assert xform.transform.translation.y == 0.0
        assert xform.transform.translation.z == 1.0
        assert xform.transform.rotation.x == 0.0
        assert xform.transform.rotation.y == 0.0
        assert xform.transform.rotation.z == 0.0
        assert xform.transform.rotation.w == 1.0
