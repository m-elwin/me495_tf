import unittest
from launch import LaunchDescription
from launch_ros.actions import Node as NodeAction
import launch_testing
import pytest
import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

@pytest.mark.rostest
def generate_test_description():
    in_out_action = NodeAction(package="me495_tf",
                               executable="in_out",
                               )
    return (
        LaunchDescription([
            in_out_action,
            launch_testing.actions.ReadyToTest()
            ]),
        # These are extra parameters that get passed to the test functions
        {
            'in_out' : in_out_action
        }
    )

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        # The buffer stores received tf frames
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

class TestME495Tf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = TestNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_static_transform(self, launch_service, in_out, proc_output):
        proc_output.assertWaitFor("Static Transform: world->base", process=in_out)
        rclpy.spin_once(self.node)
        trans = self.node.buffer.lookup_transform("world", "base", rclpy.time.Time())
