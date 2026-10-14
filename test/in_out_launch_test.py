import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.markers
from launch_testing_ros.actions import EnableRmwIsolation
import pytest
import rclpy
from rclpy.time import Duration, Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return (
        LaunchDescription([
            EnableRmwIsolation(),
            Node(package='me495_tf',
                 executable='in_out'),
            ReadyToTest()
            ])
    )


class TestME495Tf(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_static_transform(self, launch_service,  proc_output):
        buffer = Buffer()
        _ = TransformListener(buffer, self.node)
        proc_output.assertWaitFor('Static Transform: world->base', process='in_out', timeout=3.0)

        start_time = self.node.get_clock().now()
        while not buffer.can_transform('world', 'base', Time()):
            rclpy.spin_once(self.node)
            if self.node.get_clock().now() - start_time > Duration(seconds=1.0):
                # Something went wrong, we'll let the buffer lookup fail
                break

        xform = buffer.lookup_transform('world', 'base', Time())

        # Even though these are floating point numbers, test for exact equality
        # Because this transform is hard-coded and not the result of any math
        # So the exact values should be copied
        assert xform.transform.translation.x == 0.0
        assert xform.transform.translation.y == 0.0
        assert xform.transform.translation.z == 1.0
        assert xform.transform.rotation.x == 0.0
        assert xform.transform.rotation.y == 0.0
        assert xform.transform.rotation.z == 0.0
        assert xform.transform.rotation.w == 1.0


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):
    """Ensure that everything exits cleanly."""

    def test_exit_codes(self, proc_info):
        """Assert that each process exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
