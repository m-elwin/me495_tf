"""
Create one static frame and two moving frames.

The tf tree produced from this node will look like

      world
       |
      base
      /  \
    left right

The left and right nodes will move in and out and rotate about the base z axis

"""
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import pi
from .quaternion import angle_axis_to_quaternion

class InOut(Node):
    """
       Static Broadcasts: world -> base
       Broadcasts base -> left and base -> right
    """
    def __init__(self):
        super().__init__('in_out')
        # Static broadcasters publish on /tf_static. We will only need to publish this once
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Now create the transform, noted that it must have a parent frame and a timestamp
        # The header contains the timing information and frame id
        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = "world"
        world_base_tf.child_frame_id = "base"

        # The base frame will be raised in the z direction by 1 meter and be aligned with world
        # We are relying on the default values of the transform message (which defaults to no rotation)
        world_base_tf.transform.translation.z = 1.0
        self.static_broadcaster.sendTransform(world_base_tf)

        self.dx = 10 # used to control frame movement
        # create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        # Create a timer to do the rest of the transforms
        self.tmr = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        base_left = TransformStamped()
        base_left.header.frame_id = "base"
        base_left.child_frame_id = "left"
        base_left.transform.translation.x = -float(self.dx)
        # get a quaternion corresponding to a rotation by theta about an axis
        degrees = 36 * self.dx
        radians = degrees * pi / 180.0
        base_left.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, 1.0])

        base_right = TransformStamped()
        base_right.header.frame_id = "base"
        base_right.child_frame_id = "right"
        base_right.transform.translation.x = float(self.dx)
        base_right.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, -1.0])

        # don't forget to put a timestamp
        time = self.get_clock().now().to_msg()
        base_right.header.stamp = time
        base_left.header.stamp = time

        self.broadcaster.sendTransform(base_left)
        self.broadcaster.sendTransform(base_right)

        # update the movement
        self.dx -= 1
        if self.dx == 0:
            self.dx = 10


def in_out_entry(args=None):
    rclpy.init(args=args)
    node = InOut()
    rclpy.spin(node)
    rclpy.shutdown()




    dx = 10
    while not rospy.is_shutdown():

        rate.sleep()

