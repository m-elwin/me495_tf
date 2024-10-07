"""A basic tf2 listener that computes the distance and angle between the left and right frames."""
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Tracker(Node):
    """Listens to TF frames and logs information based on how they change."""

    def __init__(self):
        super().__init__('tracker')

        # The buffer stores received tf frames
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.timer = self.create_timer(2, self.timer_callback)

    def timer_callback(self):
        # we listen in a try block.  If a frame has not been published
        # recently enough, then there will be an error and we continue.
        # For demonstration purposes we catch each exception individually
        try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
            trans = self.buffer.lookup_transform('left', 'right', rclpy.time.Time())
            self.get_logger().info(f'Transform is: {trans}')
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}')


def tracker_entry(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node)
    rclpy.shutdown()
