# Overview
This is an example of tf's in action for ME495 at Northwestern University.
Used with [the transformation activity](https://nu-msr.github.io/ros_notes/ros2/activity/tf_activity.html).

# Nodes

`in_out` : moves some frames in and out while rotating them.

`tracker` : Listens to tf and outputs the distance between left and right frames.

# Tests
The repository also contains examples of tests that can be run with `colcon test`.

[quaternion_test.py](test/quaternion_test.py): An example of python unit tests.

[in_out_launch_test.py](test/in_out_launch_test.py): An example of a `launch_test` integration test.
