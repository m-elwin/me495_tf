#!/usr/bin/env python
"""
    A basic tf2 listener that computes the distance and angle between the left and right frames
"""
from __future__ import division
import rospy
import tf2_ros

if __name__ == "__main__":
       rospy.init_node('tracker')

       # Create a buffer to hold the transforms for a period of time
       buffer = tf2_ros.Buffer()

       # create the listener, which will subscribe to /tf and listen for transforms,
       # storing them in buffer
       listener = tf2_ros.TransformListener(buffer)

       rate = rospy.Rate(2)
       while not rospy.is_shutdown():
           # we listen in a try block.  If a frame has not bene published
           # recently enough, then there will be an error and we continue
           # for demonstration purposes we catch each exception individually
           try:
               # get the latest transform between left and right
               trans = buffer.lookup_transform("left", "right", rospy.Time())
               rospy.loginfo("Transform is: " + str(trans))
           except tf2_ros.LookupException as e:
               # the frames don't exist yet
               rospy.loginfo("Lookup exception: " + str(e))
           except tf2_ros.ConnectivityException as e:
               # the tf tree has a disconnection
               rospy.loginfo("Connectivity exception: " + str(e))
           except tf2_ros.ExtrapolationException as e:
               # the times are two far apart to extrapolate
               rospy.loginfo("Extrapolation exception: " + str(e))
           finally:
               rate.sleep()


