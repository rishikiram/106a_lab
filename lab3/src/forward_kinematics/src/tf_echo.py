#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the dependencies as described in example_pub.py
import rospy
import tf2_ros
import sys

def tf_echo(source_frame, target_frame):
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    # trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())

    while(True):
        try:
            print(tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time()))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('exception caught')

# Python's syntax for a main() method
if __name__ == '__main__':

    rospy.init_node('tf_echo', anonymous=True)

    base_frame = sys.argv[1]
    target_frame = sys.argv[2]
    tf_echo(base_frame, target_frame)
        