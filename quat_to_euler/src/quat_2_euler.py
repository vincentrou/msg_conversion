#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
#import roslib
#roslib.load_manifest('quat_to_euler')
import rospy
import tf
import math

# ROS messages.
from sensor_msgs.msg import Imu
from quat_to_euler.msg import Euler

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.euler_msg = Euler()

        # Create subscribers and publishers.
        sub_imu   = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        pub_euler = rospy.Publisher("/imu/euler", Euler)
        rate = rospy.Rate(500)
        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_euler.publish(self.euler_msg)
                self.got_new_msg = False
            else:
                rate.sleep()

    # IMU callback function.
    def imu_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.roll  = r*180/math.pi
        self.euler_msg.pitch = p*180/math.pi
        self.euler_msg.yaw   = y*180/math.pi

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass
