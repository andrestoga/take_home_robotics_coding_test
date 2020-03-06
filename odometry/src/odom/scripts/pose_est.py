# Instructions

# Get the ticks of the encoders from topics
# the encoders provide absolute positions in units of 'ticks' provided on topics for each wheel (encoders/left_wheel and encoders/right_wheel) of type std_msgs/Int64. The data range of the encoder is [0 - 16777216], with the value wrapping between the min and max in the case of overflow / underflow.
# 
# Get from ROS parameters
# the ticks-per-meter calibration of each wheel
# the distance between the center of the two wheels
# fixed frequency of the calculation of the odometry specified as a ROS parameter
# 
# Publish to a topic
# the x, y position of the robot, the heading, the instantaneous speed of the robot and the instantaneous rotational speed of the robot should be reported on a topic with messages of type nav_msgs/Odometry

#!/usr/bin/env python
import numpy as np
import math
import csv
import matplotlib.pyplot as plt
import sys

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rospy
from std_msgs.msg import Int64
import message_filters
from nav_msgs.msg import Odometry
import tf2_ros

class Encoder:
    """docstring for Encoder"""
    def __init__(self, prev_ticks=0, ticks=0):
        self.prev_ticks = 0
        self.ticks = 0

class Odom:
    """docstring for Odom"""
    def __init__(self, min_ticks, max_ticks, ticks_per_meter, base_width):
        self.min_ticks = min_ticks
        self.max_ticks = max_ticks
        self.x = 0
        self.y = 0
        self.theta = 0
        self.ticks_per_meter = ticks_per_meter
        self.base_width = base_width
        self.inst_speed = 0
        self.inst_rot_speed = 0

        self.encoder_left = Encoder()
        self.encoder_right = Encoder()

    def update(self, left_ticks, right_ticks, dt):

        self.encoder_left.ticks = left_ticks
        self.encoder_right.ticks = right_ticks

        encoders = [self.encoder_left, self.encoder_right]

        for enc in encoders:

            if check_overflow(enc.prev_ticks, enc.ticks):
               enc.ticks = (max_ticks - enc.prev_ticks) + enc.ticks
            elif check_underflow(enc.prev_ticks, enc.ticks):
                enc.ticks = -1 * ((max_ticks - enc.ticks) + enc.prev_ticks)
            else:
                enc.ticks = enc.ticks - enc.prev_ticks

        distance_left_wheel = self.encoder_left.ticks / self.ticks_per_meter
        distance_right_wheel = self.encoder_right.ticks / self.ticks_per_meter

        heading_increment = (distance_right_wheel - distance_left_wheel) / self.base_width

        x_increment = cos( heading_increment ) * ( distance_left_wheel + distance_right_wheel ) / 2
        y_increment = -sin( heading_increment ) * ( distance_left_wheel + distance_right_wheel ) / 2

        x_fixed_increment = cos( heading_increment ) * x_increment - sin( heading_increment ) * y_increment

        y_fixed_increment = sin( heading_increment ) * x_increment + cos( heading_increment ) * y_increment

        omega_left = distance_left_wheel / dt
        omega_right = distance_right_wheel / dt

        self.x = x_fixed_increment + self.x
        self.y = y_fixed_increment + self.y
        self.theta = wrap_theta(heading_increment + self.theta)

        self.inst_speed = ( omega_left + omega_right ) / 2
        # self.inst_speed = ( distance_left_wheel + distance_right_wheel ) / 2
        self.inst_rot_speed = ( omega_left - omega_right ) / self.base_width
        # self.inst_rot_speed = ( distance_left_wheel - distance_right_wheel ) / self.base_width

        self.encoder_left.prev_ticks = self.encoder_left.ticks
        self.encoder_right.prev_ticks = self.encoder_right.ticks

    def check_underflow(self, prev_ticks, ticks):
        if prev_ticks < ticks:
            return True
        return False

    def check_overflow(self, prev_ticks, ticks):
        if prev_ticks > ticks:
            return True
        return False

    #   Wrapping from 0 to 180, -180 to 0
    def wrap_theta(self, theta):
        if theta > math.pi:
            theta += -math.pi * 2
        elif theta < -math.pi:
            theta += math.pi * 2
        return theta        

class OdomNode:
    """docstring for OdomNode"""
    def __init__(self, odom):
        self.odom = odom
        self.left_enc_sub = message_filters.Subscriber( 'encoders/left_wheel', Int64 )
        self.right_enc_sub = message_filters.Subscriber( 'encoders/right_wheel', Int64 )

        self.ts = message_filters.TimeSynchronizer([self.left_enc_sub, self.right_enc_sub], 10)
        self.ts.registerCallback(self.leftRightEncCallback)

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)

    def leftRightEncCallback( left_ticks, right_ticks ):

        odom_broadcaster = tf2_ros.TransformBroadcaster()

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).toSec()

        odom.update( left_ticks, right_ticks, dt )

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, odom.theta)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (odom.x, odom.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"

        # set the position
        odom_msg.pose.pose = Pose(Point(odom.x, odom.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist = Twist(Vector3(odom.inst_speed, 0, 0), Vector3(0, 0, odom.inst_rot_speed))

        # publish the message
        self.pub_odom.publish(odom_msg)

        self.last_time = self.current_time

if __name__ == '__main__':
    rospy.init_node('pose_estimation', anonymous=True)

    min_ticks = rospy.get_param("min_ticks")
    max_ticks = rospy.get_param("max_ticks")
    ticks_per_meter = rospy.get_param("ticks_per_meter")
    base_width = rospy.get_param("base_width")

    fixed_frecuency = rospy.get_param("fixed_frecuency")

    odom = Odom(min_ticks, max_ticks, ticks_per_meter, base_width)
    odomN = OdomNode(odom)

    rate = rospy.Rate(fixed_frecuency)

    while not rospy.is_shutdown():
        rospy.spinOnce()
        rate.sleep()