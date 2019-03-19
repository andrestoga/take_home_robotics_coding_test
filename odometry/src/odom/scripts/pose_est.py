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

class Odom:
    """docstring for Odom"""
    def __init__(self, min_ticks, max_ticks, ticks_per_meter, base_width):
        self.min_ticks = min_ticks
        self.max_ticks = max_ticks
        self.x = 0
        self.y = 0
        self.theta = 0
        self.left_ticks = 0
        self.right_ticks = 0
        self.ticks_per_meter = ticks_per_meter
        self.base_width = base_width
        self.inst_speed = 0
        self.inst_rot_speed = 0

    def update(self, left_ticks, right_ticks, dt):

        left_ticks = ticksPreprocessing( left_ticks )
        right_ticks = ticksPreprocessing( right_ticks )

        delta_left = left_ticks - self.left_ticks
        delta_right = right_ticks - self.right_ticks

        distance_left_wheel = delta_right / self.ticks_per_meter
        distance_right_wheel = delta_left / self.ticks_per_meter

        heading_increment = ( distance_right_wheel - distance_left_wheel ) / self.base_width

        x_increment = cos( heading_increment ) * ( distance_left_wheel + distance_right_wheel ) / 2
        y_increment = -sin( heading_increment ) * ( distance_left_wheel + distance_right_wheel ) / 2

        x_fixed_increment = cos( heading_increment ) * x_increment - sin( heading_increment ) * y_increment

        y_fixed_increment = sin( heading_increment ) * x_increment + cos( heading_increment ) * y_increment

        omega_left = distance_left_wheel / dt
        omega_right = distance_right_wheel / dt

        self.x = x_fixed_increment + self.x
        self.y = y_fixed_increment + self.y
        self.theta = heading_increment + self.theta
        self.theta = wrap_theta( self.theta )

        # self.inst_speed = ( omega_left + omega_right ) / 2
        self.inst_speed = ( distance_left_wheel + distance_right_wheel ) / 2
        # self.inst_rot_speed = ( omega_left - omega_right ) / self.base_width
        self.inst_rot_speed = ( distance_left_wheel - distance_right_wheel ) / self.base_width

        self.left_ticks = left_ticks
        self.right_ticks = right_ticks

    def ticks_preprocessing(self, ticks):

        if ( ticks < self.min_ticks )
        {
            ticks = self.min_ticks;
        }

        if ( ticks > self.max_ticks )
        {
            ticks = self.max_ticks
        }

        return ticks

    def wrap_theta(self, theta):
        if theta > math.pi:
            theta = -math.pi + theta
        elif theta < -math.pi
            theta = math.pi + theta
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