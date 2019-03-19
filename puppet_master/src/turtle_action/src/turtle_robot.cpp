/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-03-13
 */
#include "ros/ros.h"
#include "turtle_action/turtle_robot.hpp"

namespace canonical
{
	double TurtleRobot::getDistance( double x1, double y1, double x2, double y2 )
	{
		return sqrt( pow( ( x2 - x1 ), 2 ) + pow( ( y2 - y1 ), 2 ) );
	}

	geometry_msgs::Twist TurtleRobot::calculateSpeed( turtlesim::Pose goal_pose )
	{
		geometry_msgs::Twist vel_msg;

		//linear velocity 
		// vel_msg.linear.x = 1.5 * getDistance( pose_.x, pose_.y, goal_pose.x, goal_pose.y );
		float distance = getDistance( pose_.x, pose_.y, goal_pose.x, goal_pose.y );
		vel_msg.linear.x = 1.5 * distance;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		// vel_msg.angular.z = 4 * ( atan2( goal_pose.y - pose_.y, goal_pose.x - pose_.x ) - pose_.theta );
		float goal_angle = atan2( goal_pose.y - pose_.y, goal_pose.x - pose_.x );
		vel_msg.angular.z = 8.0 * ( goal_angle - pose_.theta );

		// ROS_INFO( "Turtle pose: %f %f", pose_.x, pose_.y );
		// ROS_INFO( "Goal : %f %f", goal_pose.x, goal_pose.y );
		// ROS_INFO( "Distance : %f", distance );
		// ROS_INFO( "Goal angle : %f", goal_angle*180/M_PI );
		// ROS_INFO( "Turtle angle : %f", pose_.theta*180/M_PI );
		// ROS_INFO( "Linear : %f", vel_msg.linear.x );
		// ROS_INFO( "Angular : %f", vel_msg.angular.z );
		// ROS_INFO("\n");

		return vel_msg;
	}
}