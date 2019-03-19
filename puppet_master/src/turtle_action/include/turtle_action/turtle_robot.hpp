/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-03-14
 */

#ifndef TURTLE_ROBOT_HPP
#define TURTLE_ROBOT_HPP

#include "ros/ros.h"
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

namespace canonical
{
	class TurtleRobot
	{
	public:
		
		// Proportional Controller. Turtle needs to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
		/**
		* @brief      Calculates the speed.
		*
		* @param[in]  goal_pose  The goal pose
		*
		* @return     The speed.
		*/
		geometry_msgs::Twist calculateSpeed( turtlesim::Pose goal_pose );

		/**
		* @brief      Gets the euclidean distance.
		*
		* @param[in]  x1    The x 1
		* @param[in]  y1    The y 1
		* @param[in]  x2    The x 2
		* @param[in]  y2    The y 2
		*
		* @return     The distance.
		*/
		double getDistance( double x1, double y1, double x2, double y2 );

		turtlesim::Pose pose_;
	};
}

#endif