/*
* @Author: Andres
* @Date:   2019-03-01 21:26:06
* @Last Modified by:   Andres
* @Last Modified time: 2019-03-01 22:13:43
*/

#include "multi_agent_plan/manhattan_planner.hpp"

namespace multi_agent_plan
{
	ManhattanPlanner::ManhattanPlanner(const Map& grid)
	: Planner2D(grid)
	{
	}

	std::vector<geometry_msgs::Pose2D> ManhattanPlanner::pathPlanning( geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D goal )
	{
		std::vector<geometry_msgs::Pose2D> path;

		std::vector<int> xs;
		std::vector<int> ys;

		// Calculating the size of the x and y coordinates
		int x_size = std::abs( start_pose.x - goal.x ) + 1;
		int y_size = std::abs( start_pose.y - goal.y ) + 1;

		xs.resize( x_size );
		ys.resize( y_size );
		path.resize( x_size + y_size - 1 );

		// Checking if they are changes in the x coordinate
		if ( start_pose.x > goal.x )
		{
			std::iota(xs.begin(), xs.end(), goal.x);
			std::reverse(xs.begin(), xs.end());
		}
		else if ( start_pose.x < goal.x )
		{
			std::iota(xs.begin(), xs.end(), start_pose.x );
		}
		else
		{
			xs[ 0 ] = start_pose.x;// No changes in x coordinate
		}

		// for(auto m : xs)
		// {
		// 	std::cout << m << " ";
		// }

		// std::cout << std::endl;

		// Checking if they are changes in the y coordinate
		if ( start_pose.y > goal.y )
		{
			std::iota(ys.begin(), ys.end(), goal.y);
			std::reverse(ys.begin(), ys.end());
		}
		else if ( start_pose.y < goal.y )
		{
			std::iota(ys.begin(), ys.end(), start_pose.y);
		}
		else
		{
			ys[ 0 ] = start_pose.y;// No changes in y coordinate
		}

		// for(auto m : ys)
		// {
		// 	std::cout << m << " ";
		// }

		// std::cout << std::endl;

		// Adding the x coordinates in the path by keeping fix the y coordinate
		for (int i = 0; i < xs.size(); ++i)
		{
			path[ i ].x = xs[ i ];
			path[ i ].y = start_pose.y;
		}

		// Adding the y coordinates in the path by keeping fix the x coordinate
		for (int i = 1; i < ys.size(); ++i)
		{
			path[ x_size + i - 1 ].x = path[ x_size - 1 ].x;
			path[ x_size + i - 1 ].y = ys[ i ];	
		}

		// ROS_INFO_STREAM( "Size of the path: " << path.size() );

		return path;
	}
}