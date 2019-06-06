/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-22 11:03:38
 */

#ifndef CELL_H
#define CELL_H

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

namespace multi_agent_plan
{
	class Cell
	{
	public:
		geometry_msgs::Pose2D pose_;
		Cell* parent_;
		bool is_occupied_;

		Cell( geometry_msgs::Pose2D pose, Cell* parent = nullptr, bool is_occupied = false )
		: pose_( pose )
		, parent_( parent )
		, is_occupied_( is_occupied )
		{
			
		}

		Cell(Cell* parent = nullptr, bool is_occupied = false)
		: parent_( parent )
		, is_occupied_( is_occupied )
		{}

		// virtual ~Cell()
		// {}

		// expand( int m, int n ) = 0;
	};
}

#endif