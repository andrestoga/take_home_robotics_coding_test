/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-22 11:02:41
 */

#ifndef CELL_DFS_H
#define CELL_DFS_H

#include "ros/ros.h"
#include "cell.h"

namespace multi_agent_plan
{
	class CellDFS : public Cell
	{
	public:
		bool is_visited_;

		CellDFS( geometry_msgs::Pose2D pose, multi_agent_plan::CellDFS* parent = nullptr, bool is_visited = false )
		: Cell( pose, parent )
		, is_visited_( is_visited )
		{
			
		}

		CellDFS(bool is_visited = false)
		: is_visited_( is_visited )
		{}
	};
}

#endif