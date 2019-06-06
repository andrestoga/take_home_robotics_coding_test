/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-11 17:44:05
 */

#ifndef DFS_H
#define DFS_H

#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"

#include "multi_agent_plan/det_planner_2d.h"
#include "multi_agent_plan/cell_dfs.h"

#include "nav_msgs/OccupancyGrid.h"

#include <queue>
#include <stack>

namespace multi_agent_plan
{
	class DFS: public DetPlanner2D<CellDFS>
	{
	public:

		std::stack<CellDFS*> open_;
		std::stack<CellDFS*> closed_;
		
		DFS( int width, int height, bool is_edge_quartile = true );
		DFS( nav_msgs::OccupancyGrid& ogm, bool is_edge_quartile = true );
		DFS(bool is_edge_quartile = true){}

		/**
		 * @brief      Plan for manhattan worlds with no obstacles. By using this algorithm all the paths will have L or | shape depending on the x and y coordinates.
		 *
		 * @param[in]  goal  The goal
		 *
		 * @return     the series of poses to move to reach the goal
		 */
		virtual std::vector<geometry_msgs::Pose2D> pathPlanning( geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D goal ) override;

	};

	DFS::DFS( int width, int height, bool is_edge_quartile )
	: DetPlanner2D<CellDFS>( width, height, is_edge_quartile )
	{
	}

	/**
	 * @brief      Constructs the object.
	 *
	 * @param      ogm               The ogm
	 * @param[in]  is_edge_quartile  Indicates if edge quartile
	 */
	DFS::DFS( nav_msgs::OccupancyGrid& ogm, bool is_edge_quartile )
	: DetPlanner2D<CellDFS>( ogm, is_edge_quartile )
	{
		// map_->loadMapFromOccGridMap( ogm );
	}

	// DFS planning
	std::vector<geometry_msgs::Pose2D> DFS::pathPlanning( geometry_msgs::Pose2D start, geometry_msgs::Pose2D goal )
	{
		multi_agent_plan::CellDFS* start_cell = map_->getCell( start.x, start.y );
		start_cell->is_visited_ = true;

		open_.push( start_cell );

		while( !open_.empty() )
		{
			// multi_agent_plan::CellDFS* cell = open_.front();
			multi_agent_plan::CellDFS* cell = open_.top();
			open_.pop();

			closed_.push( cell );

			std::vector<CellDFS*> neighbors = map_->expand( cell->pose_.x, cell->pose_.y );

			for (int i = 0; i < neighbors.size(); ++i)
			{
				CellDFS* tmp = neighbors[i];

				if ( !tmp->is_visited_ )
				{
					tmp->is_visited_ = true;
					tmp->parent_ = cell;

					if ( tmp->pose_.x == goal.x && tmp->pose_.y == goal.y )
					{
						return extractPath( tmp );
					}

					open_.push( tmp );
				}
			}
		}

		std::vector<geometry_msgs::Pose2D> failure;

		return failure;
	}
}

#endif