/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-25 21:48:40
 */

#ifndef DET_PLANNER_2D_H
#define DET_PLANNER_2D_H

#include "ros/ros.h"
#include "planner_2d.h"
#include "multi_agent_plan/map.h"
#include <memory>

namespace multi_agent_plan
{
	template< typename T >
	class DetPlanner2D : public Planner2D
	{
	public:

		std::unique_ptr<Map<T>> map_;

		DetPlanner2D( int width = 10, int height = 10, bool is_edge_quartile = true )
		: Planner2D( width, height )
		{
			map_ = std::unique_ptr<Map<T>>( new Map<T>( width, height, is_edge_quartile ) );
		}

		DetPlanner2D( nav_msgs::OccupancyGrid& ogm, bool is_edge_quartile = true )
		: Planner2D( ogm.info.width, ogm.info.height )
		{
			map_ = std::unique_ptr<Map<T>>( new Map<T>( ogm, is_edge_quartile ) );
		}

		std::vector<geometry_msgs::Pose2D> extractPath( Cell* goal )
		{
			std::vector<geometry_msgs::Pose2D> path;

			while( goal != nullptr )
			{
				geometry_msgs::Pose2D tmp = goal->pose_;
				path.push_back( tmp );

				goal = goal->parent_;
			}

			std::reverse( path.begin(), path.end() );

			return path;
		}
	};
}

#endif