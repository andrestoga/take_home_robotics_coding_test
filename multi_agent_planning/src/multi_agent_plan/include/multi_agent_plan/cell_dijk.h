/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-22 11:02:41
 */

#ifndef CELL_DIJK_H
#define CELL_DIJK_H

#include "ros/ros.h"
#include "cell.h"

namespace multi_agent_plan
{
	class CellDijk : public Cell
	{
	public:
		float g_;

		CellDijk( geometry_msgs::Pose2D pose, multi_agent_plan::CellDijk* parent = nullptr, float g = -1.0 )
		: Cell( pose, parent )
		, g_( g )
		{
			
		}

		CellDijk( float g = -1.0 )
		: g_( g )
		{}
	};
}

#endif