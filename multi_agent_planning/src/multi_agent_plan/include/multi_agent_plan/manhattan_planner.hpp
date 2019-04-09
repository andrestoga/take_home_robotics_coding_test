#ifndef MANHATTAN_PLANNER_HPP
#define MANHATTAN_PLANNER_HPP

#include "geometry_msgs/Pose2D.h"
#include "multi_agent_plan/GetPlan.h"
#include "multi_agent_plan/CurrPose.h"
#include "ros/ros.h"
#include "map.h"

#include "multi_agent_plan/planner_2d.hpp"

namespace multi_agent_plan
{
	class ManhattanPlanner: public Planner2D
	{
	public:
		ManhattanPlanner( const Map& grid );

		/**
		 * @brief      Plan for manhattan worlds with no obstacles. By using this algorithm all the paths will have L or | shape depending on the x and y coordinates.
		 *
		 * @param[in]  goal  The goal
		 *
		 * @return     the series of poses to move to reach the goal
		 */
	    virtual std::vector<geometry_msgs::Pose2D> pathPlanning( geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D goal ) override;

	};
}

#endif