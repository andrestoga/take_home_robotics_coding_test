#ifndef PLANNER_2D_HPP
#define PLANNER_2D_HPP

#include "geometry_msgs/Pose2D.h"
#include "multi_agent_plan/GetPlan.h"
#include "multi_agent_plan/CurrPose.h"
#include "ros/ros.h"
#include "map.h"

namespace multi_agent_plan
{
	/**
	 * @brief      Pose in 2D
	 */
	struct Coord2D
	{
		float x_;
		float y_;
		float theta_;

		/**
		 * @brief      Constructor
		 *
		 * @param[in]  x      x coordinate
		 * @param[in]  y      y coordinate
		 * @param[in]  theta  theta angle
		 */
		Coord2D( float x, float y, float theta )
		: x_(x)
		, y_(y)
		, theta_(theta)
		{

		}
	};

	class Planner2D
	{
	public:
		Planner2D(const Map& grid)
		: grid_(grid.m_, grid.n_)
		{

		}

	    std::map<Coord2D, std::map<Coord2D, std::vector<geometry_msgs::Pose2D>>> saved_paths_;// Map used to store

	    /**
	     * 2D grid map
	     */
	    Map grid_;

		/**
		 * @brief      Pure function to be implemented in derived classes for planners in 2D
		 *
		 * @param[in]  start_pose  The start pose
		 * @param[in]  goal        The goal
		 *
		 * @return     The path to follow
		 */
	    virtual std::vector<geometry_msgs::Pose2D> pathPlanning( geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D goal ) = 0;
	};

}

#endif