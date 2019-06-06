#ifndef PLANNER_2D_H
#define PLANNER_2D_H

#include "geometry_msgs/Pose2D.h"
#include "multi_agent_plan/GetPlan.h"
#include "multi_agent_plan/Pose.h"
#include "ros/ros.h"
#include "cell.h"

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

		int width_;
		int height_;

		Planner2D( int width, int height )
		: width_( width )
		, height_( height )
		{

		}

		Planner2D(){}

		virtual ~Planner2D()
		{}
		
	    std::map<Coord2D, std::map<Coord2D, std::vector<geometry_msgs::Pose2D>>> saved_paths_;// Map used to store previous queries.

	    /**
	     * 2D grid map
	     */

	    /**
	     * grid_ Map to plan
	     */
	    // Map<T> grid_;

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