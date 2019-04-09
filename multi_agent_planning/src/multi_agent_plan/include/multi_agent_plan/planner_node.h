/* planner.h
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#ifndef PLANNER_NODE_H
#define PLANNER_NODE_H

#include "geometry_msgs/Pose2D.h"
#include "multi_agent_plan/GetPlan.h"
#include "multi_agent_plan/CurrPose.h"
#include "ros/ros.h"
#include "map.h"
#include "multi_agent_plan/manhattan_planner.hpp"

namespace multi_agent_plan
{
		class PlannerNode
		{
			geometry_msgs::Pose2D start_pose_;

			ros::ServiceServer plan_service_;
			ros::Subscriber curr_pose_sub_;

			/**
			 * @brief      Calculates the path.
			 *
			 * @param      req   The request: serial id and the goal pose
			 * @param      res   The response: the path
			 *
			 * @return     Return true
			 */
			bool computePath(multi_agent_plan::GetPlan::Request  &req,
			 multi_agent_plan::GetPlan::Response &res);

			bool is_curr_pose_ready_;

			/**
			 * @brief      Gets the agent pose.
			 *
			 * @param[in]  msg   The message containing the agent pose
			 */
			void getAgentPose(const multi_agent_plan::CurrPose::ConstPtr& msg);

		public:

			ros::NodeHandle nh_;

			std::shared_ptr<ManhattanPlanner> pl_;

		  	/**
		  	 * @brief      Constructs the object.
		  	 *
		  	 * @param[in]  nh    node handle
		  	 * @param[in]  grid  The grid
		  	 */
		    PlannerNode(const ros::NodeHandle& nh, const Map& grid);
		};
}

#endif