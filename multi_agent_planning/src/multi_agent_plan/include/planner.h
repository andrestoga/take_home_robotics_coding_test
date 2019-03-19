#ifndef AGENT_H
#define AGENT_H

#include "geometry_msgs/Pose2D.h"
#include "multi_agent_plan/GetPlan.h"
#include "multi_agent_plan/CurrPose.h"
#include "ros/ros.h"
#include "map.h"

namespace multi_agent_plan
{
	class Planner
	{

		// bool is_curr_pose_ready_;
		bool is_curr_

		// Access specifier 
		public:

	    // Data Members
	    Map grid_;

	    geometry_msgs::Pose2D start_pose_;

		ros::ServiceServer plan_service_;
		ros::Subscriber curr_pose_sub_;
		ros::NodeHandle nh_;
	  
	    // Member Functions() 
	    Planner(const ros::NodeHandle& nh, const Map& grid);
	    bool computePath(multi_agent_plan::GetPlan::Request  &req,
         multi_agent_plan::GetPlan::Response &res);
	    void getAgentPose(const multi_agent_plan::CurrPose::ConstPtr& msg);
		std::vector<geometry_msgs::Pose2D> manhattanPlanning( geometry_msgs::Pose2D goal );
	};
}

#endif