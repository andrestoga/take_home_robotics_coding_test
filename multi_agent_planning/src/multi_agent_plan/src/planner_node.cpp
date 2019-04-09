/* planner.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/planner_node.h"

namespace multi_agent_plan
{
	/**
	 * @brief      less than, overload to make it work in the data structure map
	 *
	 * @param[in]  l     left object
	 * @param[in]  r     right object
	 *
	 * @return     true if left is less than right, otherwise false.
	 */
	bool operator<(const Coord2D& l, const Coord2D& r)
	{
	    return ( l.x_ < r.x_ || ( l.x_ == r.x_ && l.y_ < r.y_ ) );
	}

	PlannerNode::PlannerNode(const ros::NodeHandle& nh, const Map& grid)
	: nh_(nh)
	, is_curr_pose_ready_(false)
	{
		plan_service_ = nh_.advertiseService("get_plan", &PlannerNode::computePath, this );
		pl_ = std::make_shared<ManhattanPlanner>( grid );
	}

	bool PlannerNode::computePath(multi_agent_plan::GetPlan::Request  &req,
         multi_agent_plan::GetPlan::Response &res)
	{
		std::stringstream agent_topic;

		agent_topic << "/" << req.serial_id << "/" << "agent_feedback";

		ROS_INFO_STREAM( "Topic to subscribe: " + agent_topic.str() );

		// Subscribe to the correct topic of the agent to get its current pose.
		curr_pose_sub_ = nh_.subscribe( agent_topic.str(), 1, &PlannerNode::getAgentPose, this);

		ROS_INFO_STREAM( "Checking topic subscribed: " << curr_pose_sub_.getTopic() );

		while( !is_curr_pose_ready_ )
			ros::spinOnce();

		is_curr_pose_ready_ = false;
		curr_pose_sub_.shutdown();

		ROS_INFO_STREAM( "Start pose: " << start_pose_ );
		ROS_INFO_STREAM( "Goal pose: " << req.goal_pose );

		ROS_INFO( "Calculating path for %s", req.serial_id.c_str() );

		Coord2D start_pose( start_pose_.x, start_pose_.y, start_pose_.theta );
		Coord2D goal_pose( req.goal_pose.x, req.goal_pose.y, req.goal_pose.theta );

		// Check if we already calculated this path
		if ( pl_->saved_paths_.find( start_pose ) != pl_->saved_paths_.end() )
		{
			std::map<Coord2D, std::vector<geometry_msgs::Pose2D>> goal_map = pl_->saved_paths_[ start_pose ];

			// If yes, reuse the path
			if(goal_map.find( goal_pose ) != goal_map.end())
			{
				ROS_INFO_STREAM( "Reusing path!" );
				res.path = goal_map[ goal_pose ];
				return true;
			}
		}

		// If not compute the path
		res.path = pl_->pathPlanning( start_pose_, req.goal_pose );
		// and store it for future use
		pl_->saved_paths_[ start_pose ][ goal_pose ] = res.path;

		return true;
	}

	void PlannerNode::getAgentPose(const multi_agent_plan::CurrPose::ConstPtr& msg)
	{
		start_pose_.x = msg->pose.x;
		start_pose_.y = msg->pose.y;
		start_pose_.theta = msg->pose.theta;

		is_curr_pose_ready_ = true;
	}
}