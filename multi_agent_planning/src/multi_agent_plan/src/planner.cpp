/* planner.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/planner.h"


namespace multi_agent_plan
{
	// bool operator<(const geometry_msgs::Pose2D& l, const geometry_msgs::Pose2D& r)
	// {
	//     return ( l.x < r.x || ( l.x == r.x && l.y < r.y ) );
	// }
	
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

	Planner::Planner(const ros::NodeHandle& nh, const Map& grid)
	: nh_(nh)
	, grid_(grid)
	, is_curr_pose_ready_(false)
	{
		plan_service_ = nh_.advertiseService("get_plan", &Planner::computePath, this );
	}

	bool Planner::computePath(multi_agent_plan::GetPlan::Request  &req,
         multi_agent_plan::GetPlan::Response &res)
	{
		std::stringstream agent_topic;

		agent_topic << "/" << req.serial_id << "/" << "agent_feedback";

		ROS_INFO_STREAM( "Topic to subscribe: " + agent_topic.str() );

		// Subscribe to the correct topic of the agent to get its current pose.
		curr_pose_sub_ = nh_.subscribe( agent_topic.str(), 1, &Planner::getAgentPose, this);

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
		if ( saved_paths_.find( start_pose ) != saved_paths_.end() )
		{
			std::map<Coord2D, std::vector<geometry_msgs::Pose2D>> goal_map = saved_paths_[ start_pose ];

			// If yes, reuse the path
			if(goal_map.find( goal_pose ) != goal_map.end())
			{
				ROS_INFO_STREAM( "Reusing path!" );
				res.path = goal_map[ goal_pose ];
				return true;
			}
		}

		// If not compute the path
		res.path = manhattanPlanning( req.goal_pose );
		// and store it for future use
		saved_paths_[ start_pose ][ goal_pose ] = res.path;

		return true;
	}

	std::vector<geometry_msgs::Pose2D> Planner::manhattanPlanning( geometry_msgs::Pose2D goal )
	{
		std::vector<geometry_msgs::Pose2D> path;

		std::vector<int> xs;
		std::vector<int> ys;

		// Calculating the size of the x and y coordinates
		int x_size = std::abs( start_pose_.x - goal.x ) + 1;
		int y_size = std::abs( start_pose_.y - goal.y ) + 1;

		xs.resize( x_size );
		ys.resize( y_size );
		path.resize( x_size + y_size - 1 );

		// Checking if they are changes in the x coordinate
		if ( start_pose_.x > goal.x )
		{
			std::iota(xs.begin(), xs.end(), goal.x);
			std::reverse(xs.begin(), xs.end());
		}
		else if ( start_pose_.x < goal.x )
		{
			std::iota(xs.begin(), xs.end(), start_pose_.x );
		}
		else
		{
			xs[ 0 ] = start_pose_.x;// No changes in x coordinate
		}

		// for(auto m : xs)
		// {
		// 	std::cout << m << " ";
		// }

		// std::cout << std::endl;

		// Checking if they are changes in the y coordinate
		if ( start_pose_.y > goal.y )
		{
			std::iota(ys.begin(), ys.end(), goal.y);
			std::reverse(ys.begin(), ys.end());
		}
		else if ( start_pose_.y < goal.y )
		{
			std::iota(ys.begin(), ys.end(), start_pose_.y);
		}
		else
		{
			ys[ 0 ] = start_pose_.y;// No changes in y coordinate
		}

		// for(auto m : ys)
		// {
		// 	std::cout << m << " ";
		// }

		// std::cout << std::endl;

		// Adding the x coordinates in the path by keeping fix the y coordinate
		for (int i = 0; i < xs.size(); ++i)
		{
			path[ i ].x = xs[ i ];
			path[ i ].y = start_pose_.y;
		}

		// Adding the y coordinates in the path by keeping fix the x coordinate
		for (int i = 1; i < ys.size(); ++i)
		{
			path[ x_size + i - 1 ].x = path[ x_size - 1 ].x;
			path[ x_size + i - 1 ].y = ys[ i ];	
		}

		// ROS_INFO_STREAM( "Size of the path: " << path.size() );

		return path;
	}

	void Planner::getAgentPose(const multi_agent_plan::CurrPose::ConstPtr& msg)
	{
		start_pose_.x = msg->pose.x;
		start_pose_.y = msg->pose.y;
		start_pose_.theta = msg->pose.theta;

		is_curr_pose_ready_ = true;
	}
}