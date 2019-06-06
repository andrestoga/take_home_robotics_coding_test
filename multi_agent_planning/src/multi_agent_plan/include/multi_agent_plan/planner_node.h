/* planner.h
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#ifndef PLANNER_NODE_H
#define PLANNER_NODE_H

#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"
#include "multi_agent_plan/GetPlan.h"
#include "multi_agent_plan/Pose.h"
#include "multi_agent_plan/planner_2d.h"
#include "ros/ros.h"
#include <memory>

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
	
	template< typename T >
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

		bool is_agent_pose_ready_;

			/**
			 * @brief      Gets the agent pose.
			 *
			 * @param[in]  msg   The message containing the agent pose
			 */
		void getAgentPose(const multi_agent_plan::Pose::ConstPtr& msg);

		bool isValidPose( geometry_msgs::Pose2D pose );


	public:

		ros::NodeHandle nh_;

		std::unique_ptr<T> pl_;

		  	/**
		  	 * @brief      Constructs the object.
		  	 *
		  	 * @param[in]  nh    node handle
		  	 */
		PlannerNode(const ros::NodeHandle& nh);
	};

	template< typename T >
	PlannerNode<T>::PlannerNode(const ros::NodeHandle& nh)
	: nh_(nh)
	, is_agent_pose_ready_(false)
	{
		plan_service_ = nh_.advertiseService("get_plan", &PlannerNode::computePath, this );

		int width;
		int height;
		bool is_server_map;

		ros::param::param<bool>("is_server_map", is_server_map, true);

		if ( is_server_map )
		{
			nav_msgs::OccupancyGrid ogm;
			bool is_map_ready = false;

			ROS_INFO( "Loading map...." );

			auto sub_map = nh_.subscribe<nav_msgs::OccupancyGrid>( "/map", 1, [ &ogm, &is_map_ready ]( const nav_msgs::OccupancyGrid::ConstPtr& msg )
			{
				ROS_DEBUG( "Getting map" );
				ogm = *msg;
				is_map_ready = true;
			});

			while( !is_map_ready && ros::ok() )
				ros::spinOnce();

			is_map_ready = false;
			pl_ = std::unique_ptr<T>( new T( ogm ) );

			ROS_INFO( "Map successfully loaded...." );
		}
		else
		{
			ROS_INFO( "Creating generic map" );

			ros::param::param<int>("/width", width, 10);
			ros::param::param<int>("/height", height, 10);

			pl_ = std::unique_ptr<T>( new T( width, height ) );
		}

		// if( ros::param::get( "is_server_map", width ) )
		// {
		// 	ROS_DEBUG( "param ok!" );
		// }
		// else
		// {
		// 	ROS_ERROR("Failed to get param");
		// }

		// if( ros::param::get( "/width", width ) )
		// {
		// 	ROS_DEBUG( "param ok!" );
		// }
		// else
		// {
		// 	ROS_ERROR("Failed to get param");
		// }

		// if( ros::param::get( "/height", height ) )
		// {
		// 	ROS_DEBUG( "param ok!" );
		// }
		// else
		// {
		// 	ROS_ERROR("Failed to get param");
		// }		
	}

	template< typename T >
	bool PlannerNode<T>::computePath(multi_agent_plan::GetPlan::Request  &req,
		multi_agent_plan::GetPlan::Response &res)
	{
		std::stringstream agent_topic;

		agent_topic << "/" << req.serial_id << "/" << "agent_feedback";

		ROS_INFO_STREAM( "Topic to subscribe: " + agent_topic.str() );

		// Subscribe to the correct topic of the agent to get its current pose.
		curr_pose_sub_ = nh_.subscribe( agent_topic.str(), 1, &PlannerNode::getAgentPose, this);

		ROS_INFO_STREAM( "Checking topic subscribed: " << curr_pose_sub_.getTopic() );

		while( !is_agent_pose_ready_ )
			ros::spinOnce();

		if ( !isValidPose( start_pose_ ) )
		{
			ROS_DEBUG( "Invalid start pose" );
			ROS_DEBUG_STREAM( "X: " << start_pose_.x << " Y: " << start_pose_.y << " Theta: " << start_pose_.theta );
			return false;
		}

		if ( !isValidPose( req.goal_pose ) )
		{
			ROS_DEBUG( "Invalid goal pose" );
			ROS_DEBUG_STREAM( "Y: " << req.goal_pose.x << " Y: " << req.goal_pose.y << " Theta: " << start_pose_.theta );
			return false;
		}

		is_agent_pose_ready_ = false;
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

	template< typename T >
	bool PlannerNode<T>::isValidPose( geometry_msgs::Pose2D pose )
	{
		if ( pose.x > pl_->width_ || pose.x < 0.0 )
		{
			return false;
		}

		if ( pose.y > pl_->height_ || pose.y < 0.0 )
		{
			return false;
		}

		if ( pose.theta > M_PI || pose.theta < -M_PI )
		{
			return false;
		}

		return true;
	}

	template< typename T >
	void PlannerNode<T>::getAgentPose(const multi_agent_plan::Pose::ConstPtr& msg)
	{
		start_pose_.x = msg->pose.x;
		start_pose_.y = msg->pose.y;
		start_pose_.theta = msg->pose.theta;

		is_agent_pose_ready_ = true;
	}
}

#endif