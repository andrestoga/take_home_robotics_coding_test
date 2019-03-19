/* agent.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/agent.h"

#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

namespace multi_agent_plan
{
	Agent::Agent(const ros::NodeHandle& nh, geometry_msgs::Pose2D curr_pose, std::string serial_id, const Map& grid )
	: nh_(nh)
	, serial_id_(serial_id)
	, grid_(grid)
	{
		pose_pub_ = nh_.advertise<multi_agent_plan::CurrPose>("agent_feedback", 1000);
		goal_service_ = nh_.advertiseService("update_goal", &Agent::updateGoal, this );
		planner_client_ = nh_.serviceClient<multi_agent_plan::GetPlan>("/get_plan");
		path_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_path", 10);

		curr_pose_ = checkPose( curr_pose );
	}

	geometry_msgs::Pose2D Agent::checkPose( geometry_msgs::Pose2D pose )
	{
		if ( pose.x > grid_.n_ || pose.x < 0.0 )
		{
			pose.x = 0;	
		}

		if ( pose.y > grid_.n_ || pose.y < 0.0 )
		{
			pose.y = 0;	
		}

		if ( pose.theta > M_PI || pose.theta < -M_PI )
		{
			pose.theta = 0;	
		}

		return pose;
	}

	bool Agent::updateGoal( multi_agent_plan::UpdateGoal::Request  &req,
		multi_agent_plan::UpdateGoal::Response &res )
	{

		goal_pose_ = checkPose( req.new_goal );

		// Getting the path for the new goal.
		getPlan();

		ROS_INFO( "Making an agent to follow the path received from the Planner" );
		moveAgent();

		return true;
	}

	bool Agent::moveAgent()
	{
  		ros::Rate loop_rate(.1);
  		int i = 0;
				
  		while (ros::ok())
  		{
  			if ( i < last_path_.size() )
  			{
  				ROS_INFO_STREAM( "Agent moving every 10 seconds." );
  				ROS_INFO_STREAM( "Current position: " << last_path_[ i ] );
  				curr_pose_.x = last_path_[ i ].x;
  				curr_pose_.y = last_path_[ i ].y;
  				i++;
  			}
  			else
  			{
  				break;
  			}

    		loop_rate.sleep();
  		}

		return true;
	}

	void Agent::getPlan()
	{
		multi_agent_plan::GetPlan srv;

		srv.request.serial_id = serial_id_;	
		srv.request.goal_pose.x = goal_pose_.x;
		srv.request.goal_pose.y = goal_pose_.y;
		srv.request.goal_pose.theta = goal_pose_.theta;

		if (planner_client_.call(srv))
		{
			last_path_ = srv.response.path;
			ROS_INFO( "Plan received" );
		}
		else
		{
			ROS_ERROR("Failed to call service get_plan");
			return;
		}
	}

	void Agent::displayPathOnRviz()
	{
		visualization_msgs::Marker points;
		points.header.frame_id = "/path";
		points.header.stamp = ros::Time::now();
		points.ns = "path";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;

		points.id = 0;

		points.type = visualization_msgs::Marker::POINTS;

		points.scale.x = 0.2;
		points.scale.y = 0.2;

		// Points are green
		points.color.g = 1.0f;
		points.color.a = 1.0;

		for( auto m : last_path_ )
		{
			geometry_msgs::Point p;
			geometry_msgs::Pose2D tmp = transformPointsToWd( m, grid_.m_ );
			p.x = tmp.x;
			p.y = tmp.y;
			points.points.push_back( p );
		}

		path_pub_.publish(points);
	}

	geometry_msgs::Pose2D Agent::transformPointsToWd( geometry_msgs::Pose2D point,float offset )
	{
		geometry_msgs::Pose2D tmp;
		tmp.x = point.x - offset;
		tmp.y = point.y - offset;

		return tmp;
	}

	void Agent::publishCurrPose()
	{
		multi_agent_plan::CurrPose pub;

		pub.pose.x = curr_pose_.x;
		pub.pose.y = curr_pose_.y;
		pub.pose.theta = curr_pose_.theta;

		geometry_msgs::Pose2D pose_snd = transformPointsToWd( curr_pose_, grid_.m_ );

		static tf2_ros::TransformBroadcaster br;
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "path";
		transformStamped.child_frame_id = serial_id_;
		transformStamped.transform.translation.x = pose_snd.x;
		transformStamped.transform.translation.y = pose_snd.y;
		transformStamped.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, curr_pose_.theta);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();

		br.sendTransform(transformStamped);
		pose_pub_.publish( pub );
	}
}