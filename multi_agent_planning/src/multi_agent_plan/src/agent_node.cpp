/* agent_node.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/agent.h"
#include "multi_agent_plan/agent_node.h"
// #include <multi_agent_plan/map.h>
#include "multi_agent_plan/map.h"

#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <math.h>

namespace multi_agent_plan
{
  AgentNode::AgentNode( const ros::NodeHandle& nh )
  : nh_(nh)
  , is_agent_moving_(false)
  , i_path_(1)
  , move_duration_( 1.0 )
  {
    pose_pub_ = nh_.advertise<multi_agent_plan::CurrPose>("agent_feedback", 1000);
    moving_pub_ = nh_.advertise<std_msgs::Bool>("is_moving", 1000);
    goal_service_ = nh_.advertiseService("update_goal", &AgentNode::updateGoal, this );
    update_pose_srv_ = nh_.advertiseService("update_pose", &AgentNode::updatePose, this );
    planner_client_ = nh_.serviceClient<multi_agent_plan::GetPlan>("/get_plan");
    path_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_path", 10);

    std::string serial_id = "test_1";
    geometry_msgs::Pose2D pose;

    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;

    if( ros::param::get( "~serial_id", serial_id ) )
    {
      ROS_INFO( "param ok! %s", serial_id.c_str() );
    }
    else
    {
      ROS_ERROR("Failed to get param");
    }

    if( ros::param::get( "~x", pose.x ) )
    {
      ROS_INFO( "param ok! %f", pose.x );
    }
    else
    {
      ROS_ERROR("Failed to get param");
    }

    if( ros::param::get( "~y", pose.y ) )
    {
      ROS_INFO( "param ok! %f", pose.y );
    }
    else
    {
      ROS_ERROR("Failed to get param");
    }

    if( ros::param::get( "~theta", pose.theta ) )
    {
      ROS_INFO( "param ok! %f", pose.theta );
    }
    else
    {
      ROS_ERROR("Failed to get param");
    }

    Map map( 5, 10 );
    ag_ = std::unique_ptr<Agent>( new Agent( pose, serial_id, map ) );
  }

  bool AgentNode::updatePose( multi_agent_plan::UpdateGoal::Request  &req, multi_agent_plan::UpdateGoal::Response &res )
  {
    ag_->curr_pose_.x = req.new_goal.x;
    ag_->curr_pose_.y = req.new_goal.y;
    ag_->curr_pose_.theta = req.new_goal.theta;

    return true;
  }

  bool AgentNode::updateGoal( multi_agent_plan::UpdateGoal::Request  &req,
    multi_agent_plan::UpdateGoal::Response &res )
  {
    ag_->goal_pose_ = ag_->checkPose( req.new_goal );

    // Getting the path for the new goal.
    getPlan();

    ROS_INFO( "Making an agent to follow the path received from the Planner" );

    is_agent_moving_ = true;
    move_start_time_ = ros::Time::now();

    return true;
  }

  void AgentNode::getPlan()
  {
    multi_agent_plan::GetPlan srv;

    srv.request.serial_id = ag_->serial_id_; 
    srv.request.goal_pose.x = ag_->goal_pose_.x;
    srv.request.goal_pose.y = ag_->goal_pose_.y;
    srv.request.goal_pose.theta = ag_->goal_pose_.theta;

    if (planner_client_.call(srv))
    {
      ag_->last_path_ = srv.response.path;
      ROS_INFO_STREAM( "Plan received with size: " << ag_->last_path_.size() );
    }
    else
    {
      ROS_ERROR("Failed to call service get_plan");
      return;
    }
  }

  bool AgentNode::followPathPlan()
  {
    if (is_agent_moving_)
    {
      //Checking if it has passed the duration of the rotation.
      if((ros::Time::now().sec - move_start_time_.sec) >= move_duration_.sec)
      {
        if ( i_path_ < ag_->last_path_.size() )
        {
          ROS_INFO( "Agent moving every %f seconds.", move_duration_.toSec() );
          ROS_INFO_STREAM( "Current position: " << ag_->last_path_[ i_path_ ] );
          ag_->curr_pose_.x = ag_->last_path_[ i_path_ ].x;
          ag_->curr_pose_.y = ag_->last_path_[ i_path_ ].y;
          i_path_++;
        }
        else
        {
          is_agent_moving_ = false;
          i_path_ = 1;
        }

        move_start_time_ = ros::Time::now();
      }
    }
    return true;
  }

  void AgentNode::displayPathOnRviz()
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

    for( auto m : ag_->last_path_ )
    {
      geometry_msgs::Point p;
      geometry_msgs::Pose2D tmp = ag_->transformPointsToWd( m, ag_->grid_.m_ );
      p.x = tmp.x;
      p.y = tmp.y;
      points.points.push_back( p );
    }

    path_pub_.publish(points);
  }

  void AgentNode::publishCurrPose()
  {
    multi_agent_plan::CurrPose pub;

    pub.pose.x = ag_->curr_pose_.x;
    pub.pose.y = ag_->curr_pose_.y;
    pub.pose.theta = ag_->curr_pose_.theta;

    geometry_msgs::Pose2D pose_snd = ag_->transformPointsToWd( ag_->curr_pose_, ag_->grid_.m_ );

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "path";
    transformStamped.child_frame_id = ag_->serial_id_;
    transformStamped.transform.translation.x = pose_snd.x;
    transformStamped.transform.translation.y = pose_snd.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, ag_->curr_pose_.theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
    pose_pub_.publish( pub );
  }

  void AgentNode::update()
  {
    publishCurrPose();
    followPathPlan();
    displayPathOnRviz();

    if ( is_agent_moving_ )
    {
      std_msgs::Bool b;
      b.data = true;
      moving_pub_.publish( b );
    }
    else
    {
     std_msgs::Bool b;
      b.data = false;
      moving_pub_.publish( b );
    }
  }

  // void updatePose()
  // {
  //   ros::param::param<std::Float32>("pose/x", ag_->curr_pose_.x, 0.0);
  //   ros::param::param<std::Float32>("pose/y", ag_->curr_pose_.y, 0.0);
  //   ros::param::param<std::Float32>("pose/theta", ag_->curr_pose_.theta, 0.0);
  // }
}