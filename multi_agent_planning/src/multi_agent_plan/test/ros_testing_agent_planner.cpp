/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-09 18:49:57
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <geometry_msgs/Pose2D.h>
#include <gtest/gtest.h>
#include "multi_agent_plan/GetPlan.h"
#include "multi_agent_plan/UpdateGoal.h"
#include "multi_agent_plan/CurrPose.h"

#include <std_msgs/Bool.h>


// helper function to create a geometry_msg::Vector3
auto createPose2D = [](double x, double y, double theta) {
  geometry_msgs::Pose2D v;
  v.x = x;
  v.y = y;
  v.theta = theta;
  return v;
};

class AgentFixture : public ::testing::Test 
{
public:
	AgentFixture()
	{
		nh_.reset( new ros::NodeHandle );
		plan_client_ = nh_->serviceClient<multi_agent_plan::GetPlan>( "/get_plan" );
		agent_goal_client_ = nh_->serviceClient<multi_agent_plan::UpdateGoal>( "update_goal" );
		agent_update_pose_client_ = nh_->serviceClient<multi_agent_plan::UpdateGoal>( "update_pose" );

		// setInitPose();
	}

	std::unique_ptr<ros::NodeHandle> nh_;
	ros::ServiceClient plan_client_;
	ros::ServiceClient agent_goal_client_;
	ros::ServiceClient agent_update_pose_client_;

// private:

// 	void setInitPose()
// 	{
// 		multi_agent_plan::UpdateGoal srv_update_pose;

// 		geometry_msgs::Pose2D pose = createPose2D( 0.0, 2.0, 0.0 );
// 		srv_update_pose.request.new_goal = pose;

// 		if (!agent_update_pose_client_.call( srv_update_pose ))
// 		{
// 			ROS_ERROR( "Couldn't call update_pose service" );
// 		}
// 	}
};

TEST_F( AgentFixture, testAgent )
{
	bool exists_agent_update( agent_update_pose_client_.waitForExistence( ros::Duration( 1 ) ) );
	ASSERT_TRUE( exists_agent_update );

	multi_agent_plan::UpdateGoal srv_update_pose;

	geometry_msgs::Pose2D pose = createPose2D( 0.0, 2.0, 0.0 );
	srv_update_pose.request.new_goal = pose;

	if (!agent_update_pose_client_.call( srv_update_pose ))
	{
		ADD_FAILURE();
	}

	bool exists_agent_goal( agent_goal_client_.waitForExistence( ros::Duration( 1 ) ) );
	ASSERT_TRUE( exists_agent_goal );

	multi_agent_plan::UpdateGoal srv_agent_goal;

	geometry_msgs::Pose2D goal = createPose2D( 4.0, 6.0, 0.0 );
	srv_agent_goal.request.new_goal = goal;

	if (!agent_goal_client_.call( srv_agent_goal ))
	{
		ADD_FAILURE();
	}

	bool exists_plan( plan_client_.waitForExistence( ros::Duration( 1 ) ) );
	ASSERT_TRUE( exists_plan );

	geometry_msgs::Pose2D agnt_pose;
	bool is_pose_received = false;
	bool is_moving = true;

	auto sub_pose = nh_->subscribe<multi_agent_plan::CurrPose>(
		"agent_feedback", 1, [&agnt_pose, &is_pose_received](const multi_agent_plan::CurrPose::ConstPtr& msg)
	{
		agnt_pose.x = msg->pose.x;
		agnt_pose.y = msg->pose.y;
		agnt_pose.theta = msg->pose.theta;
		is_pose_received = true;
	});

	auto sub_feed = nh_->subscribe<std_msgs::Bool>(
		"is_moving", 1, [&is_moving](const std_msgs::Bool::ConstPtr& msg)
	{
		is_moving = msg->data;
	});

	while( !is_pose_received || is_moving )
		ros::spinOnce();

	EXPECT_EQ( goal.x, agnt_pose.x );
	EXPECT_EQ( goal.y, agnt_pose.y );
	EXPECT_EQ( goal.theta, agnt_pose.theta );
}

TEST_F( AgentFixture, testPlanner )
{
	bool exists_agent_update( agent_update_pose_client_.waitForExistence( ros::Duration( 1 ) ) );
	ASSERT_TRUE( exists_agent_update );

	multi_agent_plan::UpdateGoal srv_update_pose;

	geometry_msgs::Pose2D pose = createPose2D( 0.0, 2.0, 0.0 );
	srv_update_pose.request.new_goal = pose;

	if (!agent_update_pose_client_.call( srv_update_pose ))
	{
		ADD_FAILURE();
	}

	bool exists_agent( agent_goal_client_.waitForExistence( ros::Duration( 1 ) ) );
	ASSERT_TRUE( exists_agent );

	bool exists_plan( plan_client_.waitForExistence( ros::Duration( 1 ) ) );
	ASSERT_TRUE( exists_plan );

	multi_agent_plan::GetPlan srv_plan;

	srv_plan.request.serial_id = "agent_1";
	srv_plan.request.goal_pose = createPose2D( 4.0, 6.0, 0.0 );

	if (!plan_client_.call( srv_plan ))
	{
		ADD_FAILURE();
	}

	std::vector<geometry_msgs::Pose2D> test;

	test.push_back( createPose2D( 0.0, 2.0, 0.0 ) );
	test.push_back( createPose2D( 1.0, 2.0, 0.0 ) );
	test.push_back( createPose2D( 2.0, 2.0, 0.0 ) );
	test.push_back( createPose2D( 3.0, 2.0, 0.0 ) );
	test.push_back( createPose2D( 4.0, 2.0, 0.0 ) );
	test.push_back( createPose2D( 4.0, 3.0, 0.0 ) );
	test.push_back( createPose2D( 4.0, 4.0, 0.0 ) );
	test.push_back( createPose2D( 4.0, 5.0, 0.0 ) );
	test.push_back( createPose2D( 4.0, 6.0, 0.0 ) );

	ASSERT_EQ( test.size(), srv_plan.response.path.size() );

	for (int i = 0; i < test.size(); ++i)
	{
		EXPECT_EQ( test[ i ].x, srv_plan.response.path[ i ].x );
		EXPECT_EQ( test[ i ].y, srv_plan.response.path[ i ].y );
		EXPECT_EQ( test[ i ].theta, srv_plan.response.path[ i ].theta );
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_testing_agent_planner");
  // nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}