/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-02 19:49:34
 */
#include "gtest/gtest.h"
#include "multi_agent_plan/agent.h"
#include "multi_agent_plan/manhattan_planner.hpp"
#include "multi_agent_plan/map.h"

auto createPose2D = [](double x, double y, double theta) {
  geometry_msgs::Pose2D p;
  p.x = x;
  p.y = y;
  p.theta = theta;
  return p;
};

TEST(MultiAgent, limits)
{
	multi_agent_plan::Map map( 10, 10 );
	multi_agent_plan::Agent ag_1( multi_agent_plan::Agent( createPose2D( -1.0, -20.0, -4.0 ), "agent_1", map ) );
	multi_agent_plan::Agent ag_2( multi_agent_plan::Agent( createPose2D( 20.0, 20.0, 4.0 ), "agent_2", map ) );

	EXPECT_EQ( ag_1.curr_pose_.x, 0.0 );
	EXPECT_EQ( ag_1.curr_pose_.y, 0.0 );
	EXPECT_EQ( ag_1.curr_pose_.theta, 0.0 );

	EXPECT_EQ( ag_2.curr_pose_.x, 0.0 );
	EXPECT_EQ( ag_2.curr_pose_.y, 0.0 );
	EXPECT_EQ( ag_2.curr_pose_.theta, 0.0 );
}

TEST(MultiAgent, pathManhattan)
{
	multi_agent_plan::Map map( 10, 10 );
	multi_agent_plan::ManhattanPlanner planner( map );

	std::vector<geometry_msgs::Pose2D> plan1 = planner.pathPlanning( createPose2D( 0.0, 2.0, 0.0 ), createPose2D( 0.0, 6.0, 0.0 ) );
	std::vector<geometry_msgs::Pose2D> plan2 = planner.pathPlanning( createPose2D( 2.0, 0.0, 0.0 ), createPose2D( 6.0, 0.0, 0.0 ) );
	std::vector<geometry_msgs::Pose2D> plan3 = planner.pathPlanning( createPose2D( 0.0, 2.0, 0.0 ), createPose2D( 4.0, 6.0, 0.0 ) );

	std::vector<geometry_msgs::Pose2D> test1;

	test1.push_back( createPose2D( 0.0, 2.0, 0.0 ) );
	test1.push_back( createPose2D( 0.0, 3.0, 0.0 ) );
	test1.push_back( createPose2D( 0.0, 4.0, 0.0 ) );
	test1.push_back( createPose2D( 0.0, 5.0, 0.0 ) );
	test1.push_back( createPose2D( 0.0, 6.0, 0.0 ) );

	ASSERT_EQ( plan1.size(), test1.size() );

	for (int i = 0; i < test1.size(); ++i)
	{
		EXPECT_EQ( plan1[ i ].x, test1[ i ].x );
		EXPECT_EQ( plan1[ i ].y, test1[ i ].y );
		EXPECT_EQ( plan1[ i ].theta, test1[ i ].theta );
	}

	std::vector<geometry_msgs::Pose2D> test2;

	test2.push_back( createPose2D( 2.0, 0.0, 0.0 ) );
	test2.push_back( createPose2D( 3.0, 0.0, 0.0 ) );
	test2.push_back( createPose2D( 4.0, 0.0, 0.0 ) );
	test2.push_back( createPose2D( 5.0, 0.0, 0.0 ) );
	test2.push_back( createPose2D( 6.0, 0.0, 0.0 ) );

	ASSERT_EQ( plan2.size(), test2.size() );

	for (int i = 0; i < test2.size(); ++i)
	{
		EXPECT_EQ( plan2[ i ].x, test2[ i ].x );
		EXPECT_EQ( plan2[ i ].y, test2[ i ].y );
		EXPECT_EQ( plan2[ i ].theta, test2[ i ].theta );
	}

	std::vector<geometry_msgs::Pose2D> test3;

	test3.push_back( createPose2D( 0.0, 2.0, 0.0 ) );
	test3.push_back( createPose2D( 1.0, 2.0, 0.0 ) );
	test3.push_back( createPose2D( 2.0, 2.0, 0.0 ) );
	test3.push_back( createPose2D( 3.0, 2.0, 0.0 ) );
	test3.push_back( createPose2D( 4.0, 2.0, 0.0 ) );
	test3.push_back( createPose2D( 4.0, 3.0, 0.0 ) );
	test3.push_back( createPose2D( 4.0, 4.0, 0.0 ) );
	test3.push_back( createPose2D( 4.0, 5.0, 0.0 ) );
	test3.push_back( createPose2D( 4.0, 6.0, 0.0 ) );

	ASSERT_EQ( plan3.size(), test3.size() );

	for (int i = 0; i < test3.size(); ++i)
	{
		EXPECT_EQ( plan3[ i ].x, test3[ i ].x );
		EXPECT_EQ( plan3[ i ].y, test3[ i ].y );
		EXPECT_EQ( plan3[ i ].theta, test3[ i ].theta );
	}
}

int main(int argc, char **argv)
{
  	testing::InitGoogleTest(&argc, argv);
 //  	ros::init(argc, argv, "test_multi_agent_plan");
	// ros::NodeHandle nh;
  
  	return RUN_ALL_TESTS();
}