/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-04-02 19:49:34
 */
#include "gtest/gtest.h"
#include "multi_agent_plan/manhattan_planner.h"
#include "multi_agent_plan/dfs.h"
#include "multi_agent_plan/dijkstra.h"
#include "map_server/image_loader.h"
#include "nav_msgs/GetMap.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "multi_agent_plan/cell_dfs.h"

auto createPose2D = [](double x, double y, double theta) {
	geometry_msgs::Pose2D p;
	p.x = x;
	p.y = y;
	p.theta = theta;
	return p;
};

const char g_valid_image_content[] = {
	0,0,0,0,0,0,0,0,0,0,
	100,100,100,100,0,0,100,100,100,0,
	100,100,100,100,0,0,100,100,100,0,
	100,0,0,0,0,0,0,0,0,0,
	100,0,0,0,0,0,0,0,0,0,
	100,0,0,0,0,0,100,100,0,0,
	100,0,0,0,0,0,100,100,0,0,
	100,0,0,0,0,0,100,100,0,0,
	100,0,0,0,0,0,100,100,0,0,
	100,0,0,0,0,0,0,0,0,0,
	};

class PathFixture : public ::testing::Test 
{
public:
	nav_msgs::GetMap::Response map_resp_;
	const char* g_valid_png_file = "/test/testmap.png";
	const float g_valid_image_res = 0.1;

	PathFixture()
	{
		std::string full_path_map = ros::package::getPath("multi_agent_plan") + g_valid_png_file;

		const char* g_valid_png_file = full_path_map.c_str();

		double origin[3] = { 0.0, 0.0, 0.0 };

		try
		{
			map_server::loadMapFromFile(&map_resp_, g_valid_png_file, g_valid_image_res, false, 0.65, 0.1, origin);
		}
		catch(...)
		{
			ADD_FAILURE() << "Uncaught exception";
		}
	}
};

TEST_F( PathFixture, map )
{
	// Testing creating an empty map
	multi_agent_plan::Map<multi_agent_plan::Cell> map( 2, 2 );
	map.createEmptyMap();

	std::vector<multi_agent_plan::Cell> test;

	test.push_back( multi_agent_plan::CellDFS( createPose2D( 0.0, 0.0, 0.0 ) ) );
	test.push_back( multi_agent_plan::CellDFS( createPose2D( 1.0, 0.0, 0.0 ) ) );
	test.push_back( multi_agent_plan::CellDFS( createPose2D( 0.0, 1.0, 0.0 ) ) );
	test.push_back( multi_agent_plan::CellDFS( createPose2D( 1.0, 1.0, 0.0 ) ) );

	for (int i = 0; i < test.size(); ++i)
	{
		EXPECT_EQ( test[i].pose_.x, map.grid_[ i ].pose_.x );
		EXPECT_EQ( test[i].pose_.y, map.grid_[ i ].pose_.y );
	}

	// Testing getting cell
	multi_agent_plan::Cell* cell = map.getCell( 1, 1 );

	EXPECT_EQ( 1.0, cell->pose_.x );
	EXPECT_EQ( 1.0, cell->pose_.y );

	// Testing expansion
	std::vector<multi_agent_plan::Cell*> m = map.expand( 0, 0 );
	ASSERT_EQ( 2, m.size() );

	EXPECT_EQ( 1, m[0]->pose_.x );
	EXPECT_EQ( 0, m[0]->pose_.y );

	EXPECT_EQ( 0, m[1]->pose_.x );
	EXPECT_EQ( 1, m[1]->pose_.y );

	m = map.expand( 1, 0 );
	ASSERT_EQ( 2, m.size() );

	EXPECT_EQ( 0, m[0]->pose_.x );
	EXPECT_EQ( 0, m[0]->pose_.y );

	EXPECT_EQ( 1, m[1]->pose_.x );
	EXPECT_EQ( 1, m[1]->pose_.y );

	m = map.expand( 0, 1 );
	ASSERT_EQ( 2, m.size() );

	EXPECT_EQ( 0, m[0]->pose_.x );
	EXPECT_EQ( 0, m[0]->pose_.y );

	EXPECT_EQ( 1, m[1]->pose_.x );
	EXPECT_EQ( 1, m[1]->pose_.y );

	m = map.expand( 1, 1 );
	ASSERT_EQ( 2, m.size() );

	EXPECT_EQ( 1, m[0]->pose_.x );
	EXPECT_EQ( 0, m[0]->pose_.y );

	EXPECT_EQ( 0, m[1]->pose_.x );
	EXPECT_EQ( 1, m[1]->pose_.y );

	cell->is_occupied_ = true;
	m = map.expand( 0, 1 );
	ASSERT_EQ( 1, m.size() );

	EXPECT_EQ( 0, m[0]->pose_.x );
	EXPECT_EQ( 0, m[0]->pose_.y );

	multi_agent_plan::Map<multi_agent_plan::Cell> map2( map_resp_.map );

	// Testing map loaded correctly
	for (int i = 0; i < 100; ++i)
	{
		bool is_occupied_ = false;

		if ( g_valid_image_content[ i ] == 100 )
		{
			is_occupied_ = true;
		}

		ASSERT_EQ( is_occupied_, map2.grid_[ i ].is_occupied_ );
	}
}

TEST_F( PathFixture, pathDijkstra )
{
	multi_agent_plan::Dijkstra planner( map_resp_.map );

	// Testing path computed
	std::vector<geometry_msgs::Pose2D> plan1 = planner.pathPlanning( createPose2D( 0.0, 0.0, 0.0 ), createPose2D( 9.0, 9.0, 0.0 ) );

	ASSERT_NE( plan1.size(), 0 );

	std::vector<geometry_msgs::Pose2D> test1;

	test1.push_back( createPose2D( 0.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 1.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 2.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 3.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 0.0, 0.0 ) );

	test1.push_back( createPose2D( 4.0, 1.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 2.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 3.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 4.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 5.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 6.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 7.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 8.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 9.0, 0.0 ) );

	test1.push_back( createPose2D( 5.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 6.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 7.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 8.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 9.0, 9.0, 0.0 ) );

	ASSERT_EQ( test1.size(), plan1.size() );

	for (int i = 0; i < test1.size(); ++i)
	{
		EXPECT_EQ( test1[ i ].x, plan1[ i ].x ) << i;
		EXPECT_EQ( test1[ i ].y, plan1[ i ].y ) << i;
		EXPECT_EQ( test1[ i ].theta, plan1[ i ].theta ) << i;
	}
}

TEST_F(PathFixture, pathDFS)
{
	multi_agent_plan::DFS planner( map_resp_.map );

	// Testing path computed
	std::vector<geometry_msgs::Pose2D> plan1 = planner.pathPlanning( createPose2D( 0.0, 0.0, 0.0 ), createPose2D( 9.0, 9.0, 0.0 ) );

	ASSERT_NE( plan1.size(), 0 );

	std::vector<geometry_msgs::Pose2D> test1;

	test1.push_back( createPose2D( 0.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 1.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 2.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 3.0, 0.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 0.0, 0.0 ) );

	test1.push_back( createPose2D( 4.0, 1.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 2.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 3.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 4.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 5.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 6.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 7.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 8.0, 0.0 ) );
	test1.push_back( createPose2D( 4.0, 9.0, 0.0 ) );

	test1.push_back( createPose2D( 5.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 6.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 7.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 8.0, 9.0, 0.0 ) );
	test1.push_back( createPose2D( 9.0, 9.0, 0.0 ) );

	ASSERT_EQ( test1.size(), plan1.size() );

	for (int i = 0; i < test1.size(); ++i)
	{
		EXPECT_EQ( test1[ i ].x, plan1[ i ].x ) << i;
		EXPECT_EQ( test1[ i ].y, plan1[ i ].y ) << i;
		EXPECT_EQ( test1[ i ].theta, plan1[ i ].theta ) << i;
	}
}

TEST(MultiAgent, pathManhattan)
{
	multi_agent_plan::ManhattanPlanner planner( 10, 10 );

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
	return RUN_ALL_TESTS();
}