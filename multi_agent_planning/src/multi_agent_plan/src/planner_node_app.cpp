/*
* @Author: Andres
* @Date:   2019-02-27 20:45:41
* @Last Modified by:   Andres
* @Last Modified time: 2019-02-27 20:58:03
*/

#include "multi_agent_plan/planner_node.h"
#include "multi_agent_plan/manhattan_planner.h"
#include "multi_agent_plan/dfs.h"
#include "ros/ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner_node_app");
	ros::NodeHandle n;

	// if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
	// {
	// 	ros::console::notifyLoggerLevelsChanged();
	// }

	// multi_agent_plan::PlannerNode<multi_agent_plan::ManhattanPlanner> pl_n(n);
	multi_agent_plan::PlannerNode<multi_agent_plan::DFS> pl_n(n);

	ROS_INFO("Ready to plan.");
	ros::spin();

	return 0;
}