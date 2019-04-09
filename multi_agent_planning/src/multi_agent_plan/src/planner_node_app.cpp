/*
* @Author: Andres
* @Date:   2019-02-27 20:45:41
* @Last Modified by:   Andres
* @Last Modified time: 2019-02-27 20:58:03
*/

#include "multi_agent_plan/planner_node.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner_node_app");
	ros::NodeHandle n;

	multi_agent_plan::Map map( 5, 10 );
	multi_agent_plan::PlannerNode pl_n(n, map);

	ROS_INFO("Ready to plan.");
	ros::spin();

  return 0;
}