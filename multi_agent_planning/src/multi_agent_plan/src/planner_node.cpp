/* planner_node.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */

/*

Create a planner node in ROS (35 points)

// Start position would be the current position of that agent which can be read from rostopic "/agent_feedback". The response would have list of points constituting the path (minimum distance). Follow any algorithm of your choice to generate the path.
// Should have a rosservice "/get_plan" that requests plan for an agent with serial id and goal position.
// The planner show retain the plan for an agent. This would be useful for path planning of future agents. Assume that controller is ideal; follows the trajectory perfectly.
#Clearly describe the approach in the code.

*/

#include "multi_agent_plan/planner.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner");
	ros::NodeHandle n;

	multi_agent_plan::Map map( 5, 10 );
	multi_agent_plan::Planner pl(n, map);

	ROS_INFO("Ready to plan.");
	ros::spin();

  return 0;
}