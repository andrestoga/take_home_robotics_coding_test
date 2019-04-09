/*

agent_node.cpp
Andres Torres Garcia (andrestoga@gmail.com)

Create an agent node: (20 points)
#Should be able to launch the node using launch file with agent information: serial id and start position

#Publish the current position of agent on a topic "/agent_feedback"

#Should have a rosservice “/update_goal” which takes goal position (x, y, theta) as request. Once the goal is input, the agent should request planner for a path.

#Assume that agent moves uniformly, between adjoining nodes, over a time a period of 10 seconds. You are not required to simulate the motion of the agent.

#Display the path on rviz

*/

#include "multi_agent_plan/agent_node.h"
#include <ros/spinner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent_node_app");
  ros::NodeHandle n;

  // Creating more threads to keep publishing an agent pose when one thread is busy updating its goal.
  ros::AsyncSpinner spinner(2);

  ros::Rate loop_rate(30);

  // Starting threads.
  spinner.start();

  multi_agent_plan::AgentNode agn( n );

  while (ros::ok())
  {
    agn.update();
    loop_rate.sleep();
  }

  // Waiting for the ROS threads.
  ros::waitForShutdown();

  return 0;
}