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

#include "multi_agent_plan/agent.h"
#include <ros/spinner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent");
  ros::NodeHandle n;

  std::string serial_id = "test_1";
  geometry_msgs::Pose2D pose;

  pose.x = 0;
  pose.y = 0;
  pose.theta = 0;

  // Creating more threads to keep publishing an agent pose when one thread is busy updating its goal.
  ros::AsyncSpinner spinner(4);

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

  multi_agent_plan::Map map( 5, 10 );
  multi_agent_plan::Agent ag( n, pose, serial_id, map ); 

  ros::Rate loop_rate(20);

  // Starting threads.
  spinner.start();

  while (ros::ok())
  {
    ag.publishCurrPose();
    ag.displayPathOnRviz();
    loop_rate.sleep();
  }

  // Waiting for the ROS threads.
  ros::waitForShutdown();

  return 0;
}