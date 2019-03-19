/**
 * Copyright (C) 2019
 *
 * All rights reserved.
 */

#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <turtlesim/Pose.h>

#include "take_home_test/MoveAction.h"
#include "take_home_test/MoveGoal.h"

namespace
{
	const float THRESHOLD = 0.1;
}

void sendGoal(
	actionlib::SimpleActionClient<take_home_test::MoveAction> &client,
	const turtlesim::Pose &destination,
	const float &turtle_x,
	const float &turtle_y)
{
	take_home_test::MoveGoal goal;
	goal.destination = destination;
	ROS_INFO("Sending goal (%f, %f)", destination.x, destination.y);

	client.sendGoal(goal,
		[goal, &turtle_x, &turtle_y](
			const actionlib::SimpleClientGoalState &state,
			const take_home_test::MoveResultConstPtr &result)
		{
			auto x_difference = goal.destination.x - turtle_x;
			auto y_difference = goal.destination.y - turtle_y;
			auto distance = sqrt(pow(x_difference, 2) + pow(y_difference, 2));

			// Detect success and say so, or detect failure and bail
			if (distance < THRESHOLD)
			{
				ROS_INFO("Goal reached");
			}
			else
			{
				ROS_FATAL("Too far from the goal!");
				ros::shutdown();
			}
		});
}

void initializeDestinations(std::vector<turtlesim::Pose> &destinations)
{
	turtlesim::Pose destination;
	destination.x = 0;
	destination.y = 5;
	destinations.push_back(destination);

	destination.x = 4;
	destination.y = 2;
	destinations.push_back(destination);

	destination.x = 9;
	destination.y = 1;
	destinations.push_back(destination);

	destination.x = 2;
	destination.y = 8;
	destinations.push_back(destination);

	destination.x = 8;
	destination.y = 8;
	destinations.push_back(destination);
}

void waitForActionServer(const actionlib::SimpleActionClient<take_home_test::MoveAction> &client)
{
	ros::Rate rate(10);
	ROS_INFO("Waiting for action server...");
	while (ros::ok() && !client.isServerConnected())
	{
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Connected to action server");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "puppet_master");
	ros::NodeHandle node;

	float turtle_x = 0;
	float turtle_y = 0;

	auto sub = node.subscribe<turtlesim::Pose>(
		"turtle_pose", 1, [&turtle_x, &turtle_y](const turtlesim::Pose::ConstPtr &message)
	{
		turtle_x = message->x;
		turtle_y = message->y;
	});

	actionlib::SimpleActionClient<take_home_test::MoveAction> action_client("move", false);
	waitForActionServer(action_client);

	std::vector<turtlesim::Pose> destinations;
	initializeDestinations(destinations);

	auto destination_iterator = destinations.cbegin();

	auto timer = node.createTimer(ros::Duration(1),
		[&action_client, &destination_iterator, &destinations, &turtle_x, &turtle_y](
			const ros::TimerEvent &event)
		{
			// Early exit, existing goal is not done
			if (destination_iterator != destinations.cbegin() &&
			    !action_client.getState().isDone())
			{
				return;
			}

			// If all destinations are complete, exit
			if (destination_iterator == destinations.cend())
			{
				ROS_INFO("Success: all destinations complete!");
				ros::shutdown();
			}

			// Send the next destination as the action goal
			sendGoal(action_client, *(destination_iterator++), turtle_x, turtle_y);
		});

	ros::spin();

	return 0;
}