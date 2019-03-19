# Multi-agent planning

This repository contains a ROS package which simulates agents planning and moving in a 10x10 4-connected grid.

## Requirements

- Ubuntu 16.04 with ROS Kinetic.

## Setup

1) Create a ROS workspace in your computer(optional if you want to put it inside of an existing one).
2) Go to the src folder.
3) Clone this repo.
4) Compile workspace.

## How to use

- Launch the launch file named test.launch.
- The launch file will run the following nodes:
```sh
Node agent with serial id: "agent_1", start position (x, y, yaw): (2, 0, 0 degree).

Node agent with serial id: "agent_2", start position (x, y, yaw): (0, 3, 0 degree).

Node planner.
```

**Note:** You can play with the private parameters for each node. The map is 10*10 4-connected grid in x-y plane bounded by x=0, x=10, y=0 and y=10 lines. If you set the start position outside the bounded plane, the start position would be set to x=0 and y=0. For the yaw value, it doesn't matter because we are assuming that the robot is omnidirectional and it doesn't need to rotate to move to one of its four adjacent cells. However, if you set a value that is not between -pi and pi, it would set it to zero. For the serial id, any unique value will do.

After launching the launch file, you can use the services to update the agents goals, agent 1 and agent 2 respectively in this example. Before doing that, open RViz and open the configuration file located in the RViz folder to see the agents start positions. Once you requested the new goal positions, you will see the path for each agent and the agents moving towards their new goals.

## Screenshot

![](https://raw.githubusercontent.com/andrestoga/Multi-agent-planning/master/multi_agent_planning_rviz_screenshot.png?token=ALPuszrijPeDI6OmjbjsGlloEDt39gMqks5cfNKzwA%3D%3D)

## Example

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/y4GiCHfjclk/0.jpg)](https://www.youtube.com/watch?v=y4GiCHfjclk)

## Topics

`~agent_feedback`
Publish the current position of an agent.

## Services

`~update_goal`
Takes goal position (x, y, theta) as request. Once the goal is input, the agent should request planner for a path.

`/get_plan`
Requests plan for an agent with serial id and goal position. Start position would be the current position of that agent which can be read from rostopic `~agent_feedback`. The response would have list of points constituting the path (minimum distance).

## TODOs

- Implement anonymous nodes.
- Implement other planners( A-start, Dijkstra, BFS ).
- Unit tests.
	- Limits in x, y and theta.
	- Check the final position of the agent.
- Coordination between agents to not collide.

## Maintainer

Andres Torres Garcia (andrestoga@gmail.com)