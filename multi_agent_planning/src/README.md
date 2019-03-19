## Recuitment - Multi-agent_planning - # 5

# Multi-agent planning

Create roadmap (10 points)

```
Create a 10X10 4-connected grid in x-y plane bounded by x=0, x=10, y=0 and y=10 lines.
Use data structure of your choice
Assumptions: All the nodes are at yaw angle of 0 degrees. Each node has zero cost associated with it
Assumptions: All the edges are bi-directional. Each edge has cost of "10" associated with it
Assumptions: All agents are circle of radius 0.5m
```
Create a planner node in ROS (35 points)

```
Should have a rosservice "/get_plan" that requests plan for an agent with serial id and goal position. Start position would be the
current position of that agent which can be read from rostopic "/agent_feedback". The response would have list of points
constituting the path (minimum distance). Follow any algorithm of your choice to generate the path.
The planner show retain the plan for an agent. This would be useful for path planning of future agents. Assume that controller is
ideal; follows the trajectory perfectly.
Clearly describe the approach in the code.
```
Create an agent node: (20 points)

```
Should be able to launch the node using launch file with agent information: serial id and start position
Publish the current position of agent on a topic "/agent_feedback"
Should have a rosservice “/update_goal” which takes goal position (x, y, theta) as request. Once the goal is input, the agent
should request planner for a path.
Assume that agent moves uniformly, between adjoining nodes, over a time a period of 10 seconds. You are not required to
simulate the motion of the agent.
Display the path on rviz
```
Test case: (additional 20 points if unit test is created)

```
Launch two different agents
Launch agent with serial id: “agent_1”, start position (x, y, yaw): (2, 0, 0 degree)
Launch agent with serial id: “agent_2”, start position (x, y, yaw): (0, 3, 0 degree)
Call the rosservice “/update_goal” for agent_1 with goal: (2, 5, 0 deg). This should have the path displayed on rviz
Call the rosservice “/update_goal” for agent_2 with goal: (6, 3, 0 deg). This should have the path displayed on rviz
```
Git: (15 points)

```
Your code should be uploaded on a Git repository created for this test. The name of the repo would be
"FIRSTNAME_LASTNAME_FULLTIME". Name of the company "Bito" should not be there anywhere in your code or in repo.
It is advised to do frequent commits during code development.
```
```
02/08/2019 1/
```

```
Good practice: Develop each feature in a separate branch and merge back to master after module test is a bonus.
README should be updated with procedure to run your codes.
```
Skills:

```
C++ (C++11 version)
ROS
Motion planning
Coding style (partial points in each of the above section is designated to this)
```
Final Submission:

```
Submit the link of the repository when you are done.
```
**Files**

picture463-1.png 50.4 KB 02/08/2019 Puneet Singhal

```
Powered by TCPDF (www.tcpdf.org)
```
```
02/08/2019 2/
```