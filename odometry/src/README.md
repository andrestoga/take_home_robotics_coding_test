# Instructions

## Context

Our weeding robot is a car-like robot with two front wheels that are both steering and driving and
two rear fixed wheels. An absolute encoder is mounted on each rear wheel. For the purpose of
keeping track of the x and y positions of the robot as well as of its orientation, the two rear
wheels can be seen as a differential drive system. The goal is to write a ROS node that
implements this logic.

## Expected outcome

**Once you receive these instructions, you have 90 minutes to** ​please email us the
implementation of a ROS node that keeps track of the position of the robot with the following
requirements:

- the encoders provide absolute positions in units of 'ticks' provided on topics for each wheel
(encoders/left_wheel and encoders/right_wheel) of type std_msgs/Int64. The data range of the
encoder is [0 - 16777216], with the value wrapping between the min and max in the case of
overflow / underflow.
- the ticks-per-meter calibration of each wheel should be specified as ROS parameters
- the distance between the center of the two wheels should be specified as a ROS parameter
- the x, y position of the robot, the heading, the instantaneous speed of the robot and the
instantaneous rotational speed of the robot should be reported on a topic with messages of type
nav_msgs/Odometry at a fixed frequency specified as a ROS parameter
- the ROS node can either be implemented in python or C++.

## Formulas

If distance_left_wheel (resp. distance_right_wheel) denotes the distance traveled by the left
(resp. right) wheel as reported by its encoder since the last update, we have:

- the heading increment in radians is given by (this approximation is only valid when both
distance_right_wheel and distance_left_wheel are small):
heading_increment = (distance_right_wheel - distance_left_wheel) / base_width
- the increment in x and y positions in the instantaneous reference frame of the robot are given
by:
x_increment = cos(heading_increment)*(distance_left_wheel+distance_right_wheel)/
y_increment = - sin(heading_increment)*(distance_left_wheel+distance_right_wheel)/
- the increment in x and y positions in the fixed reference frame are given by:
x_fixed_increment = cos(heading) * x_increment - sin(heading) * y_increment


y_fixed_increment = sin(heading) * x_increment + cos(heading) * y_increment
where base_width is the distance between the center of the two wheels.