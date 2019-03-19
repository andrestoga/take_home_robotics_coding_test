# ROS1 take-home test

Congratulations for making it this far in the interview process! At this time,
we'd like to get a feeling for your ROS and C++ skills, but do so in a setting
that makes you comfortable and provides the opportunity for you to do any
necessary research.

You have three days from the time you receive this test to complete it. If you
are unable or do not have the time to complete it, please send us as much as you
can.

This README is contained within a single ROS1 package written for Melodic. The
purpose of this package is to determine if you've passed the test, or if you
still have some work to do. Create a new workspace for this test, place this
package within it, satisfy its dependencies, and build it.


## The test

This take-home test consists of a single node called Puppet Master. That node
connects to an action server and requests that action server move the robot to a
given destination. The action description is contained within the
`take_home_test` package. The "robot" is simulated using Turtlesim, and both
Turtlesim and Puppet Master are properly configured using the `run.launch` file
in the `take_home_test` package.

Your job is to create a new package containing a node that moves the Turtlesim
robot to the proper destination. The Puppet Master will put your node through
its paces using a number of different destinations.

The deliverable for this test is a tarball of your new package, which should at
a minimum contain the code for your node as well as a launch file that runs the
test (including Puppet Master and Turtlesim). Instructions for how to submit
this tarball were included in the email providing this test.


## Rules

1. You're welcome to read this package to your heart's content, but modifying
   any part of it is not an acceptable solution to the test. If you have issues
   with the package, please reach out to your point of contact at Canonical.
2. Your solution will be evaluated using ROS Melodic and Ubuntu 18.04 (Bionic),
   so you might want to develop in that environment.
3. Your solution must be written in C++.
4. You may not use any of turtlesim's services (only its topics).
5. Please do not share this take-home test or your solution with anyone.


## Getting results

After a successful run, Puppet Master's output will look like this:

    [ INFO] [1551731748.111257879]: Sending goal (0.000000, 5.000000)
    [ INFO] [1551731750.413010494]: Goal reached
    [ INFO] [1551731750.511485080]: Sending goal (4.000000, 2.000000)
    [ INFO] [1551731752.712767517]: Goal reached
    [ INFO] [1551731752.811583177]: Sending goal (9.000000, 1.000000)
    [ INFO] [1551731754.813269547]: Goal reached
    [ INFO] [1551731754.911743025]: Sending goal (2.000000, 8.000000)
    [ INFO] [1551731757.213968323]: Goal reached
    [ INFO] [1551731757.311347601]: Sending goal (8.000000, 8.000000)
    [ INFO] [1551731759.613065134]: Goal reached
    [ INFO] [1551731759.711099466]: Success: all destinations complete!


Failure is defined to be anything that doesn't result in that "Success"
message. It may be a segfault. It may simply never progress. It may look like
this:

    [ INFO] [1551733774.710989357]: Sending goal (0.000000, 5.000000)
    [FATAL] [1551733775.912321155]: Too far from the goal!


It's important to note that, even if you manage to get a successful run, it
doesn't necessarily mean you've passed this test. Code quality and C++ best
practices are also very important.