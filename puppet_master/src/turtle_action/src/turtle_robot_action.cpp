/**
 * 
 * @authors Andres Torres Garcia (andrestoga@gmail.com)
 * @date    2019-03-14
 */
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>

#include "take_home_test/MoveAction.h"
#include "turtle_action/turtle_robot.hpp"

namespace canonical
{
	const float g_threshold = 0.1;

	typedef actionlib::SimpleActionServer<take_home_test::MoveAction> Server;

	class TurtleRobotAction
	{

	protected:

		ros::NodeHandle nh_;
		std::shared_ptr<Server> as_;
		// NodeHandle instance must be created before this line. Otherwise strange error occurs.
		std::string action_name_;
		// create messages that are used to published feedback/result
		take_home_test::MoveFeedback feedback_;
		take_home_test::MoveResult result_;
		canonical::TurtleRobot turtle_;
		ros::Publisher velocity_publisher_;
		ros::Subscriber turtle_pose_sub_;

	public:
		TurtleRobotAction( ros::NodeHandle& n, std::string name )
		:
		nh_( n )
		, action_name_( name )
		{
			velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( "turtle_cmd", 1000);
			turtle_pose_sub_ = nh_.subscribe("turtle_pose", 1000, &TurtleRobotAction::poseCallback, this);
			as_ = std::make_shared<Server>( nh_, action_name_, boost::bind( &TurtleRobotAction::computeVelocities, this, _1 ), false );
			as_->start();
		}

		void poseCallback(const turtlesim::Pose::ConstPtr &message)
		{
			turtle_.pose_.x = message->x;
			turtle_.pose_.y = message->y;
			turtle_.pose_.theta = message->theta;
		}

		void computeVelocities( const take_home_test::MoveGoalConstPtr& goal )
		{
			geometry_msgs::Twist vel_msg;
  			bool success = true;

			ros::Rate loop_rate(10);

			turtlesim::Pose goal_pose;
			goal_pose.x = goal->destination.x;
			goal_pose.y = goal->destination.y;
			goal_pose.theta = goal->destination.theta;

			// Set limit time to 10 sec to reach the goal. If not, return failure.
			auto timer = nh_.createTimer(ros::Duration(10),
		[&success](
			const ros::TimerEvent &event)
			{
				success = false;
				ROS_INFO( "Time is up!" );
			});

			do
			{
				ros::spinOnce();
				loop_rate.sleep();

				vel_msg = turtle_.calculateSpeed( goal_pose );
				velocity_publisher_.publish( vel_msg );

				feedback_.pose = turtle_.pose_;
				as_->publishFeedback( feedback_ );

				if ( !success )
				{
					break;
				}

			}while( turtle_.getDistance( turtle_.pose_.x, turtle_.pose_.y, goal_pose.x, goal_pose.y) > g_threshold );

			timer.stop();

			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			velocity_publisher_.publish( vel_msg );

			if ( success )
			{
				result_.success = true;
    			as_->setSucceeded( result_ );
			}
		}
	};
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "turtle_action_node");
  	ros::NodeHandle n;
  	canonical::TurtleRobotAction tra( n, "move" );

  	ROS_INFO("Ready to move the turtlesim.");
	ros::spin();

  return 0;
}