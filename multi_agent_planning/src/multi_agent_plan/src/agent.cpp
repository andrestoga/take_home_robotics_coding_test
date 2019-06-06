/* agent.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/agent.h"
#include <math.h>

namespace multi_agent_plan
{
	Agent::Agent(geometry_msgs::Pose2D pose, std::string serial_id )
	: serial_id_( serial_id )
	{
		setPose( pose );
	}

	geometry_msgs::Pose2D Agent::getPose() const
	{
		return pose_;
	}

	void Agent::setPose( geometry_msgs::Pose2D pose )
	{

		pose_ = pose;

		// if ( pose.x > width || pose.x < 0.0 )
		// {
		// 	pose_.x = 0.0;
		// }

		// if ( pose.y > height || pose.y < 0.0 )
		// {
		// 	pose_.y = 0.0;
		// }

		// if ( pose.theta > M_PI || pose.theta < -M_PI )
		// {
		// 	pose_.theta = 0.0;	
		// }
	}

	geometry_msgs::Pose2D Agent::transformPointsToWd( geometry_msgs::Pose2D point,float offset )
	{
		geometry_msgs::Pose2D tmp;
		tmp.x = point.x - offset;
		tmp.y = point.y - offset;

		return tmp;
	}
}