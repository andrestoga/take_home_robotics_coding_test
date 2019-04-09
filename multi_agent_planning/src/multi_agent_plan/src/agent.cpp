/* agent.cpp
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/agent.h"
#include <math.h>

namespace multi_agent_plan
{
	Agent::Agent(geometry_msgs::Pose2D curr_pose, std::string serial_id, const Map& grid )
	: serial_id_( serial_id )
	, grid_( grid.m_, grid.n_ )
	{
		curr_pose_ = checkPose( curr_pose );
	}

	geometry_msgs::Pose2D Agent::checkPose( geometry_msgs::Pose2D pose )
	{
		if ( pose.x > grid_.n_ || pose.x < 0.0 )
		{
			pose.x = 0.0;
		}

		if ( pose.y > grid_.n_ || pose.y < 0.0 )
		{
			pose.y = 0.0;	
		}

		if ( pose.theta > M_PI || pose.theta < -M_PI )
		{
			pose.theta = 0.0;	
		}

		return pose;
	}

	geometry_msgs::Pose2D Agent::transformPointsToWd( geometry_msgs::Pose2D point,float offset )
	{
		geometry_msgs::Pose2D tmp;
		tmp.x = point.x - offset;
		tmp.y = point.y - offset;

		return tmp;
	}
}