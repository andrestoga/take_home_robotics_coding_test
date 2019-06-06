/* agent.h
 * Andres Torres Garcia (andrestoga@gmail.com)
 */

#ifndef AGENT_H
#define AGENT_H

#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"

namespace multi_agent_plan
{
	class Agent
	{	
		// Access specifier 
		public:

			/**
			 * @brief      Constructs the object.
			 *
			 * @param[in]  nh         node handle
			 * @param[in]  curr_pose  The curr pose
			 * @param[in]  serial_id  The serial identifier
			 * @param[in]  grid       The grid
			 */
			Agent( geometry_msgs::Pose2D pose, std::string serial_id );
	    	/**
	    	 * @brief      Transform the points from the grid to the Gazebo grid
	    	 *
	    	 * @param[in]  point   The point to transform
	    	 * @param[in]  offset  The offset from the middle point to the 0, 0 position.
	    	 *
	    	 * @return     The transformed point
	    	 */
	    	geometry_msgs::Pose2D transformPointsToWd( geometry_msgs::Pose2D point, float offset );
	    	/**
	    	 * @brief      Check that the pose is inside the limits of the map
	    	 *
	    	 * @param[in]  pose  The pose to check
	    	 *
	    	 * @return     Return a valid pose. If the input pose was not inside the limits of the map, the pose will be 0, 0, 0. Otherwise, it will be the input pose.
	    	 */
	    	// geometry_msgs::Pose2D checkPose( geometry_msgs::Pose2D pose, int width, int height );
	    	void setPose( geometry_msgs::Pose2D pose );
	    	
	    	geometry_msgs::Pose2D getPose() const;

	    // protected:
	    	geometry_msgs::Pose2D pose_;

	    	// Data Members
	    	std::string serial_id_;
	    	
	    	geometry_msgs::Pose2D goal_pose_;
	    	std::vector<geometry_msgs::Pose2D> last_path_;
	};
}

#endif