/* agent.h
 * Andres Torres Garcia (andrestoga@gmail.com)
 */

#ifndef AGENT_H
#define AGENT_H

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "map.h"
#include "multi_agent_plan/CurrPose.h"
#include "multi_agent_plan/UpdateGoal.h"
#include "multi_agent_plan/GetPlan.h"

namespace multi_agent_plan
{
	class Agent
	{
		// Data Members
		geometry_msgs::Pose2D curr_pose_;
		std::string serial_id_;
		Map grid_;
		geometry_msgs::Pose2D goal_pose_;
		std::vector<geometry_msgs::Pose2D> last_path_;

		ros::NodeHandle nh_;
		ros::Publisher pose_pub_;
		ros::ServiceServer goal_service_;
		ros::ServiceClient planner_client_;
		ros::Publisher path_pub_;
		
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
			Agent(const ros::NodeHandle& nh, geometry_msgs::Pose2D curr_pose, std::string serial_id, const Map& grid );
		  
		  	/**
		  	 * @brief      Updates the position of the agent
		  	 *
		  	 * @return     Return true when it's done
		  	 */
		    bool moveAgent();
		    /**
		     * @brief      Callback function to update an agent's goal. After updating its goal, it will call a service to get a path for the new goal and move an agent through that path.
		     *
		     * @param      req   The request
		     * @param      res   The response
		     *
		     * @return     Return true when it's done
		     */
	    	bool updateGoal( multi_agent_plan::UpdateGoal::Request  &req,
		             multi_agent_plan::UpdateGoal::Response &res );
	    	/**
	    	 * @brief      Display the movement of the robot on RViz using the given path
	    	 */
	    	void displayPathOnRviz();
	    	/**
	    	 * @brief      Publish the current pose and its transformation of the agent with respect to the origin
	    	 */
	    	void publishCurrPose();
	    	/**
	    	 * @brief      Get the of an agent depending of its current pose and its new goal.
	    	 */
	    	void getPlan();
	    	/**
	    	 * @brief      Transform the points from the grid to the Gazebo grid
	    	 *
	    	 * @param[in]  point   The point to transform
	    	 * @param[in]  offset  The offset from the middle point to the 0, 0 position.
	    	 *
	    	 * @return     The transformed point
	    	 */
	    	geometry_msgs::Pose2D transformPointsToWd( geometry_msgs::Pose2D point, float offset );
	    	geometry_msgs::Pose2D checkPose( geometry_msgs::Pose2D pose );


	};
}

#endif