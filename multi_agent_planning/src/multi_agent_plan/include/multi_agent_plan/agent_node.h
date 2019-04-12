#ifndef AGENT_NODE_H
#define AGENT_NODE_H

/* agent_node.h
 * Andres Torres Garcia (andrestoga@gmail.com)
 */
#include "multi_agent_plan/agent.h"
#include <math.h>
#include "multi_agent_plan/CurrPose.h"
#include "multi_agent_plan/UpdateGoal.h"
#include "multi_agent_plan/GetPlan.h"

namespace multi_agent_plan
{
	class AgentNode
	{
		public:

			std::unique_ptr<Agent> ag_;
			
			ros::Publisher pose_pub_;
			ros::Publisher moving_pub_;
			ros::ServiceServer goal_service_;
			ros::ServiceServer update_pose_srv_;
			ros::ServiceClient planner_client_;
			ros::Publisher path_pub_;

			std::vector<geometry_msgs::Pose2D> last_path_;

			ros::NodeHandle nh_;

			ros::Time move_start_time_;//Start time of the movement.
			ros::Duration move_duration_;//Duration of the movement.

    		int i_path_;
    		bool is_agent_moving_;

			/**
			 * @brief      Constructs the object.
			 *
			 * @param[in]  nh    ros node handler
			 */
			AgentNode( const ros::NodeHandle& nh );

			/**
			 * @brief      Follows the path plan computed
			 *
			 * @return     Return true when it's done
			 */
		  	bool followPathPlan();

	  	    /**
	  	     * @brief      Callback function to update an agent's goal. After updating its goal, it will call a service to get a path for the new goal and move an agent through that path.
	  	     *
	  	     * @param      req   The request
	  	     * @param      res   The response
	  	     *
	  	     * @return     Return true when it's done
	  	     */
	      	bool updateGoal( multi_agent_plan::UpdateGoal::Request &req,
	  	             multi_agent_plan::UpdateGoal::Response &res );

	      	bool updatePose( multi_agent_plan::UpdateGoal::Request &req, multi_agent_plan::UpdateGoal::Response &res );

	      	/**
	      	 * @brief      Display the movement of the robot on RViz using the given path
	      	 */
	      	void displayPathOnRviz();

	      	/**
	      	 * @brief      Publish the current pose and its transformation of the agent with respect to the origin
	      	 */
	      	void publishCurrPose();

	      	/**
	      	 * @brief      Get the plan depending on the current pose and the new goal.
	      	 */
	      	void getPlan();

  			void update();

  			void updatePose();

	};
}

#endif