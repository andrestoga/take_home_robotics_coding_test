#ifndef AGENT_H
#define AGENT_H

#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include "map.h"
#include "multi_agent_plan/CurrPose.h"
#include "multi_agent_plan/UpdateGoal.h"

namespace multi_agent_plan
{
	class Agent
	{
		// Access specifier 
		public:

		    // Data Members
		    Pose2D curr_pose_;
		    string serial_id_;
		    Map grid_;
		    Pose2D goal_pose_;
		    std::vector<Pose2D> last_path_;

		    ros::NodeHandle nh_;
		    ros::Publisher pose_pub_;
		    ros::ServiceServer goal_service_;
		    ros::ServiceClient planner_client_;

			Agent(const ros::NodeHandle& nh, Pose2D curr_pose, string serial_id, const Map& grid );
		  
		    // w = up, a = left, d = right, s = down
		    bool moveAgent( char action );
	    	bool updateGoal( multi_agent_plan::UpdateGoal::Request  &req,
		             multi_agent_plan::UpdateGoal::Response &res );
	    	void displayPathOnRviz();
	    	void publish_curr_pose();

	};
}

#endif