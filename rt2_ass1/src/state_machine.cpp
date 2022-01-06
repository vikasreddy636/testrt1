#include "ros/ros.h"
#include "rt2_ass1/Command.h"
#include "rt2_ass1/Position.h"
#include "rt2_ass1/RandomPosition.h"
#include <rt2_ass1/go_toAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false;
bool not_moving = true;

bool user_interface(rt2_ass1::Command::Request &req, rt2_ass1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::NodeHandle n1;

   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_ass1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_ass1::go_toAction> ac("/go_to");
   
   rt2_ass1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_ass1::Position p;
   
	while(ros::ok())
		{
			ros::spinOnce();
			if (start)
				{
					if (not_moving)
						{
							client_rp.call(rp);
							rt2_ass1::go_toGoal goal;
							goal.target_pose.header.frame_id = "base_link";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose.position.x = rp.response.x;
							goal.target_pose.pose.position.y = rp.response.y;
							goal.target_pose.pose.orientation.z = rp.response.theta;
							std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
							ac.sendGoal(goal);
							not_moving = false;
						}
					else 
						{
							if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
								{
									std::cout << "Goal completed" << std::endl;
									not_moving= true;
								}
						}
				}
			else 
				{
					if (!not_moving)
						{
							ac.cancelAllGoals();
							std::cout << "Goal cancelled" << std::endl;
							not_moving= true;
						}
				}
  

		}
   return 0;
}
