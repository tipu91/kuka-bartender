#include "BartenderManager.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bartender_manager");

	BartenderManager manager;

	std::vector<int> closure_value; 
	std::vector<int> opening_value; 

	std::string s_l, s_r;

	closure_value.resize(2);
	opening_value.resize(2);

	closure_value[0] = 1;
	closure_value[1] = 1;

	opening_value[0] = 0;
	opening_value[1] = 0;

	s_l = "left";
	s_r = "right";

	// wait for tf service
	ros::spinOnce();
	usleep(1000000);
	
	manager.Init();
	
	    
	manager.pub_bartender_config_right.publish(manager.msg_config);
	manager.pub_bartender_config_left.publish(manager.msg_config);

	bool select = true;	
	bool arrived = false;
	int action;
	
	while(ros::ok())
	//while(ros::ok() && manager.run_manager)
	{
		
		// ros::spinOnce();
		
		if(select) {
		  action = 1;
		  select = false;
		  manager.ToPose("right_arm", "vodka", manager.pub_bartender_cmd_right,true);
		  manager.ToPose("left_arm", "lemon", manager.pub_bartender_cmd_left,false);
		}
		
		switch(action){
		  
		  // Right arm go to grasping pose
		  case 1:
		    
		    ROS_INFO("Action 1");
		        
		    if( manager.compare_error(manager.error_right) && !arrived)
		    {
			ROS_INFO("ready for grasp");
			arrived = true;
			//select = true;
			manager.ToPose("right_arm", "vodka", manager.pub_bartender_cmd_right,false);
			
			//grasp selection
			std::string ans;

			while(action==1){
			    std::cout << "Are You ready for grasping? (y/n) " << std::endl;
			    getline (std::cin, ans);
			    
			    if(ans.compare("y") == 0) {
			      action = 2;
			      manager.Grasping(closure_value,s_r);
			    }
			    else sleep(2);
			}
		    
		      
		     }
		    
		    ros::spinOnce();
		  
		    break;
		    
		  case 2:
		    
		    ROS_INFO("Action 2");
		    
		    break;
		    
		    
		}
		    
		    
		    

	}
		
	
	return 0;
}