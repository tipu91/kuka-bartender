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
	
	int action;
	
	while(ros::ok())
	//while(ros::ok() && manager.run_manager)
	{
		
		//ros::spinOnce();
		
		if(select) {
		  // manager.DrinkSelection();
		  action = 1;
		  select = false;
		}
		
		switch(action){
		  
		  // Right arm go to grasping pose
		  case 1:
		    
		    manager.ToPose(manager.msg_right, "right_arm", "vodka", manager.pub_bartender_cmd_right,true);
		    manager.ToPose(manager.msg_left, "left_arm", "coca", manager.pub_bartender_cmd_left,false);
		        
		    if( manager.compare_error(manager.error_right) )
		    {
		      ROS_INFO("ready for grasp");
		      select = true;
		      manager.ToPose(manager.msg_right, "right_arm", "vodka", manager.pub_bartender_cmd_right,false);
		      //action = 2;
		    }
		    
		    ros::spinOnce();
		  
		    break;
		}
		    
		    
		    

	}
		
	
	return 0;
}