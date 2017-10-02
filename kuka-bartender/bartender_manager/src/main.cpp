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
	
	sleep(2);

	bool init_select = true;
	bool select = true;	
	
	bool arrived;
	int action;
	
	std::string ans;
	
	while(ros::ok())
	//while(ros::ok() && manager.run_manager)
	{
		
		if(init_select) {
		  action = 1;
		  init_select = false;
		}
		
		switch(action){
		  
		  // Right arm go to grasping pose (for bottle)
		  case 1:
		    
		    if(select) {
		      select = false;
		      manager.ToPose("right_arm", "right_grasp", action, manager.pub_bartender_cmd_right,true, false);
		      ROS_INFO("Action 1");
		    }

		    if( manager.compare_error(manager.error_right, 0.05, 0.05) )
		    {
			ROS_INFO("ready for grasp");
			manager.ToPose("right_arm", "right_grasp", action, manager.pub_bartender_cmd_right,false, false);

			while(action==1){
			    std::cout << "Are You ready for grasping? (y/n) " << std::endl;
			    getline (std::cin, ans);
			    
			    if(ans.compare("y") == 0) {
			      action = 2;
			      select = true;
			      manager.Grasping(closure_value,s_r);
			    }
			    else sleep(2);
			}
		    
		      
		     }
		    
		    ros::spinOnce();
		  
		    break;
		    
		  // Left arm go to grasping pose (for glass)  
		  case 2:
		    
		    if(select) {
		      select = false;
		      manager.resetError(manager.error_right);
		      manager.resetError(manager.error_left);
		      manager.ToPose("left_arm", "left_grasp", action, manager.pub_bartender_cmd_left,true, false);
		      ROS_INFO("Action 2");
		    }
		    
		    if( manager.compare_error(manager.error_left, 0.05, 0.05) )
		    {
			ROS_INFO("ready for grasp");
			manager.ToPose("left_arm", "left_grasp", action, manager.pub_bartender_cmd_left,false, false);

			while(action==2){
			    std::cout << "Are You ready for grasping? (y/n) " << std::endl;
			    getline (std::cin, ans);
			    
			    if(ans.compare("y") == 0) {
			      action = 3;
			      select = true;
			      manager.Grasping(closure_value,s_l);
			    }
			    else sleep(2);
			}
		    
		      
		     }
		    
		    ros::spinOnce();
		    
		    break;
		    
		  // To pouring position  
		  case 3:
		    
		    if(select) {
		      select = false;
		      manager.resetError(manager.error_right);
		      manager.resetError(manager.error_left);
		      manager.ToPose("left_arm", "left_pour", action, manager.pub_bartender_cmd_left,true, false);
		      manager.ToPose("right_arm", "right_pour", action, manager.pub_bartender_cmd_right,true, false);
		      ROS_INFO("Action 3");
		    }
		        
		    if( manager.compare_error(manager.error_right, 0.08, 0.05) && manager.compare_error(manager.error_left, 0.08, 0.05) )
		    {
			ROS_INFO("ready for pouring");
			
			manager.ToPose("left_arm", "left_pour", action, manager.pub_bartender_cmd_left,false, false);
			manager.ToPose("right_arm", "right_pour", action, manager.pub_bartender_cmd_right,false, false);

			while(action==3){
			    std::cout << "Are You ready for pouring? (y/n) " << std::endl;
			    getline (std::cin, ans);
			    
			    if(ans.compare("y") == 0) {
			      action = 4;
			      select = true;
			    }
			    else sleep(2);
			}
		    
		      
		     }
		    
		    ros::spinOnce();
		    
		    break;
		    
		  // Pouring action 
		  case 4:
		    
		    if(select) {
		      select = false;
		      manager.resetError(manager.error_right);
		      manager.resetError(manager.error_left);
		      manager.ToPose("right_arm", "pouring", action, manager.pub_bartender_cmd_right,true, false);
		      ROS_INFO("Action 4");
		    }
		    
		        
		    if( manager.compare_error(manager.error_right, 0.05, 0.1) )
		    {
			ROS_INFO("Pouring");
			
			manager.ToPose("right_arm", "pouring", action, manager.pub_bartender_cmd_right,false, false);
			
			ros::Time Init_time = ros::Time::now();
			ros::Duration duration;

			while(action==4){
			    
			  duration = ros::Time::now() - Init_time;
			  std::cout << duration.toSec() << std::endl;
			  
			  if(duration.toSec() > 5) {
			    action = 5;
			    select = true;
			  }
			  
			}
		        
		     }
		    
		    ros::spinOnce();
		    
		    break;
		    
		  // Stop Pouring action 
		  case 5:
		    
		    if(select) {
		      select = false;
		      manager.resetError(manager.error_right);
		      manager.resetError(manager.error_left);
		      manager.ToPose("right_arm", "right_pour", action, manager.pub_bartender_cmd_right,true, false);
		      ROS_INFO("Action 5");
		    }
		    
		        
		    if( manager.compare_error(manager.error_right, 0.1, 0.05) )
		    {
			ROS_INFO("ready for shaking");
			
			manager.ToPose("right_arm", "right_pour", action, manager.pub_bartender_cmd_right,false, false);
			
			while(action==5){
			    std::cout << "Are You ready for shaking? (y/n) " << std::endl;
			    getline (std::cin, ans);
			    
			    if(ans.compare("y") == 0) {
			      action = 6;
			      select = true;
			    }
			    else sleep(2);
			}
		        
		     }
		    
		    ros::spinOnce();
		    
		    break;
		    
		  // Shake action 
		  case 6:
		    
		    ROS_INFO("action 6");
		    
		    for(int i=1; i<=20; i++)
		    {
		      if(i%2 == 0) manager.ToPose("left_arm", "left_pour", action, manager.pub_bartender_cmd_left,true, false);
		      else manager.ToPose("left_arm", "shaking", action, manager.pub_bartender_cmd_left,true, false);		      
		      sleep(1);
		    }
		    
		    manager.ToPose("left_arm", "left_pour", action, manager.pub_bartender_cmd_left,false, false);
		    
		    while(action==6){
			    std::cout << "Are You ready for serving? (y/n) " << std::endl;
			    getline (std::cin, ans);
			    
			    if(ans.compare("y") == 0) {
			      action = 7;
			      select = true;
			    }
			    else sleep(2);
		    }
		    
		    ros::spinOnce();
		    
		    break;
		    
		  // Serving action 
		  case 7:
		    
		    if(select) {
		      select = false;
		      manager.resetError(manager.error_right);
		      manager.resetError(manager.error_left);
		      manager.ToPose("left_arm", "serving", action, manager.pub_bartender_cmd_left,true, false);
		      ROS_INFO("Action 7");
		    }
		    
		        
		    if( manager.compare_error(manager.error_left, 0.1, 0.3) )
		    {
			ROS_INFO("Serving");
			
			manager.ToPose("left_arm", "serving", action, manager.pub_bartender_cmd_left,false, false);
			
			ros::Time Init_time = ros::Time::now();
			ros::Duration duration;

			while(action==7){
			  duration = ros::Time::now() - Init_time;
			  std::cout << duration.toSec() << std::endl;
			  
			  if(duration.toSec() > 7) {
			    action = 8;
			    select = true;
			  }
			  
			}
		        
		     }
		    
		    ros::spinOnce();
		    
		    break;
		    
		  // Stop Serving action 
		  case 8:
		    
		    if(select) {
		      select = false;
		      manager.resetError(manager.error_right);
		      manager.resetError(manager.error_left);
		      manager.ToPose("left_arm", "left_pour", action, manager.pub_bartender_cmd_left,true, false);
		      ROS_INFO("Action 8");
		    }
		    
		        
		    if( manager.compare_error(manager.error_left, 0.1, 0.1) )
		    {
			ROS_INFO("stop serving");
			
			manager.ToPose("left_arm", "left_pour", action, manager.pub_bartender_cmd_left,false, false);
			
			select = true;
			action = 9;
		        
		     }
		    
		    ros::spinOnce();
		    
		    break;
		}
		    
		    
		    

	}
		
	
	return 0;
}