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

	bool finished = false;
	bool fin_l = false;
	bool fin_r = false;
	
	while (ros::ok())
	{
		
		int action = 1;

		manager.DrinkSelection();

	//**********************************************************************************************************************//
        //  							Sequence of actions aimed at the realization of cocktail							    //
        //**********************************************************************************************************************//

		//**********************************FIRST ACTION: To the bottles**********************************************************

		while (action == 1)
		{
			ros::spinOnce();
			
			manager.pub_bartender_config_right.publish(manager.msg_config);
			manager.pub_bartender_config_left.publish(manager.msg_config);
	
			manager.Publish();
			
			manager.error_lin_left = manager.PoseDistance(manager.x_des_l,manager.x_left);
			manager.error_lin_right = manager.PoseDistance(manager.x_des_r,manager.x_right);
			
			// ROS_INFO("errori lineari DX: %f | SX: %f", manager.error_lin_right, manager.error_lin_left);
			
			// if ( manager.compare_error(manager.error_lin_right) && !manager.msg_right.arrived )
			if ( manager.compare_error(manager.error_right) && !manager.msg_right.arrived )
			{	
				ROS_INFO("DESTRO arrivato alla bottiglia!!");
				// manager.Grasping(closure_value, s_r);
				fin_r = !fin_r;	
				manager.msg_right.arrived = true;
			}
			// if ( manager.compare_error(manager.error_lin_left) && !manager.msg_left.arrived )
			if ( manager.compare_error(manager.error_left) && !manager.msg_left.arrived )
			{	
				ROS_INFO("SINISTRO arrivato alla bottiglia!!");
				// manager.Grasping(closure_value, s_l);	
				fin_l = !fin_l;
				manager.msg_left.arrived = true;
			}

			if ( fin_r && fin_l )
			{
				ROS_INFO("FINITO (al bicchiere)!!");
				action = 2;
				fin_l = false;
				fin_r = false;
				ros::Duration(2.0).sleep();
			}

		}

		//**********************************SECOND ACTION: To the glass**********************************************************

		/*manager.ToGlass();

		while (action == 2)
		{
			ros::spinOnce();
			
			manager.Publish();

			if ( manager.compare_error(manager.x_err_right_v) && !manager.msg_right.arrived )
			{	
				ROS_INFO("DESTRO arrivato al bicchiere!!");
				fin_r = !fin_r;	
				manager.msg_right.arrived = true;
			}

			if ( manager.compare_error(manager.x_err_left_v) && !manager.msg_left.arrived )
			{	
				ROS_INFO("SINISTRO arrivato al bicchiere!!");
				fin_l = !fin_l;
				manager.msg_left.arrived = true;
			}

			if ( fin_r && fin_l )
			{
				ROS_INFO("FINITO (al bicchiere)!!");
				action = 3;
				fin_l = false;
				fin_r = false;
				ros::Duration(2.0).sleep();
			}

		}*/

		//**********************************THIRD ACTION: Pouring****************************************************************

		/*manager.Pouring();

		while (action == 3)
		{
			ros::spinOnce();
			
			manager.Publish();

			if ( manager.compare_error(manager.x_err_right_v) && !manager.msg_right.arrived )
			{	
				ROS_INFO("DESTRO versato!!");
				fin_r = !fin_r;	
				manager.msg_right.arrived = true;
			}

			if ( manager.compare_error(manager.x_err_left_v) && !manager.msg_left.arrived )
			{	
				ROS_INFO("SINISTRO versato!!");
				fin_l = !fin_l;
				manager.msg_left.arrived = true;
			}

			if ( fin_r && fin_l )
			{
				ROS_INFO("FINITO (versato)!!");
				action = 0;
				fin_l = false;
				fin_r = false;
				ros::Duration(4.0).sleep();
				manager.Stop_Pouring();
				manager.Publish();
			}

		}*/
	}
		
	
	return 0;
}