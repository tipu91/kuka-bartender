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

	manager.Init();

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
			manager.Publish();

			if ( manager.compare_error(manager.x_err_right_v) && !manager.msg_right.arrived )
			{	
				ROS_INFO("DESTRO arrivato alla bottiglia!!");
				// manager.Grasping(closure_value, s_r);
				fin_r = !fin_r;	
				manager.msg_right.arrived = true;
			}

			if ( manager.compare_error(manager.x_err_left_v) && !manager.msg_left.arrived )
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

		manager.ToGlass();

		while (action == 2)
		{

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

		}

		//**********************************THIRD ACTION: Pouring****************************************************************

		manager.Pouring();

		while (action == 3)
		{

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

		}
	}
		
	
	return 0;
}