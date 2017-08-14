#include "BartenderManager.h"

using namespace std;

BartenderManager::BartenderManager() 
{
   
    pub_bartender_cmd_right = n_.advertise<bartender_control::bartender_msg>("/right_arm/bartender_control/command", 250);
    pub_bartender_cmd_left = n_.advertise<bartender_control::bartender_msg>("/left_arm/bartender_control/command", 250);

    joint_pub_l = n_.advertise<sensor_msgs::JointState>("/left_hand/joint_states", 1);
    joint_pub_r = n_.advertise<sensor_msgs::JointState>("/right_hand/joint_states", 1);

    sub_bartender_err_right = n_.subscribe("/right_arm/bartender_control/error", 250, &BartenderManager::checkCallback_right, this);
    sub_bartender_err_left = n_.subscribe("/left_arm/bartender_control/error", 250, &BartenderManager::checkCallback_left, this);

    sub_bartender_init_right = n_.subscribe("/right_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_right_initial, this);
    sub_bartender_init_left = n_.subscribe("/left_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_left_initial, this);
   
   	for(int i=0; i<6; i++) x_err_right_v[i] = 1;
   	for(int i=0; i<6; i++) x_err_left_v[i] = 1;

   	n_.param<int>("printscreen", print, 0);
}

BartenderManager::~BartenderManager() {}

//Function callback for right arm
void BartenderManager::checkCallback_right(const std_msgs::Float64MultiArray & msg_err) {
    
    for(int i=0; i<6; i++) x_err_right_v[i] = msg_err.data[i];

    x_err_right.p(0) = msg_err.data[0];
    x_err_right.p(1) = msg_err.data[1];
    x_err_right.p(2) = msg_err.data[2];

    x_err_right.M = KDL::Rotation::EulerZYZ(msg_err.data[3], msg_err.data[4], msg_err.data[5]);

}

//Function callback for left arm
void BartenderManager::checkCallback_left(const std_msgs::Float64MultiArray & msg_err) {
    
    for(int i=0; i<6; i++) x_err_left_v[i] = msg_err.data[i];

    x_err_left.p(0) = msg_err.data[0];
    x_err_left.p(1) = msg_err.data[1];
    x_err_left.p(2) = msg_err.data[2];

    x_err_left.M = KDL::Rotation::EulerZYZ(msg_err.data[3], msg_err.data[4], msg_err.data[5]);

}

//Function callback for right arm initial position
void BartenderManager::checkCallback_right_initial(const std_msgs::Float64MultiArray &msg_init) {
    
    x_right_initial.p(0) = msg_init.data[0];
    x_right_initial.p(1) = msg_init.data[1];
    x_right_initial.p(2) = msg_init.data[2];

    q_init_right = BartenderManager::EulerToQuaternion(msg_init.data[3], msg_init.data[4], msg_init.data[5]);

    x_right_initial = KDL::Frame(KDL::Rotation::Quaternion(q_init_right[0], q_init_right[1], q_init_right[2], q_init_right[3]), x_right_initial.p);
    
}

//Function callback for left arm initial position
void BartenderManager::checkCallback_left_initial(const std_msgs::Float64MultiArray &msg_init) {
    
    x_left_initial.p(0) = msg_init.data[0];
    x_left_initial.p(1) = msg_init.data[1];
    x_left_initial.p(2) = msg_init.data[2];

    q_init_left = BartenderManager::EulerToQuaternion(msg_init.data[3], msg_init.data[4], msg_init.data[5]);

    x_left_initial = KDL::Frame(KDL::Rotation::Quaternion(q_init_left[0], q_init_left[1], q_init_left[2], q_init_left[3]), x_left_initial.p);
    
}

//This function initializes the bottle map (string,frame)
void BartenderManager::Init ()
{
	/*n_.param<std::vector<double> >("vodka", vodka_, {0, 0, 0});

	x_bottle.p(0) = vodka_.at(0) + link7_to_palm;
	x_bottle.p(1) = vodka_.at(1);
	x_bottle.p(2) = vodka_.at(2);*/

	x_bottle.p(0) = -1.15;
	x_bottle.p(1) = 0.15;
	x_bottle.p(2) = 0.15;	

	Z1_eul_bott = 0;
	Y_eul_bott = -M_PI/2;			//-M_PI/2;		// 0	-90
	Z2_eul_bott = M_PI/2;			//M_PI/2;		// 90	90

	x_bottle = KDL::Frame(KDL::Rotation::EulerZYZ(Z1_eul_bott, Y_eul_bott, Z2_eul_bott), x_bottle.p);

	bottle["vodka"] = x_bottle;

	x_bottle.p(1) = -0.15;
	bottle["lemon"] = x_bottle;

	x_bottle.p(1) = 0.3;
	bottle["rum"] = x_bottle;
  
	x_bottle.p(1) = -0.3;
	bottle["coca"] = x_bottle;

	x_bottle.p(0) = -1.5;
	x_bottle.p(1) = 0;
	x_bottle.p(2) = 0.25;
	bottle["glass"] = x_bottle;

}

//Function who transforms Euler agles (RPY) in quaternion
double *BartenderManager::EulerToQuaternion(float R, float P, float Y)
{
	
	static double q_[4];

	double t0 = std::cos(Y*0.5f);
	double t1 = std::sin(Y*0.5f);
	double t2 = std::cos(R*0.5f);
	double t3 = std::sin(R*0.5f);
	double t4 = std::cos(P*0.5f);
	double t5 = std::sin(P*0.5f);

	q_[0] = t0 * t2 * t4 + t1 * t3 * t5;
	q_[1] = t0 * t3 * t4 - t1 * t2 * t5;
	q_[2] = t0 * t2 * t5 + t1 * t3 * t4;
	q_[3] = t1 * t2 * t4 - t0 * t3 * t5;

	return q_;

}

//Function who let the user insert 2 bottle and it creates 2 msg
void BartenderManager::DrinkSelection ()
{	
	string choise1, choise2;

  	cout << "Please, enter the first bottle (rum, vodka, lemon, coca): " << endl;
  	getline (cin, choise1);

  	cout << "Please, enter the second bottle (rum, vodka, lemon, coca): " << endl;
  	getline (cin, choise2);

  	bool count_r = false;
  	bool count_l = false;

  	for (auto bot : bottle){
  		if(!choise1.compare(bot.first)){
		    msg_right.des_frame.position.x = bot.second.p(0);
		    msg_right.des_frame.position.y = bot.second.p(1);
		    msg_right.des_frame.position.z = bot.second.p(2);

		    //Put desired EE orientation in the control message
		    msg_right.des_frame.orientation.x = Z1_eul_bott;
		    msg_right.des_frame.orientation.y = Y_eul_bott;
		    msg_right.des_frame.orientation.z = Z2_eul_bott;

		    msg_right.arrived = false;

		    msg_right.position_task_gain = 8;
		    msg_right.gravity_task_gain = 0.4;

		    msg_right.multi_task = true;

		    bottle_right.p = bot.second.p;
		    bottle_right.M = bot.second.M;

		    count_r = true;
		}
		
  	}

  	for (auto bot : bottle){
  		if(!choise2.compare(bot.first)){
  			msg_left.des_frame.position.x = bot.second.p(0);
		    msg_left.des_frame.position.y = bot.second.p(1);
		    msg_left.des_frame.position.z = bot.second.p(2);

		    //Put desired EE orientation in the control message
		    msg_left.des_frame.orientation.x = Z1_eul_bott;
		    msg_left.des_frame.orientation.y = Y_eul_bott;
		    msg_left.des_frame.orientation.z = Z2_eul_bott;

		    msg_left.arrived = false;
		    
		    msg_left.position_task_gain = 8;
		    msg_left.gravity_task_gain = 0.4;

		    msg_left.multi_task = true;

		    bottle_left.p = bot.second.p;
		    bottle_left.M = bot.second.M;

		    count_l = true;
  		}
  		
  	}

  	if (!count_r || !count_l) 
  	{
  		ROS_INFO("You wrote bad!!");
  		BartenderManager::DrinkSelection();
  	}

  	cout << "You have chosen " << choise1 << " and " << choise2 << endl;

  	msg_right.do_rot_z = false;
	msg_left.do_rot_z = false;

	BartenderManager::Publish();

}

void BartenderManager::Grasping(std::vector<int> closure_value, std::string s)
{
	ROS_INFO("Grasping function!!");
	
	sensor_msgs::JointState joint_state_r;
	sensor_msgs::JointState joint_state_l;

	joint_state_l.header.stamp = ros::Time::now();
	joint_state_r.header.stamp = joint_state_l.header.stamp;

	if(s == "left")
	{
		joint_state_l.name.push_back("left_hand_synergy_joint");
		joint_state_l.position.push_back(closure_value[0]);
	}

	if(s == "right")
	{
		joint_state_r.name.push_back("right_hand_synergy_joint");
		joint_state_r.position.push_back(closure_value[1]);
	}
	
	joint_pub_r.publish(joint_state_r);
	joint_pub_l.publish(joint_state_l);
	// ros::spinOnce();

}

void BartenderManager::OpeningHand(std::vector<int> opening_value, std::string s)
{
	ROS_INFO("Opening Hand function!!");
	sensor_msgs::JointState joint_state_r;
	sensor_msgs::JointState joint_state_l;

	joint_state_l.header.stamp = ros::Time::now();
	joint_state_r.header.stamp = joint_state_l.header.stamp;	

	if(s == "left")
	{
		joint_state_l.name.push_back("left_hand_synergy_joint");
		joint_state_l.position.push_back(opening_value[0]);
	}

	if(s == "right")
	{
		joint_state_r.name.push_back("right_hand_synergy_joint");
		joint_state_r.position.push_back(opening_value[1]);
	}
	
 	joint_pub_r.publish(joint_state_r);
 	joint_pub_l.publish(joint_state_l);
	// ros::spinOnce();

}

void BartenderManager::ToGlass()
{

	string glassDx = "glass";
	string glassSx = "glass";

	msg_right.do_rot_z = false;
	msg_left.do_rot_z = false;

	for (auto bot : bottle){
  		if(!glassDx.compare(bot.first)){
		    msg_right.des_frame.position.x = bot.second.p(0);
		    msg_right.des_frame.position.y = bot.second.p(1) + 0.15;
		    msg_right.des_frame.position.z = bot.second.p(2);

		    msg_right.arrived = false;

		    msg_right.position_task_gain = 8;
		    msg_right.gravity_task_gain = 0.4;

		    msg_right.multi_task = true;
		 }
  	}

  	for (auto bot : bottle){
  		if(!glassSx.compare(bot.first)){
		    msg_left.des_frame.position.x = bot.second.p(0);
		    msg_left.des_frame.position.y = bot.second.p(1) - 0.15;
		    msg_left.des_frame.position.z = bot.second.p(2);

		    msg_left.arrived = false;
		    
		    msg_left.position_task_gain = 8;
		    msg_left.gravity_task_gain = 0.4;

		    msg_left.multi_task = true;

  		}
  	}
  	
  	BartenderManager::Publish();
	
}

void BartenderManager::Pouring()
{

	msg_right.position_task_gain = 8;
	msg_right.gravity_task_gain = 0.4;
	msg_right.multi_task = true;
	
	msg_right.do_rot_z = false;
	msg_right.rot_z = -pouring_angle;
	
	msg_right.arrived = false;



	msg_left.position_task_gain = 8;
	msg_left.gravity_task_gain = 0.4;
	msg_left.multi_task = true;

	msg_left.do_rot_z = false;
	msg_left.rot_z = pouring_angle;

	msg_left.arrived = false;

}

void BartenderManager::Stop_Pouring()
{

	msg_right.des_frame.position.x = bottle_right.p(0);
	msg_right.des_frame.position.y = bottle_right.p(1);
	msg_right.des_frame.position.z = bottle_right.p(2);

	msg_right.position_task_gain = 8;
	msg_right.gravity_task_gain = 0.4;
	msg_right.multi_task = true;
	
	msg_right.do_rot_z = false;
	msg_right.rot_z = pouring_angle;

	msg_right.arrived = false;


	msg_left.des_frame.position.x = bottle_left.p(0);
	msg_left.des_frame.position.y = bottle_left.p(1);
	msg_left.des_frame.position.z = bottle_left.p(2);
	
	msg_left.position_task_gain = 8;
	msg_left.gravity_task_gain = 0.4;
	msg_left.multi_task = true;
	
	msg_left.do_rot_z = false;
	msg_left.rot_z = -pouring_angle;
	
	msg_left.arrived = false;


}

void BartenderManager::InitialPosition()
{

	msg_right.arrived = false;
	msg_left.arrived = false;

	msg_right.des_frame.position.x = x_right_initial.p(0);
	msg_right.des_frame.position.y = x_right_initial.p(1);
	msg_right.des_frame.position.z = x_right_initial.p(2);

	/*msg_right.des_frame.orientation.x = q_bottle[0];
	msg_right.des_frame.orientation.y = q_bottle[1];
	msg_right.des_frame.orientation.z = q_bottle[2];
	msg_right.des_frame.orientation.w = q_bottle[3];*/

	msg_left.des_frame.position.x = x_left_initial.p(0);
	msg_left.des_frame.position.y = x_left_initial.p(1);
	msg_left.des_frame.position.z = x_left_initial.p(2);

	/*msg_left.des_frame.orientation.x = q_bottle[0];
	msg_left.des_frame.orientation.y = q_bottle[1];
	msg_left.des_frame.orientation.z = q_bottle[2];
	msg_left.des_frame.orientation.w = q_bottle[3];*/

}

// :-)
void BartenderManager::Dub()
{
	msg_right.des_frame.position.x = -0.3;
	msg_right.des_frame.position.y = 3;
	msg_right.des_frame.position.z = 1;

	msg_right.arrived = false;

	msg_right.position_task_gain = 8;
	msg_right.gravity_task_gain = 2;

	msg_right.multi_task = true;



  	msg_left.des_frame.position.x = -0.5;
	msg_left.des_frame.position.y = 0.4;
	msg_left.des_frame.position.z = 0.4;


    msg_left.arrived = false;
		    
    msg_left.position_task_gain = 8;
    msg_left.gravity_task_gain = 0.4;
    msg_left.multi_task = false;
}

//Function who publishes 2 msg
void BartenderManager::Publish() 
{
	pub_bartender_cmd_right.publish(msg_right);
	pub_bartender_cmd_left.publish(msg_left);
}

float BartenderManager::Mod_Error(KDL::Frame err)
{
	static float d;

	float d2 = ( err.p(0)*err.p(0) ) + ( err.p(1)*err.p(1) ) + ( err.p(2)*err.p(2) );
	d = sqrt(d2);

	return d;
}

bool BartenderManager::compare_error(double err[6])
{
	static bool near_p;

	if ( (err[0] < threshold) && (err[1] < threshold) && (err[2] < threshold) &&  (err[3] < threshold_rot) && (err[4] < threshold_rot) && (err[5] < threshold_rot) ) near_p = true;
	else near_p = false;

	return near_p;

}
