#include "BartenderManager.h"

using namespace std;

BartenderManager::BartenderManager() 
{
   
    f = boost::bind(&BartenderManager::config_callback, this, _1, _2);   
    server.setCallback(f);
    
    pub_bartender_cmd_right = n_.advertise<bartender_control::bartender_msg>("/right_arm/bartender_control/command", 250);
    pub_bartender_cmd_left = n_.advertise<bartender_control::bartender_msg>("/left_arm/bartender_control/command", 250);
    
    joint_pub_l = n_.advertise<sensor_msgs::JointState>("/left_hand/joint_states", 1);
    joint_pub_r = n_.advertise<sensor_msgs::JointState>("/right_hand/joint_states", 1);

    sub_bartender_err_right = n_.subscribe("/right_arm/bartender_control/error", 250, &BartenderManager::checkCallback_right, this);
    sub_bartender_err_left = n_.subscribe("/left_arm/bartender_control/error", 250, &BartenderManager::checkCallback_left, this);

    sub_pose_right = n_.subscribe("/right_arm/bartender_control/position", 250, &BartenderManager::checkCallbackPoseright, this);
    sub_pose_left = n_.subscribe("/left_arm/bartender_control/position", 250, &BartenderManager::checkCallbackPoseleft, this);
    
    sub_bartender_init_right = n_.subscribe("/right_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_right_initial, this);
    sub_bartender_init_left = n_.subscribe("/left_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_left_initial, this);
   
   	for(int i=0; i<6; i++) x_err_right_v[i] = 1;
   	for(int i=0; i<6; i++) x_err_left_v[i] = 1;

   	n_.param<int>("printscreen", print, 0);
    
}

BartenderManager::~BartenderManager() {}

void BartenderManager::config_callback(bartender_manager::managerConfig& config, uint32_t level)
{
    // controller proportional constants
    ROS_INFO("Reconfigure Request");

   
}

void BartenderManager::checkCallbackPoseright(const geometry_msgs::PoseStamped::ConstPtr & msg_pose) {
    
    x_right = msg_pose->pose;

}

void BartenderManager::checkCallbackPoseleft(const geometry_msgs::PoseStamped::ConstPtr & msg_pose) {
    
    x_left = msg_pose->pose;
}

//Function callback for right arm
void BartenderManager::checkCallback_right(const std_msgs::Float64MultiArray & msg_err) {
    
    for(int i=0; i<6; i++) x_err_right_v[i] = msg_err.data[i];

    x_err_right.p(0) = msg_err.data[0];
    x_err_right.p(1) = msg_err.data[1];
    x_err_right.p(2) = msg_err.data[2];

    x_err_right.M = KDL::Rotation::EulerZYX(msg_err.data[3], msg_err.data[4], msg_err.data[5]);

}

//Function callback for left arm
void BartenderManager::checkCallback_left(const std_msgs::Float64MultiArray & msg_err) {
    
    for(int i=0; i<6; i++) x_err_left_v[i] = msg_err.data[i];

    x_err_left.p(0) = msg_err.data[0];
    x_err_left.p(1) = msg_err.data[1];
    x_err_left.p(2) = msg_err.data[2];

    x_err_left.M = KDL::Rotation::EulerZYX(msg_err.data[3], msg_err.data[4], msg_err.data[5]);

}

//Function callback for right arm initial position
void BartenderManager::checkCallback_right_initial(const geometry_msgs::Pose::ConstPtr &msg_init) {
    
    x_right_initial = *msg_init;
    
}

//Function callback for left arm initial position
void BartenderManager::checkCallback_left_initial(const geometry_msgs::Pose::ConstPtr &msg_init) {
    
    x_left_initial = *msg_init;
      
}

//This function initializes the bottle map (string,frame)
void BartenderManager::Init ()
{
	//***********************This piece of code is for wait the real transforms*******************************
	try{
	  listener.waitForTransform( "world_link", "world",ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "world_link", "world", ros::Time(0), fake);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	//*********************************************************************************************************
	
	try{
	  listener.waitForTransform( "world_link", "vodka",ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "world_link", "vodka", ros::Time(0), world_T_vodka);
	  
	  vodka.position.x = world_T_vodka.getOrigin().getX(); 
	  vodka.position.y = world_T_vodka.getOrigin().getY(); 
	  vodka.position.z = world_T_vodka.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_vodka.getRotation(), vodka.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  listener.waitForTransform( "world_link","rum", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "world_link", "rum", ros::Time(0), world_T_rum);
	  
	  rum.position.x = world_T_rum.getOrigin().getX(); 
	  rum.position.y = world_T_rum.getOrigin().getY(); 
	  rum.position.z = world_T_rum.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_rum.getRotation(), rum.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  
	  listener.waitForTransform( "world_link", "coca", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "world_link", "coca", ros::Time(0), world_T_coca);
	  
	  coca.position.x = world_T_coca.getOrigin().getX(); 
	  coca.position.y = world_T_coca.getOrigin().getY(); 
	  coca.position.z = world_T_coca.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_coca.getRotation(), coca.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  
	  listener.waitForTransform( "world_link", "lemon", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "world_link", "lemon", ros::Time(0), world_T_lemon);
	  
	  lemon.position.x = world_T_lemon.getOrigin().getX(); 
	  lemon.position.y = world_T_lemon.getOrigin().getY(); 
	  lemon.position.z = world_T_lemon.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_lemon.getRotation(), lemon.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  
	  listener.waitForTransform( "world_link", "glass", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "world_link", "glass", ros::Time(0), world_T_glass);
	  
	  glass.position.x = world_T_glass.getOrigin().getX(); 
	  glass.position.y = world_T_glass.getOrigin().getY(); 
	  glass.position.z = world_T_glass.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_glass.getRotation(), glass.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
      
	
	bottle["vodka"] = vodka;
	bottle["lemon"] = lemon;
	bottle["rum"] = rum;
	bottle["coca"] = coca;
	bottle["glass"] = glass;
	
	ROS_INFO("vodka: x = %f | y = %f | z = %f", vodka.position.x, vodka.position.y, vodka.position.z);
	ROS_INFO("rum: x = %f | y = %f | z = %f", rum.position.x, rum.position.y, rum.position.z);
	ROS_INFO("coca: x = %f | y = %f | z = %f", coca.position.x, coca.position.y, coca.position.z);
	ROS_INFO("lemon: x = %f | y = %f | z = %f", lemon.position.x, lemon.position.y, lemon.position.z);
	ROS_INFO("glass: x = %f | y = %f | z = %f", glass.position.x, glass.position.y, glass.position.z);

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
		  
		    msg_right.arm = "right_arm";		  
		    msg_right.goal_tf = bot.first;		    
		    msg_right.des_frame = bot.second;
		    
		    x_des_r = bot.second;

		    bottle_right = bot.second;

		    count_r = true;
		}
		
  	}

  	for (auto bot : bottle){
  		if(!choise2.compare(bot.first)){

		    msg_left.goal_tf = bot.first;
		    msg_left.arm = "left_arm";	    
		    msg_left.des_frame = bot.second;
		    
		    x_des_l = bot.second;

		    msg_left.arrived = false;

		    bottle_left = bot.second;

		    count_l = true;
  		}
  		
  	}

  	if (!count_r || !count_l) 
  	{
  		ROS_INFO("You wrote bad!!");
  		BartenderManager::DrinkSelection();
  	}

  	cout << "You have chosen " << choise1 << " and " << choise2 << endl;

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

}

void BartenderManager::ToGlass()
{

	string glassDx = "glass";
	string glassSx = "glass";

	for (auto bot : bottle){
  		if(!glassDx.compare(bot.first)){
		    
		    msg_right.des_frame = bot.second;

		    msg_right.arrived = false;

		 }
  	}

  	for (auto bot : bottle){
  		if(!glassSx.compare(bot.first)){
		    
		    msg_right.des_frame = bot.second;

		    msg_left.arrived = false;

  		}
  	}
  	
  	BartenderManager::Publish();
	
}

void BartenderManager::Pouring()
{
	msg_right.arrived = false;

	msg_left.arrived = false;

}

void BartenderManager::Stop_Pouring()
{

	msg_right.des_frame = bottle_right;
	msg_right.arrived = false;


	msg_left.des_frame = bottle_left;	
	msg_left.arrived = false;


}

void BartenderManager::InitialPosition()
{

	msg_right.arrived = false;
	msg_left.arrived = false;

	msg_right.des_frame = x_right_initial;
	msg_left.des_frame = x_left_initial;

}

// :-)
void BartenderManager::Dub()
{
	msg_right.des_frame.position.x = -0.3;
	msg_right.des_frame.position.y = 3;
	msg_right.des_frame.position.z = 1;

	msg_right.arrived = false;


  	msg_left.des_frame.position.x = -0.5;
	msg_left.des_frame.position.y = 0.4;
	msg_left.des_frame.position.z = 0.4;

	msg_left.arrived = false;

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

/*bool BartenderManager::compare_error(double err)
{
	static bool near_p;

	if (err < threshold) near_p = true;
	else near_p = false;

	return near_p;

}*/

double BartenderManager::PoseDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{

    double Ex = pose2.position.x - pose1.position.x;  
    double Ey = pose2.position.y - pose1.position.y;
    double Ez = pose2.position.z - pose1.position.z;
    
    double Etx = sqrt( Ex*Ex + Ey*Ey + Ez*Ez );
    
    return Etx;
}