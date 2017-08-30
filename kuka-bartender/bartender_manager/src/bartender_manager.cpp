#include "BartenderManager.h"

using namespace std;

BartenderManager::BartenderManager() 
{
   
    f = boost::bind(&BartenderManager::config_callback, this, _1, _2);   
    server.setCallback(f);
    
    pub_bartender_cmd_right = n_.advertise<bartender_control::bartender_msg>("/right_arm/bartender_control/command", 250);
    pub_bartender_cmd_left = n_.advertise<bartender_control::bartender_msg>("/left_arm/bartender_control/command", 250);
    
    pub_bartender_config_right = n_.advertise<bartender_control::cfg_msg>("/right_arm/bartender_control/config", 250);
    pub_bartender_config_left = n_.advertise<bartender_control::cfg_msg>("/left_arm/bartender_control/config", 250);
    
    joint_pub_l = n_.advertise<sensor_msgs::JointState>("/left_hand/joint_states", 1);
    joint_pub_r = n_.advertise<sensor_msgs::JointState>("/right_hand/joint_states", 1);

    sub_bartender_err_right = n_.subscribe("/right_arm/bartender_control/error", 250, &BartenderManager::checkCallback_right, this);
    sub_bartender_err_left = n_.subscribe("/left_arm/bartender_control/error", 250, &BartenderManager::checkCallback_left, this);

    sub_pose_right = n_.subscribe("/right_arm/bartender_control/position", 250, &BartenderManager::checkCallbackPoseright, this);
    sub_pose_left = n_.subscribe("/left_arm/bartender_control/position", 250, &BartenderManager::checkCallbackPoseleft, this);
    
    sub_bartender_init_right = n_.subscribe("/right_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_right_initial, this);
    sub_bartender_init_left = n_.subscribe("/left_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_left_initial, this);

    n_.param<int>("printscreen", print, 0);
    
    /*for(int i=0; i<6; i++) error_right[i] = 1;
    for(int i=0; i<6; i++) error_left[i] = 1;*/
}

BartenderManager::~BartenderManager() {}

void BartenderManager::config_callback(bartender_manager::managerConfig& config, uint32_t level)
{
    // controller proportional constants
    ROS_INFO("Reconfigure Request");
    
    msg_config.alpha1 = config.alpha_1;
    msg_config.alpha2 = config.alpha_2;
    msg_config.second_task = config.second_task;
    
    run_manager = config.run;
    
    /*pub_bartender_config_right.publish(msg_config);
    pub_bartender_config_left.publish(msg_config);*/
    
}

void BartenderManager::checkCallbackPoseright(const geometry_msgs::PoseStamped::ConstPtr & msg_pose) {
    
    x_right = msg_pose->pose;

}

void BartenderManager::checkCallbackPoseleft(const geometry_msgs::PoseStamped::ConstPtr & msg_pose) {
    
    x_left = msg_pose->pose;
}

//Function callback for right arm
void BartenderManager::checkCallback_right(const std_msgs::Float64MultiArray & msg_err) {
    
    for(int i=0; i<6; i++) error_right[i] = msg_err.data[i];

}

//Function callback for left arm
void BartenderManager::checkCallback_left(const std_msgs::Float64MultiArray & msg_err) {
    
    for(int i=0; i<6; i++) error_left[i] = msg_err.data[i];

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
	
	try{
	  
	  listener.waitForTransform( "bartender_anchor", "right_grasp", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "bartender_anchor", "right_grasp", ros::Time(0), world_T_rightGrasp);
	  
	  right_grasp.position.x = world_T_rightGrasp.getOrigin().getX(); 
	  right_grasp.position.y = world_T_rightGrasp.getOrigin().getY(); 
	  right_grasp.position.z = world_T_rightGrasp.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_rightGrasp.getRotation(), right_grasp.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  
	  listener.waitForTransform( "bartender_anchor", "left_grasp", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "bartender_anchor", "left_grasp", ros::Time(0), world_T_leftGrasp);
	  
	  left_grasp.position.x = world_T_leftGrasp.getOrigin().getX(); 
	  left_grasp.position.y = world_T_leftGrasp.getOrigin().getY(); 
	  left_grasp.position.z = world_T_leftGrasp.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_leftGrasp.getRotation(), left_grasp.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  
	  listener.waitForTransform( "bartender_anchor", "right_pour", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "bartender_anchor", "right_pour", ros::Time(0), world_T_rightPour);
	  
	  right_pour.position.x = world_T_rightPour.getOrigin().getX(); 
	  right_pour.position.y = world_T_rightPour.getOrigin().getY(); 
	  right_pour.position.z = world_T_rightPour.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_rightPour.getRotation(), right_pour.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  
	  listener.waitForTransform( "bartender_anchor", "left_pour", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "bartender_anchor", "left_pour", ros::Time(0), world_T_leftPour);
	  
	  left_pour.position.x = world_T_leftPour.getOrigin().getX(); 
	  left_pour.position.y = world_T_leftPour.getOrigin().getY(); 
	  left_pour.position.z = world_T_leftPour.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_leftPour.getRotation(), left_pour.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	try{
	  
	  listener.waitForTransform( "bartender_anchor", "pouring", ros::Time::now(), ros::Duration(3));
	  listener.lookupTransform( "bartender_anchor", "pouring", ros::Time(0), world_T_pouring);
	  
	  pouring.position.x = world_T_pouring.getOrigin().getX(); 
	  pouring.position.y = world_T_pouring.getOrigin().getY(); 
	  pouring.position.z = world_T_pouring.getOrigin().getZ();
	  tf::quaternionTFToMsg(world_T_pouring.getRotation(), pouring.orientation);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	
	pose["right_grasp"] = right_grasp;
	pose["left_grasp"] = left_grasp;
	pose["right_pour"] = right_pour;
	pose["left_pour"] = left_pour;
	pose["pouring"] = pouring;

}

void BartenderManager::ToPose(std::string arm, std::string target, ros::Publisher pub, bool run)
{
  
	bartender_control::bartender_msg msg; 
	
	msg.arm = arm;  
	msg.goal_tf = target;    
	msg.des_frame = pose[target];
	msg.run = run;
    
	pub.publish(msg);
	
	if(run) {
	  cout<<arm<<endl;
	  ROS_INFO("DES_POSE x=%f | y=%f | z=%f", msg.des_frame.position.x, msg.des_frame.position.y, msg.des_frame.position.z);
	}
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

bool BartenderManager::compare_error(double err[6], double thr_lin, double thr_rot)
{
	static bool near_p;

	if ( (fabs(err[0]) < thr_lin) && (fabs(err[1]) < thr_lin) && (fabs(err[2]) < thr_lin) &&  (fabs(err[3]) < thr_rot) && (fabs(err[4]) < thr_rot) && (fabs(err[5]) < thr_rot) ) near_p = true;
	else near_p = false;

	return near_p;

}