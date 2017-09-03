#ifndef BARTENDER_MANAGER_H
#define BARTENDER_MANAGER_H

#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <bartender_control/bartender_msg.h> 
#include <bartender_control/cfg_msg.h>
#include <Eigen/LU>
#include <dynamic_reconfigure/server.h>
#include <bartender_manager/managerConfig.h>
#include <XmlRpcValue.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#define		link7_to_palm	0.15		// m
#define 	pouring_angle	(M_PI/2)	// rad

#define _USE_MATH_DEFINES

class BartenderManager {

	public:
	    BartenderManager();
	    ~BartenderManager();

	    void checkCallback_right(const std_msgs::Float64MultiArray &msg_err);
	    void checkCallback_left(const std_msgs::Float64MultiArray &msg_err);
	    void checkCallback_right_initial(const geometry_msgs::Pose::ConstPtr &msg_init);
	    void checkCallback_left_initial(const geometry_msgs::Pose::ConstPtr &msg_init);
	    void checkCallbackPoseright(const geometry_msgs::PoseStamped::ConstPtr &msg_pose);
	    void checkCallbackPoseleft(const geometry_msgs::PoseStamped::ConstPtr &msg_pose);
		
		double *EulerToQuaternion(float R, float P, float Y);
		void Init();
		void Grasping(std::vector<int> action_value, std::string s);
		void ToPose(std::string arm, std::string target, int action, ros::Publisher pub, bool stop, bool print);
		bool compare_error(double err[6], double thr_lin, double thr_rot);
		void resetError(double *err);
		
		// config file
		dynamic_reconfigure::Server<bartender_manager::managerConfig> server;
		dynamic_reconfigure::Server<bartender_manager::managerConfig>::CallbackType f;
		void config_callback(bartender_manager::managerConfig &config, uint32_t level);

		bool BottleGrasping = false;
		bool ActionPouring = false;
		bool Init_cond = false;
		bool run_manager;

		bartender_control::bartender_msg msg_right;
		bartender_control::bartender_msg msg_left;
		
		bartender_control::cfg_msg msg_config;
		
		geometry_msgs::Pose grasp_des_right;
		geometry_msgs::Pose grasp_des_left;
		
		geometry_msgs::Pose x_right;
		geometry_msgs::Pose x_left;
		
		KDL::Frame x_err_right;
		KDL::Frame x_err_left;

		double error_lin_left, error_lin_right;
		double error_right[6], error_left[6];
		
		KDL::Frame x_err_compare;

		geometry_msgs::Pose x_right_initial;
		geometry_msgs::Pose x_left_initial;
		geometry_msgs::Pose x_des_r, x_des_l; 
		
		ros::NodeHandle n_;

		double threshold = 0.02;
		double threshold_rot = 0.05;

		tf::StampedTransform world_T_rightPour;
		tf::StampedTransform world_T_leftPour;
		
		tf::StampedTransform world_T_rightGrasp;
		tf::StampedTransform world_T_leftGrasp;

		tf::StampedTransform world_T_pouring;
		tf::StampedTransform world_T_shaking;
		tf::StampedTransform world_T_serving;
		
		tf::TransformListener listener;

		int print;

		std::vector<double> vodka_;
		
		bool grasp;

	// private:

		ros::Publisher pub_bartender_cmd_right;
		ros::Publisher pub_bartender_cmd_left;
		
		ros::Publisher pub_bartender_config_right;
		ros::Publisher pub_bartender_config_left;
		
		ros::Publisher joint_pub_l;
		ros::Publisher joint_pub_r;
		
		ros::Subscriber sub_bartender_err_right;
		ros::Subscriber sub_bartender_err_left;

		ros::Subscriber sub_bartender_init_right;
		ros::Subscriber sub_bartender_init_left;
		
		ros::Subscriber sub_pose_right;
		ros::Subscriber sub_pose_left;

		KDL::Frame x_;

		KDL::Frame x_bottle;
		geometry_msgs::Pose bottle_right, bottle_left;
		
		geometry_msgs::Pose right_pour;
		geometry_msgs::Pose left_pour;
		geometry_msgs::Pose right_grasp;
		geometry_msgs::Pose left_grasp;
		geometry_msgs::Pose pouring;
		geometry_msgs::Pose shaking;
		geometry_msgs::Pose serving;

		geometry_msgs::Pose pose_rot_;

		std::map<std::string,geometry_msgs::Pose > pose;	//Positions array: the first fild is the name (STRING), second is the position (VECTOR) 

		float Z1_eul_bott, Y_eul_bott, Z2_eul_bott;
		double *q_bottle, *q_err_right, *q_err_left, *q_init_right, *q_init_left;

	};

#endif