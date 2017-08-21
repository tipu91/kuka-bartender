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
		void DrinkSelection();
		void Publish();
		void Init();
		void Grasping(std::vector<int> closure_value, std::string s);
		void OpeningHand(std::vector<int> opening_value, std::string s);
		void ToGlass();
		void Pouring();
		void Stop_Pouring();
		void Dub();
		void InitialPosition();
		float Mod_Error(KDL::Frame err);
		// bool compare_error(double err);
		bool compare_error(double err[6]);
		double PoseDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
		
		// config file
		dynamic_reconfigure::Server<bartender_manager::managerConfig> server;
		dynamic_reconfigure::Server<bartender_manager::managerConfig>::CallbackType f;
		void config_callback(bartender_manager::managerConfig &config, uint32_t level);

		bool BottleGrasping = false;
		bool ActionPouring = false;
		bool Init_cond = false;

		bartender_control::bartender_msg msg_right;
		bartender_control::bartender_msg msg_left;
		
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
		double threshold_rot = 0.1;

		double x_err_right_v[6];
		double x_err_left_v[6];
		
		tf::StampedTransform world_T_vodka;
		tf::StampedTransform world_T_rum;
		tf::StampedTransform world_T_lemon;
		tf::StampedTransform world_T_coca;
		tf::StampedTransform world_T_glass;
		tf::StampedTransform fake;
		
		tf::TransformListener listener;

		int print;

		std::vector<double> vodka_;

	private:

		ros::Publisher pub_bartender_cmd_right;
		ros::Publisher pub_bartender_cmd_left;
		
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
		
		geometry_msgs::Pose vodka;
		geometry_msgs::Pose rum;
		geometry_msgs::Pose lemon;
		geometry_msgs::Pose coca;
		geometry_msgs::Pose glass;

		geometry_msgs::Pose pose_rot_;

		std::map<std::string,geometry_msgs::Pose > bottle;	//Positions array: the first fild is the name (STRING), second is the position (VECTOR) 

		float Z1_eul_bott, Y_eul_bott, Z2_eul_bott;
		double *q_bottle, *q_err_right, *q_err_left, *q_init_right, *q_init_left;

	};

#endif