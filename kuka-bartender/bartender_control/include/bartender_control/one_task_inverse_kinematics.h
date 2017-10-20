#ifndef BARTENDER_CONTROL__ONE_TASK_INVERSE_KINEMATICS_H
#define BARTENDER_CONTROL__ONE_TASK_INVERSE_KINEMATICS_H

#include <lwr_controllers/KinematicChainControllerBase.h>

// #include <bartender_manager/BartenderManager.h">

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <bartender_control/bartender_msg.h> 
#include <bartender_control/cfg_msg.h> 

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <time.h>

#include<bartender_control/bartender_srv.h>

#include <dynamic_reconfigure/server.h>

#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <tf/transform_listener.h>

#include <XmlRpc.h>

namespace bartender_control
{
	class OneTaskInverseKinematics: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
	public:
		OneTaskInverseKinematics();
		~OneTaskInverseKinematics();
		
		bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
		void FrameToPose(KDL::Frame &frame, geometry_msgs::PoseStamped &pose);
		void update(const ros::Time& time, const ros::Duration& period);
		void commandCallback(const bartender_control::bartender_msg::ConstPtr &msg);
		bool commandReceived(bartender_srv::Request& req, bartender_srv::Response& res);
		void param_update();
		void configCallback(const bartender_control::cfg_msg::ConstPtr &msg);
		void Error(geometry_msgs::PoseStamped& p_curr, geometry_msgs::PoseStamped& p_des, std::vector<double> &error);
		Eigen::Matrix<double, 7, 1> potentialEnergy(KDL::JntArray q);
		
		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;
		
		ros::NodeHandle pnh;
		std::string ns_param;
		std::string controller;
		
		inline const char * const BoolToString(bool b)
		{
		  return b ? "true" : "false";
		}

		ros::Subscriber sub_bartender_cmd, sub_bartender_config;
		
		ros::Subscriber sub_bartender_goal, sub_bartender_tf;
		ros::ServiceServer cmd_server;
		
		ros::Publisher pub_check_error;
		ros::Publisher pub_check_initial;
		ros::Publisher pub_pose;
		ros::Publisher des_pose_bag;

		KDL::Frame x_;		//current pose
		geometry_msgs::PoseStamped x_pose;
		
		KDL::Frame x_initial;		//initial pose
		geometry_msgs::PoseStamped x_initial_pose;
		
		KDL::Frame x_des_;	//desired pose
		geometry_msgs::PoseStamped x_des_pose;
		
		geometry_msgs::PoseStamped x_diff;

		
		// KDL::Frame x_err_;	//error position

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;	//Pseudoinverse jacobian of dynamic dimension (double)
		// Eigen::Matrix<double,3,3> skew_;	//skew-matrix (3x3) of double
		Eigen::Matrix<double, 7, 7> P_null;	// Null-space projector

		Eigen::Matrix<double, 7, 7> K_p;
		Eigen::Matrix<double, 7, 7> K_d;
		
		std::vector<double> x_error;
		//double x_error[6];
		
		KDL::Vector rot_err;
		KDL::Vector lin_err;
		
		Eigen::Matrix<double, 7, 1> q_null;
		
		tf::StampedTransform Goal_T_Ee, W_T_Ee, W_T_Goal;
		tf::TransformListener listener;
		
		std::string goal_ref;

		double Roll_x_init, Pitch_x_init, Yaw_x_init;

		// KDL::Vector v_temp_;
		//KDL::JntArray G_local;

		int cmd_flag_;
		int action;
		bool second_task;

		double alpha1, alpha2;

		double x_or[3], x_des_or[3], x_pose_or[3], x_des_pose_or[3], x_diff_or[3];
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;		// Class to calculate the jacobian of a general KDL::Chain, it is used by other solvers. It should not be used outside of KDL.
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;	// Implementation of a recursive forward position kinematics algorithm to calculate the position transformation from joint space to Cartesian space of a general kinematic chain (KDL::Chain)
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;		// Implementation of a inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the velocity transformation from Cartesian to joint space of a general KDL::Chain. It uses a svd-calculation based on householders rotations
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;		// Implementation of a general inverse position kinematics algorithm based on Newton-Raphson iterations to calculate the position transformation from Cartesian to joint space of a general KDL::Chain. Takes joint limits into account.
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;			// Implementation of inverse dynamic resolver (to calculate Gravity matrix)
	  
	};

}

#endif
