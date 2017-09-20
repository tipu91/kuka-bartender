#include <bartender_control/one_task_inverse_kinematics.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>
#include "std_msgs/Bool.h"

using namespace std;

namespace bartender_control 
{
    OneTaskInverseKinematics::OneTaskInverseKinematics():pnh("~") {
	  pnh.param<std::string>("ns_arm", ns_param, "none_ns");
	  pnh.param<std::string>("class", controller,"bartender_control");
	  
    }	//costruttore
    
    OneTaskInverseKinematics::~OneTaskInverseKinematics() {}	//distruttore

    bool OneTaskInverseKinematics::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }

        cout << "debug: INIT function" << endl;

	 
	 
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());

        J_.resize(kdl_chain_.getNrOfJoints());

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        //Definition of publishers and subscribes

        pub_check_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 250);
        pub_check_initial = nh_.advertise<geometry_msgs::Pose>("initial_position", 250);
	pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("position", 250);
	
	des_pose_bag = nh_.advertise<geometry_msgs::PoseStamped>("des_pose", 250);

        sub_bartender_cmd = nh_.subscribe("command", 250, &OneTaskInverseKinematics::commandCallback, this);
	sub_bartender_config = nh_.subscribe("config",250, &OneTaskInverseKinematics::configCallback, this);

	x_error.resize(6);
	
	//*********************************Initialization default values*********************************//
	alpha1 = 8;
	alpha2 = 0.16;
	second_task = true;

        cmd_flag_ = 0;
	action = 0;
	
        return true;
    }
    
    
  void OneTaskInverseKinematics::configCallback(const bartender_control::cfg_msg::ConstPtr &msg)
  {
      // controller proportional constants
      ROS_INFO("Reconfigure Request");
      
      second_task = msg->second_task;
      alpha1 = msg->alpha1;
      alpha2 = msg->alpha2;
    
      ROS_INFO("ALPHA 1 = %f", alpha1);
      ROS_INFO("ALPHA 2 = %f", alpha2);
      ROS_INFO("Second task = %s", bartender_control::OneTaskInverseKinematics::BoolToString(second_task));
      
      return;
  }

    void OneTaskInverseKinematics::FrameToPose(KDL::Frame &frame, geometry_msgs::PoseStamped &pose)
    {
     
      pose.pose.position.x = frame.p(0);
      pose.pose.position.y = frame.p(1);
      pose.pose.position.z = frame.p(2);
      
      frame.M.GetQuaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
      
    }


    void OneTaskInverseKinematics::commandCallback(const bartender_control::bartender_msg::ConstPtr &msg)
    {
	
	//************************** reading message from manager **************************
	x_des_pose.pose = msg->des_frame;
	
        if (msg->run) cmd_flag_ = 1;
        if (!msg->run) cmd_flag_ = 0;
	
	goal_ref = msg->goal_tf;
	
	ns_param = msg->arm;	
	
	action = msg->action;
	
	cout << "received message from " << ns_param << " and ACTION = " << action << endl; 
	//**********************************************************************************
	
	return;
    }



    //  Updating parameters for controller
    void OneTaskInverseKinematics::param_update()
    {

        // computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

        // computing J_pinv_
        pseudo_inverse(J_.data, J_pinv_);

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
	
	bartender_control::OneTaskInverseKinematics::FrameToPose(x_,x_pose);

	bartender_control::OneTaskInverseKinematics::Error(x_pose,x_des_pose, x_error);
	
	des_pose_bag.publish(x_des_pose);
	
	std_msgs::Float64MultiArray msg_error;
	for(int i = 0; i < x_error.size(); i++) msg_error.data.push_back( x_error.at(i) );
	pub_check_error.publish(msg_error);

    }
    
    //  Controller function:: Multy Task Inverse Kinematics
    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {
	// precision movement 
	if ( action == 3 && ns_param == "right_arm") second_task = false;
	else second_task = true;
	
	// fast shaking (???)
	if ( action == 6 && ns_param == "left_arm") alpha1 = 20;
	else alpha1 = 8;

        if (!second_task) alpha1 = 5;

        // std::cout << "cmd_flag_ = " << cmd_flag_ << std::endl;

        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();            
        }

        bartender_control::OneTaskInverseKinematics::param_update();    //  Calculation of parameters used by controller

        //**********************************************************************************************************************//
        //  In this section you can find the MultyTask Kinematic control. The first task (alpha1) is the position control of    //
        //  the kuka-bartender to the desired position (x_des_) using the inverse kinematic control q = alpha1*pinv(J)*x_err_.  //
        //  The second task is the maximization of the potential energy of every link q = alpha2*(I-pinv(J)*J)*G, where G is    //
        //  the Gravity matrix used in the dynamic joint space model (G is a 7x1 array).                                        //
        //**********************************************************************************************************************//

        if (cmd_flag_)
        {

            //**********************************FIRST TASK***********************************************************************
            
            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += alpha1 * J_pinv_(i,k) * x_error.at(k);
          
            }

            //**********************************SECOND TASK**********************************************************************

            if (second_task)
            {
                //  Null-projector (I-pinv(J)*J)
                P_null =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_ * J_.data;

                //  Creation of the second task
                q_null = P_null * potentialEnergy( joint_msr_states_.q );

                for (int i = 0; i < J_pinv_.rows(); i++)
                {

                    joint_des_states_.qdot(i) += alpha2 * q_null[i];

                }

            }

            //*******************************************************************************************************************

            // integrating q_dot -> getting q (Euler method)
            // FIXME joint limits problems: with limits, robot doesn't work well
            //cout<<ns_param<<endl;
            for (int i = 0; i < joint_handles_.size(); i++){
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);
		//cout<<"joint "<<i+1<<":"<<joint_des_states_.q(i)<<endl;
		if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
		//cout<<"SAT joint "<<i+1<<":"<<joint_des_states_.q(i)<<endl;
	    }
	    
	    geometry_msgs::PoseStamped msg_pose;
	    msg_pose = x_pose;
	    pub_pose.publish(msg_pose);
            
        }
        else	// If control is not running
        {
            cmd_flag_ = 0;

            for(int i=0; i < joint_handles_.size(); i++)
            {
                joint_msr_states_.q(i) = 0;
            }

            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_initial);
	    
	    bartender_control::OneTaskInverseKinematics::FrameToPose(x_initial,x_initial_pose);
            
            geometry_msgs::Pose msg_initial;
            msg_initial = x_initial_pose.pose;
            pub_check_initial.publish(msg_initial);

        }

        

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }

    }

    //  Calulate the gravity vector for potential energy control (in the second taskS)
    Eigen::Matrix<double, 7, 1> OneTaskInverseKinematics::potentialEnergy(KDL::JntArray q)
    {

        KDL::JntArray G_local(7);
        id_solver_->JntToGravity(joint_msr_states_.q, G_local);
	
        return G_local.data ;

    }
    
void OneTaskInverseKinematics::Error(geometry_msgs::PoseStamped& p_curr, geometry_msgs::PoseStamped& p_des, std::vector<double> &error)
{
	
	KDL::Vector v_temp_;
	Eigen::Matrix<double,3,3> skew_;
	KDL::Vector rot_err;
	
	quaternion_ q_des;
	quaternion_ q_curr;

	q_des.v(0) = p_des.pose.orientation.x;
        q_des.v(1) = p_des.pose.orientation.y;
	q_des.v(2) = p_des.pose.orientation.z;
	q_des.a = p_des.pose.orientation.w;
	
	q_curr.v(0) = p_curr.pose.orientation.x;
        q_curr.v(1) = p_curr.pose.orientation.y;
	q_curr.v(2) = p_curr.pose.orientation.z;
	q_curr.a = p_curr.pose.orientation.w;

	
	// end-effector position error
        skew_symmetric(q_des.v, skew_); // Gets skew-matrix from the quaternion of desired frame

        for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;                               //Initializiation of the i_element of vector v_temp
                for (int k = 0; k < skew_.cols(); k++)      
                    v_temp_(i) += skew_(i,k)*(q_curr.v(k)); //Sobstitution of the the i_element of vector v_temp with the product between skew_matrix and quaternion
            }

        // end-effector orientation error
        rot_err = q_curr.a*q_des.v - q_des.a*q_curr.v - v_temp_;
	
	error.at(0) = p_des.pose.position.x - p_curr.pose.position.x;
	error.at(1) = p_des.pose.position.y - p_curr.pose.position.y;
	error.at(2) = p_des.pose.position.z - p_curr.pose.position.z;
	
	error.at(3) = rot_err(0);
	error.at(4) = rot_err(1);
	error.at(5) = rot_err(2);

}

} //  Namespace BRACKET

PLUGINLIB_EXPORT_CLASS(bartender_control::OneTaskInverseKinematics, controller_interface::ControllerBase)