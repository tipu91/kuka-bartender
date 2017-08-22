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
	  pnh.param<std::string>("ns_arm", ns_param, "unknown");
	  pnh.param<std::string>("class", controller,"bartender_control");
	  
	  //Definition of publishers and subscribes

	  pub_check_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 250);
	  pub_check_initial = nh_.advertise<geometry_msgs::Pose>("initial_position", 250);
	  pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("position", 250);

	  sub_bartender_cmd = nh_.subscribe("command", 250, &OneTaskInverseKinematics::command, this);
	  sub_bartender_config = nh_.subscribe("config",250, &OneTaskInverseKinematics::configCallback, this);
	  
	  /*f = boost::bind(&OneTaskInverseKinematics::config_callback, this, _1, _2);   
	  server.setCallback(f);*/
	  
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

	x_error.resize(6);
	
	//**************************Default values**************************//
	alpha1 = 8;
	alpha2 = 0.4;
	second_task = true;
	cmd_flag_ = 0;
	
        return true;
    }
    
    static void toEulerianAngle(geometry_msgs::PoseStamped& q, double& roll, double& pitch, double& yaw)
    {
	    double ysqr = q.pose.orientation.y * q.pose.orientation.y;

	    // roll (x-axis rotation)
	    double t0 = +2.0 * (q.pose.orientation.w * q.pose.orientation.x + q.pose.orientation.y * q.pose.orientation.z);
	    double t1 = +1.0 - 2.0 * (q.pose.orientation.x * q.pose.orientation.x + ysqr);
	    roll = std::atan2(t0, t1);

	    // pitch (y-axis rotation)
	    double t2 = +2.0 * (q.pose.orientation.w * q.pose.orientation.y - q.pose.orientation.z * q.pose.orientation.x);
	    t2 = ((t2 > 1.0) ? 1.0 : t2);
	    t2 = ((t2 < -1.0) ? -1.0 : t2);
	    pitch = std::asin(t2);

	    // yaw (z-axis rotation)
	    double t3 = +2.0 * (q.pose.orientation.w * q.pose.orientation.z + q.pose.orientation.x * q.pose.orientation.y);
	    double t4 = +1.0 - 2.0 * (ysqr + q.pose.orientation.z * q.pose.orientation.z);  
	    yaw = std::atan2(t3, t4);
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
    void OneTaskInverseKinematics::quaternionProduct(bartender_control::OneTaskInverseKinematics::quaternion_ q1, bartender_control::OneTaskInverseKinematics::quaternion_ q2, bartender_control::OneTaskInverseKinematics::quaternion_ &q)
    {      
      q.a = (q1.a * q2.a) - (q1.v(0)*q2.v(0)) - (q1.v(1)*q2.v(1)) - (q1.v(2)*q2.v(2));
      q.v(0) = (q1.a * q2.v(0)) + (q1.v(0)*q2.a) + (q1.v(1)*q2.v(2)) - (q1.v(2)*q2.v(1));
      q.v(1) = (q1.a * q2.v(1)) + (q1.v(1)*q2.a) + (q1.v(2)*q2.v(0)) - (q1.v(0)*q2.v(2));
      q.v(2) = (q1.a * q2.v(2)) + (q1.v(2)*q2.a) + (q1.v(0)*q2.v(1)) - (q1.v(1)*q2.v(0));
      
      return;
    }
    
    /*bool OneTaskInverseKinematics::TFCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
    {
      return true;
    }*/

    void OneTaskInverseKinematics::command(const bartender_control::bartender_msg::ConstPtr &msg)
    {
	
	//************************** reading message from manager **************************
	x_des_pose.pose = msg->des_frame;
	
        if (!msg->arrived) cmd_flag_ = 1;
        if (msg->arrived) cmd_flag_ = 0;
	
	goal_ref = msg->goal_tf;
	
	ns_param = msg->arm;	
	//**********************************************************************************
	
	std::string arm_EE = ns_param + "/" + controller + "/EE";

	try
	{
	    listener.waitForTransform( goal_ref, arm_EE, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( goal_ref, arm_EE, ros::Time(0), Goal_T_Ee);   
	    
	    /*listener.waitForTransform( arm_EE, goal_ref, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( arm_EE, goal_ref, ros::Time(0), Goal_T_Ee);*/
	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());               
	    return;
	}
	
	try
	{
	    listener.waitForTransform( "bartender_anchor", arm_EE, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( "bartender_anchor", arm_EE, ros::Time(0), W_T_Ee);
	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());               
	    return;
	}
	
	try
	{
	    listener.waitForTransform( "bartender_anchor", goal_ref, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( "bartender_anchor", goal_ref, ros::Time(0), W_T_Goal);
	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());               
	    return;
	}
	

	/*x_pose.pose.position.x = W_T_Ee.getOrigin().getX(); 
	x_pose.pose.position.y = W_T_Ee.getOrigin().getY(); 
	x_pose.pose.position.z = W_T_Ee.getOrigin().getZ();
	tf::quaternionTFToMsg(W_T_Ee.getRotation(), x_pose.pose.orientation);	
	x_pose.header.stamp = ros::Time::now();*/
	
	
	
	/*x_des_pose.pose.position.x = W_T_Goal.getOrigin().getX();
	x_des_pose.pose.position.y = W_T_Goal.getOrigin().getY();
	x_des_pose.pose.position.z = W_T_Goal.getOrigin().getZ();
	tf::quaternionTFToMsg(W_T_Goal.getRotation(), x_des_pose.pose.orientation);	
	x_des_pose.header.stamp = ros::Time::now();*/

	// tf::StampedTransform Goal_T_Ee;
	
	/*x_diff.pose.position.x = Goal_T_Ee.getOrigin().getX(); 
	x_diff.pose.position.y = Goal_T_Ee.getOrigin().getY(); 
	x_diff.pose.position.z = Goal_T_Ee.getOrigin().getZ();
	tf::quaternionTFToMsg(Goal_T_Ee.getRotation(), x_diff.pose.orientation);	
	x_diff.header.stamp = ros::Time::now();*/
	
	//cout << arm_EE << endl;
	
	/*ROS_INFO("EE distance x: %f | y: %f | z: %f", x_pose.pose.position.x, x_pose.pose.position.y, x_pose.pose.position.z);	
	ROS_INFO("BOTTLE distance x: %f | y: %f | z: %f", x_des_pose.pose.position.x, x_des_pose.pose.position.y, x_des_pose.pose.position.z);*/
	
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
	
	std_msgs::Float64MultiArray msg_error;
	for(int i = 0; i < x_error.size(); i++) msg_error.data.push_back( x_error.at(i) );
	pub_check_error.publish(msg_error);

    }
    
    //  Controller function:: Multy Task Inverse Kinematics
    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {
        if (!second_task) alpha1 = 3;

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
                q_null = alpha2 * P_null * potentialEnergy( joint_msr_states_.q );

                for (int i = 0; i < J_pinv_.rows(); i++)
                {

                    joint_des_states_.qdot(i) += alpha2 * q_null[i];

                }

            }

            //*******************************************************************************************************************

            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }
	    
	    geometry_msgs::PoseStamped msg_pose;
	    msg_pose = x_pose;
	    pub_pose.publish(msg_pose);
            
        }
        else
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