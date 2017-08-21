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
	
	f = boost::bind(&OneTaskInverseKinematics::config_callback, this, _1, _2);   
	server.setCallback(f);
	 
	 
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
        // initialization x_des_
        x_des_.p = KDL::Vector(-1, 0, 1);
        x_des_.M = KDL::Rotation::Quaternion(0, 0 , 0, -1);


        cmd_flag_ = 0;

        //Definition of publishers and subscribes

        pub_check_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 250);
        pub_check_initial = nh_.advertise<geometry_msgs::Pose>("initial_position", 250);
	pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("position", 250);

        sub_bartender_cmd = nh_.subscribe("command", 250, &OneTaskInverseKinematics::command, this);

	x_error.resize(6);
	
        return true;
    }
    
    
  void OneTaskInverseKinematics::config_callback(bartender_manager::managerConfig& config, uint32_t level)
  {
      // controller proportional constants
      ROS_INFO("Reconfigure Request");
      
      second_task = config.second_task;
      alpha1 = config.alpha_1;
      alpha2 = config.alpha_2;
    
      ROS_INFO("ALPHA 1 = %f", alpha1);
      ROS_INFO("ALPHA 2 = %f", alpha2);
      ROS_INFO("Second task = %s", bartender_control::OneTaskInverseKinematics::BoolToString(second_task));
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
    
    bool OneTaskInverseKinematics::getCurrentPosition()
{

	/*try
	{
	    listener.waitForTransform( goal_ref, "EE", ros::Time::now(), ros::Duration(0.25));
	    listener.lookupTransform( goal_ref, "EE", ros::Time(0), Goal_T_Ee); 
	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());               
	    return false;
	}
	
	x_pose.pose.position.x = Goal_T_Ee.getOrigin().getX(); 
	x_pose.pose.position.y = Goal_T_Ee.getOrigin().getY(); 
	x_pose.pose.position.z = Goal_T_Ee.getOrigin().getZ();
	tf::quaternionTFToMsg(Goal_T_Ee.getRotation(), x_pose.pose.orientation);
	
	x_pose.header.stamp = ros::Time::now();
	
	ROS_INFO("distance x: %f | y: %f | z: %f", x_pose.pose.position.x, x_pose.pose.position.y, x_pose.pose.position.z);
	
	*/
	
	return true;
}

    void OneTaskInverseKinematics::command(const bartender_control::bartender_msg::ConstPtr &msg)
    {

        x_des_.p = KDL::Vector(msg->des_frame.position.x, msg->des_frame.position.y, msg->des_frame.position.z);
        x_des_.M = KDL::Rotation::EulerZYZ(msg->des_frame.orientation.x, msg->des_frame.orientation.y, msg->des_frame.orientation.z);
        
	bartender_control::OneTaskInverseKinematics::FrameToPose(x_des_,x_des_pose);
	
        if (!msg->arrived) cmd_flag_ = 1;
        if (msg->arrived) cmd_flag_ = 0;
	
	goal_ref = msg->goal_tf;
	
	ns_param = msg->arm;
	
	std::string arm_EE = ns_param + "/" + controller + "/EE";
	
	//cout << arm_EE << endl;
	try
	{
	    /*istener.waitForTransform( goal_ref, arm_EE, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( goal_ref, arm_EE, ros::Time(0), Goal_T_Ee);*/
	    
	    listener.waitForTransform( arm_EE, goal_ref, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( arm_EE, goal_ref, ros::Time(0), Goal_T_Ee);
	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());               
	    return;
	}
	
	try
	{
	    listener.waitForTransform( "world_link", arm_EE, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( "world_link", arm_EE, ros::Time(0), W_T_Ee);
	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());               
	    return;
	}
	
	try
	{
	    listener.waitForTransform( "world_link", goal_ref, ros::Time::now(), ros::Duration(1.0));
	    listener.lookupTransform( "world_link", goal_ref, ros::Time(0), W_T_Goal);
	}
	catch (tf::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());               
	    return;
	}
	

	x_pose.pose.position.x = W_T_Ee.getOrigin().getX(); 
	x_pose.pose.position.y = W_T_Ee.getOrigin().getY(); 
	x_pose.pose.position.z = W_T_Ee.getOrigin().getZ();
	tf::quaternionTFToMsg(W_T_Ee.getRotation(), x_pose.pose.orientation);	
	x_pose.header.stamp = ros::Time::now();
	
	x_des_pose.pose.position.x = W_T_Goal.getOrigin().getX();
	x_des_pose.pose.position.y = W_T_Goal.getOrigin().getY();
	x_des_pose.pose.position.z = W_T_Goal.getOrigin().getZ();
	tf::quaternionTFToMsg(W_T_Goal.getRotation(), x_des_pose.pose.orientation);	
	x_des_pose.header.stamp = ros::Time::now();

	// tf::StampedTransform Goal_T_Ee;
	
	x_diff.pose.position.x = Goal_T_Ee.getOrigin().getX(); 
	x_diff.pose.position.y = Goal_T_Ee.getOrigin().getY(); 
	x_diff.pose.position.z = Goal_T_Ee.getOrigin().getZ();
	tf::quaternionTFToMsg(Goal_T_Ee.getRotation(), x_diff.pose.orientation);	
	x_diff.header.stamp = ros::Time::now();
	
	cout << arm_EE << endl;
	ROS_INFO("EE distance x: %f | y: %f | z: %f", x_pose.pose.position.x, x_pose.pose.position.y, x_pose.pose.position.z);
	ROS_INFO("BOTTLE distance x: %f | y: %f | z: %f", x_des_pose.pose.position.x, x_des_pose.pose.position.y, x_des_pose.pose.position.z);
	
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
	
	//bartender_control::OneTaskInverseKinematics::FrameToPose(x_,x_pose);

        /*lin_err.data[0] = x_des_pose.pose.position.x - x_pose.pose.position.x;
	lin_err.data[1] = x_des_pose.pose.position.y - x_pose.pose.position.y;
	lin_err.data[2] = x_des_pose.pose.position.z - x_pose.pose.position.z;*/
	
	lin_err.data[0] = x_diff.pose.position.x;
	lin_err.data[1] = x_diff.pose.position.y;
	lin_err.data[2] = x_diff.pose.position.z;
	
	quat_des_.v(0) = x_des_pose.pose.orientation.x;
        quat_des_.v(1) = x_des_pose.pose.orientation.y;
	quat_des_.v(2) = x_des_pose.pose.orientation.z;
	quat_des_.a = x_des_pose.pose.orientation.w;
	
	quat_curr_.v(0) = x_pose.pose.orientation.x;
        quat_curr_.v(1) = x_pose.pose.orientation.y;
	quat_curr_.v(2) = x_pose.pose.orientation.z;
	quat_curr_.a = x_pose.pose.orientation.w;
	
	
	// end-effector position error
        skew_symmetric(quat_des_.v, skew_); // Gets skew-matrix from the quaternion of desired frame

        for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;                               //Initializiation of the i_element of vector v_temp
                for (int k = 0; k < skew_.cols(); k++)      
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k)); //Sobstitution of the the i_element of vector v_temp with the product between skew_matrix and quaternion
            }

        // end-effector orientation error
        rot_err = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;
	
	// linear error
	x_error.at(0) = lin_err.data[0];
	x_error.at(1) = lin_err.data[1];
	x_error.at(2) = lin_err.data[2];
	// rotating error
	x_error.at(3) = rot_err.data[0];
	x_error.at(4) = rot_err.data[1];
	x_error.at(5) = rot_err.data[2];
	
	// DEBUG
	/*cout << ns_param << endl;
	ROS_INFO("linear error x: %f | y: %f | z: %f", x_error.at(0), x_error.at(1), x_error.at(2));
        ROS_INFO("rotation error x: %f | y: %f | z: %f", x_error.at(3), x_error.at(4), x_error.at(5));*/
	
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
            
	    //x_initial.M.GetEulerZYZ(Roll_x_init, Pitch_x_init, Yaw_x_init);
	    
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

} //  Namespace BRACKET

PLUGINLIB_EXPORT_CLASS(bartender_control::OneTaskInverseKinematics, controller_interface::ControllerBase)