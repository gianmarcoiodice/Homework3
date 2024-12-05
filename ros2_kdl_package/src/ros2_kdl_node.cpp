// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort")) //cmd_interface torque da aggiungere
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            robot_temp=std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);  
            robot_temp->setJntLimits(q_min,q_max);       
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_temp.resize(nj); 
            joint_velocities_temp.resize(nj);
            joint_acc_.resize(nj);
            joint_torques_cmd.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){ //qui richiamo una funzione chiamata in 250
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            robot_temp->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            robot_temp->addEE(f_T_ee);
            robot_temp->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));


            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            std::cout << "The initial EE pose is: " << std::endl;  
            std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position 
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            end_position << init_position[0], -init_position[1], init_position[2];
    
            // TRAJECTORY PLANNING
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
            double radius = 0.05;
            
            // choose a type of trajectory: 
            // k = 1 -> circular with cubic polynomial
            // k = 2 -> circular with trapezoidal velocity profile
            // k = 3 -> linear with cubic polynomial
            // k = 4 -> linear with trapezoidal velocity profile
            k = 1;
            if ( k == 1 || k == 2 ) planner_ = KDLPlanner(traj_duration, init_position, radius); 
            if ( k == 3 || k == 4 )  planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); 
            if (k>4||k<0) {            
             	k=1;
            	std::cout<<"Invalid choice of k, automatically set to 1 "<<std::endl;
            }
            // Retrieve the first trajectory point
            trajectory_point p = planner_.compute_trajectory(t,k);

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else
                if (cmd_interface_ == "velocity")
                    { 
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                        }
                    }

                else
                {
                    // Create cmd publisher
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                    // Send joint torque commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_torques_cmd(i);
                    }
                }


            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher()
        {

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; // 
            int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            KDLController controller_(*robot_);

            if (t_ < total_time){

                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_,k); 

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; 
                desFrame.M = cartpos.M; 
                desFrame.p = toKDL(p.pos);

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){ 
                    // Next Frame
                    KDL::Frame nextFrame; 
                    nextFrame.M = cartpos.M; 
                    nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt;  

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else
                    if (cmd_interface_ == "velocity")
                    { //in questo caso il comando è in velcoità

                        // Compute differential IK
                        Vector6d cartvel; 
                        cartvel << p.vel + 5*error, o_error; 
                        joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt; 

                    }
                    else if (cmd_interface_ == "effort")
                    {   //in questo caso il comando di coppia
               
			//POSITION, VELOCITY, AND DESIRED ACCELERATION IN THE OPERATING SPACE, WITH DIFFERENT TYPES
		        Eigen::Vector3d pos_eigen = p.pos;
	 
		        KDL::Frame nextFrame;
		        nextFrame.M = cartpos.M;
		        KDL::Vector pos_kdl(pos_eigen(0), pos_eigen(1), pos_eigen(2));
		        nextFrame.p = pos_kdl;
	 
		        KDL::Twist twist_vel_des = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
		        KDL::Twist twist_acc_des = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());
	 
		        o_error<< 0.0, 0.0, 0.0;
	 
		        Vector6d cartvel, cartacc;
		        cartvel << p.vel, o_error;
		        cartacc << p.acc, o_error;
		        Eigen::VectorXd J_dot_q_dot;
	 
			//  CALCULATING DESIRED JOINT ACCELERATION using a dummy robot.
		        joint_velocities_temp.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
		        joint_positions_temp.data = joint_positions_.data + joint_velocities_temp.data*dt;
		        robot_temp->update(toStdVector(joint_positions_temp.data),toStdVector(joint_velocities_temp.data));
		        Vector6d err;
		        err << error,o_error;
			//  update robot
			robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
	 
			//calculation of error in velocity in the operational space.
			Eigen::Matrix<double,6,1> err_dot;
	 
			Eigen::Vector3d p_dot_d(twist_vel_des.vel.data);
			Eigen::Vector3d p_dot_e(robot_->getEEVelocity().vel.data);
			Eigen::Matrix<double,3,1> e_dot_p = computeLinearError(p_dot_d,p_dot_e);
	 
			Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(nextFrame.M.data );
			Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
			 R_d = matrixOrthonormalization(R_d);
			 R_e = matrixOrthonormalization(R_e);
			Eigen::Vector3d w_d(twist_vel_des.rot.data);
			Eigen::Vector3d w_e(robot_->getEEVelocity().rot.data);
			Eigen::Matrix<double,3,1> e_dot_o = computeOrientationVelocityError(w_d, w_e, R_d, R_e);
	 
			err_dot << e_dot_p, e_dot_o;
			
			// J_dot*q_dot calculation
                        J_dot_q_dot=robot_temp->getEEJacDotqDot();

			//We calculate the desired acceleration with the second-order CLIK
                        joint_acc_.data = pseudoinverse(robot_temp->getEEJacobian().data)*(cartacc - J_dot_q_dot + 40*err+2*err_dot);
                      
			//Inverse dynamics in the joint space
                       // joint_torques_cmd.data = controller_.idCntr(joint_positions_temp, joint_velocities_temp, joint_acc_, 100, 2*sqrt(100));
 
			//Inverse dynamics in the operational space
                        joint_torques_cmd.data =  controller_.idCntr(nextFrame,twist_vel_des,twist_acc_des,10,10, 3, 3);
                        
 
                        std::cout <<"torque                      ";
                        for (int k=0; k<7;k++)
                        std::cout << joint_torques_cmd(k)<<"  ";
 
                        std::cout <<std::endl<<"joint_positions_desiderato  ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_positions_temp(k)<<"  ";
 
                        std::cout <<std::endl<<"joint_positions_            ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_positions_(k)<<"  ";
 
                        std::cout <<std::endl<<"joint_velocities_desiderato ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_velocities_temp(k)<<"  ";
 
                        std::cout <<std::endl<<"joint_velocities_           ";
                        for (int k=0; k<7;k++)
                        std::cout<< joint_velocities_(k)<<"  ";
 
                        std::cout <<std::endl<<"x_reale                     ";
                        for (int k=0; k<3;k++)
                        std::cout << cartpos.p(k)<<"  ";
 
                        std::cout <<std::endl<<"x_desiderato                ";
                        for (int k=0; k<3;k++)
                        std::cout <<  p.pos(k)<<"  ";
                    
                        std::cout <<std::endl<<std::endl;

                    }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else
                    if (cmd_interface_ == "velocity")
                    {
                        // Send joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_velocities_(i);
                        }
                    }
                    else
                    {
                        // Send joint torque commands
                        for (long int i = 0; i < joint_torques_cmd.data.size(); ++i) {
                            desired_commands_[i] = joint_torques_cmd(i);
                         }
                    }


                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }


            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands
                    if (cmd_interface_=="position")
                    {
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_positions_(i); //the last value of the sensor (which corresponds to the end position that we want) 
											//will be used to keep the robot still
                        }
                    }
                    if (cmd_interface_=="velocity")
                    {
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = 0.0;
                        }
                    }
                    if (cmd_interface_=="effort")
                    {

                        Vector6d cartvelf, cartaccf;
                        cartvelf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                        cartaccf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                        joint_velocities_temp.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                        
                        KDL::Frame pos_attuale= robot_->getEEFrame();
                        KDL::Frame lastFrame; 
                        lastFrame.p = pos_attuale.p; 
                        lastFrame.M = pos_attuale.M;
                        robot_->getInverseKinematics(lastFrame, joint_positions_temp);
                        robot_temp->update(toStdVector(joint_positions_temp.data),toStdVector(joint_velocities_temp.data));
                        Vector6d erro;
                        Eigen::Vector3d errore_finale;
                        errore_finale = computeLinearError(toEigen(lastFrame.p), Eigen::Vector3d(pos_attuale.p.data));
                        erro << errore_finale, 0.0, 0.0, 0.0;
                        joint_acc_.data = pseudoinverse(robot_->getEEJacobian().data)*(cartaccf - robot_->getEEJacDotqDot() + 40*erro);

                       
                        Eigen::VectorXd ddqd = joint_acc_.data;
                        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                        
                      //  joint_torques_cmd.data=controller_.idCntr(joint_positions_temp, joint_velocities_temp, joint_acc_, 100, 20);
                       
                   
                        for (long int i = 0; i < joint_torques_cmd.data.size(); i++) {
                            desired_commands_[i] = joint_torques_cmd(i);
                         }

                    }
                
                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_temp;
        KDL::JntArray joint_velocities_temp;
        
        KDL::JntArray joint_acc_;
        KDL::JntArray joint_torques_cmd;
        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLRobot> robot_temp;
        std::shared_ptr<KDLController> controller_;
        KDLPlanner planner_;
        Eigen::Vector3d end_position; 

        int iteration_;
        bool joint_state_available_;
        double t_;
        unsigned int k;
        std::string cmd_interface_;
        KDL::Frame init_cart_pose_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
