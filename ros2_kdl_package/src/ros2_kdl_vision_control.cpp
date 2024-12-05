#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

using namespace KDL;
using namespace std::chrono_literals;
using FloatArray = std_msgs::msg::Float64MultiArray;

class VisionControlNode : public rclcpp::Node
{
public:
     VisionControlNode() 
        : Node("ros2_kdl_vision_control"), 
          node_handle_(std::shared_ptr<VisionControlNode>(this))
  
    {
        // Declare parameters
        declare_parameter("cmd_interface", "velocity"); // defaults to "velocity"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "velocity" || cmd_interface_ == "effort")) 
        {
            RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
        }
        declare_parameter("task", "trajectory_lap");
        get_parameter("task", task_);


        RCLCPP_INFO(get_logger(),"Current Task is: '%s'", task_.c_str());
        if (!(task_ == "positioning" || task_ == "look-at-point" || task_ == "trajectory_lap" ))
            {
                RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
            }
        
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
        KDL::Tree robot_tree;        
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            std::cout << "Failed to retrieve robot_description param!" << std::endl;
            return;
        }

        // Create KDL robot model
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
        joint_positions_temp.resize(nj);
        joint_positions_temp_complete.resize(nj);
        joint_torques_.resize(nj); 
        joint_acc_.resize(nj);
        joint_velocities_.resize(nj);
        joint_velocities_cmd.resize(nj);
        joint_velocities_temp.resize(nj);
        joint_velocities_sum.resize(nj);
        joint_velocities_old.resize(nj);
        qinit.resize(nj);   
        torque.resize(nj);
        torque_new.resize(nj);      

        // Create subscribers
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&VisionControlNode::jointStateCallback, this, std::placeholders::_1));
        while(!joint_state_available_){ 
        RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
        rclcpp::spin_some(node_handle_);  }
        marker_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&VisionControlNode::markerPoseCallback, this, std::placeholders::_1));
        // Wait for the joint_state topic
        while(!marker_state_available_){ 
            RCLCPP_INFO(this->get_logger(), "No aruco data received yet! ...");
            rclcpp::spin_some(node_handle_);    }
        // Update KDLrobot object
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_temp->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        robot_temp->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDLController controller_(*robot_);
        // Compute EE frame
        KDL::Frame init_cart_pose_ = robot_->getEEFrame();
        std::cout << "The initial EE pose is: "<< init_cart_pose_.p <<std::endl;
        
        // Compute IK  
        robot_->getInverseKinematics(init_cart_pose_, qinit);      
        // oggetto rispetto alla telecamera
        cam_T_object = KDL::Frame(aruco_frame_.M, KDL::Vector(aruco_frame_.p.data[0], aruco_frame_.p.data[1], aruco_frame_.p.data[2]));  
        KDL::Frame tool0_T_cam = KDL::Frame(
        KDL::Rotation::RPY(0.0, -1.57, 3.14),  // RPY specified in URDF
        KDL::Vector(0.0, 0.0, 0.0)             // No translation
        );
        KDL::Frame tool0_T_link = KDL::Frame(
        KDL::Rotation::RPY(0.0, 0.0, 0.154),  // RPY specified in URDF
        KDL::Vector(0.0, 0.0, 0.0)             // No translation
        );
        // oggetto rispetto al base frame
        KDL::Frame base_T_objec = robot_->getEEFrame()*tool0_T_cam*cam_T_object;
        //ci creiamo un frame offset che rappresenta la posizione finale desiderata dell'EE
        KDL::Frame frame_offset = cam_T_object;
        base_T_offset.p=base_T_objec.p- KDL::Vector(0.0,0.0,0.2); 
        //base_T_offset.M=cam_T_object.M;
        base_T_offset.M = robot_->getEEFrame().M;

        // EE's trajectory initial position 
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));

        // EE's trajectory end position 
        aruco_position = toEigen(base_T_objec.p);
        std::cout << "aruco_pos:"<<aruco_position[0]<<" "<< aruco_position[1] <<" "<< aruco_position[2]<<std::endl;

        if (cmd_interface_=="velocity")  {
            end_position = toEigen(base_T_offset.p);
            std::cout << "end_pos:"<<end_position[0]<<" "<< end_position[1] <<" "<< end_position[2]<<std::endl;
        }  
        if (cmd_interface_=="effort") {
            end_position << 1.63, -0.31, 0.50;
        }   
        
        // Plan trajectory
        double acc_duration=0.5;
        double t = 0.0;
        double traj_duration = 1.5;

        planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
        std::cout<< "The trajectory chosen is: Linear Trajectory with  Polynomials \n";
        // Retrieve the first trajectory point
        trajectory_point p = planner_.compute_trajectory(t,k);
                
        // Create cmd publisher
        if (cmd_interface_ == "velocity")  { 
        // Create cmd publisher
        cmd_publisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                    std::bind(&VisionControlNode::controlLoop, this)); }
        else if (cmd_interface_ == "effort") {
        // Create cmd publisher
        cmd_publisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                    std::bind(&VisionControlNode::controlLoop, this));    }

        // Send joint commands
        publishCmd();
        RCLCPP_INFO(this->get_logger(), "Starting ...");
}

private:
    void jointStateCallback(const sensor_msgs::msg::JointState& sensor_msg){
        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    void markerPoseCallback(const geometry_msgs::msg::PoseStamped& pose_msg)  {
        marker_state_available_ = true;
        aruco_x = pose_msg.pose.position.x,
        aruco_y = pose_msg.pose.position.y,
        aruco_z = pose_msg.pose.position.z;
            
        aruco_q1 = pose_msg.pose.orientation.x,
        aruco_q2 = pose_msg.pose.orientation.y,
        aruco_q3 = pose_msg.pose.orientation.z,
        aruco_q4 = pose_msg.pose.orientation.w;

        KDL::Rotation rot_= KDL::Rotation::Quaternion(aruco_q1,aruco_q2,aruco_q3,aruco_q4);
        KDL::Vector trasl_(aruco_x,aruco_y,aruco_z);
        
        aruco_frame_.p = trasl_;
        aruco_frame_.M = rot_; 
    }

    void controlLoop()
    {
        if (task_ == "positioning") {
            // Perform camera alignment with marker position and orientation offsets
            alignCameraToMarker();
        } 
        else 
            if (task_ == "look-at-point") {
            // Perform the look-at-point control
            lookAtPointControl();
            }
            else
                trajectory_look_at_point();
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        robot_temp->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
    }

    void alignCameraToMarker()
    {           
        // Compute EE frame
        KDL::Frame current_ee_frame = robot_->getEEFrame();
        KDL::Twist current_ee_vel = robot_->getEEVelocity();
       
        iteration_ = iteration_ + 1;
        double total_time;
        int trajectory_len;
        total_time = 1.5;
        trajectory_len = 150;
        int loop_rate = trajectory_len / total_time;
        dt = 1.0 / loop_rate;
        t_+=dt;
        if (t_ < total_time){

            // Retrieve the trajectory point
            trajectory_point p = planner_.compute_trajectory(t_,k);

            // Compute desired Frame
            KDL::Frame desFrame;
            desFrame.M = base_T_offset.M;           
            desFrame.p = toKDL(p.pos);
            std::cout << "posizione attuale : " << current_ee_frame.p << std::endl; 
            std::cout << "posizione desiderata: " << p.pos << std::endl; 

            //Desired Velocity
            KDL::Twist des_vel = KDL::Twist::Zero();
            des_vel.rot = current_ee_vel.rot;
            des_vel.vel = toKDL(p.vel);            

            // Compute Errors
            Eigen::Vector3d error = computeLinearError(Eigen::Vector3d(desFrame.p.data), Eigen::Vector3d(current_ee_frame.p.data));
            std::cout << "The error norm is : " << error.norm() << std::endl; 

            Eigen::Vector3d o_error = computeOrientationError(toEigen(desFrame.M), toEigen(current_ee_frame.M)); 

            //VELOCITY INTERFACE
            // Compute differential IK
                Vector6d cartvel; cartvel << 1*p.vel + 5*error, 10*o_error;
                joint_velocities_cmd.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                joint_positions_.data = joint_positions_.data + joint_velocities_cmd.data*dt;      }
            else 
               for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                   joint_velocities_cmd(i) = 0.0;  
            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            publishCmd();          
    }

    void lookAtPointControl()
    {
        // Control law for look-at-point task
        cam_T_object = KDL::Frame(aruco_frame_.M, KDL::Vector(aruco_frame_.p.data[0], aruco_frame_.p.data[1], aruco_frame_.p.data[2]));
        // L Matrix(3x6) computation    
        Eigen::Matrix<double,3,1> c_Po = toEigen(cam_T_object.p);
        Eigen::Matrix<double,3,1> s = c_Po/c_Po.norm();
        //calcolo Rc
        KDL::Frame base_T_tool0 = robot_->getEEFrame(); // Base to tool0
        KDL::Frame tool0_T_cam = KDL::Frame(
        KDL::Rotation::RPY(0.0, -1.57, 3.14),  // RPY specified in URDF
        KDL::Vector(0.0, 0.0, 0.0)             // No translation
        );
        KDL::Frame base_T_cam = base_T_tool0 * tool0_T_cam;
        Eigen::Matrix<double,3,3> R_c = toEigen(base_T_cam.M);
        Eigen::Matrix<double,3,3> L_block = (-1/c_Po.norm())*(Eigen::Matrix<double,3,3>::Identity() - s*s.transpose());
        Eigen::Matrix<double,6,6> R_c_big = Eigen::Matrix<double,6,6>::Zero();
        R_c_big.block(0,0,3,3) = R_c;
        R_c_big.block(3,3,3,3) = R_c;
        Eigen::Matrix<double,3,6> L = Eigen::Matrix<double,3,6>::Zero();
        L.block(0,0,3,3) = L_block;
        L.block(0,3,3,3) = skew(s);
        L = L*(R_c_big.transpose());

        // N matrix(7x7) computation
        KDL::Jacobian J_ = robot_->getEEJacobian();
        Eigen::MatrixXd eigenJ(J_.rows(), J_.columns());
        for (unsigned int i = 0; i < J_.rows(); ++i) {
            for (unsigned int j = 0; j < J_.columns(); ++j) {
                eigenJ(i, j) = J_(i, j);  
            }
        }
        Eigen::MatrixXd LJ = L*eigenJ;
        Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd N = Eigen::Matrix<double,7,7>::Identity() - (LJ_pinv*LJ);
        std::cout <<"s"<< s.transpose() << std::endl;
        Eigen::VectorXd eigen_qinit(qinit.rows());
        for (unsigned int i = 0; i < qinit.rows(); ++i) {
            eigen_qinit(i) = qinit(i);  
        }
        Eigen::VectorXd eigen_joint_positions(joint_positions_.rows());
        for (unsigned int i = 0; i < joint_positions_.rows(); ++i) {
            eigen_joint_positions(i) = joint_positions_(i);  
        }

        joint_velocities_cmd.data = 2*LJ_pinv*Eigen::Vector3d(0,0,1)+1*N*(eigen_qinit - eigen_joint_positions);

        double s_error=(Eigen::Vector3d(0,0,1)-s).norm();
        std::cout << "errore su s: "<< s_error << std::endl;
        if (s_error>0.01)
            joint_velocities_cmd.data = 1*LJ_pinv*Eigen::Vector3d(0,0,1)+0.5*N*(eigen_qinit - eigen_joint_positions); 
        else
        {
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                joint_velocities_cmd.data[i]=0;
                if (s_error<0.01) std::cout <<"target raggiunto" << std::endl;
        }

        if(cmd_interface_== "velocity")
        {
            joint_positions_.data = joint_positions_.data + joint_velocities_cmd.data*dt; 
            joint_acc_.data=(joint_velocities_.data-joint_velocities_old.data)/dt;
        }

        if (cmd_interface_ == "effort" ){
            //TORQUE
            
                KDLController controller_(*robot_);                
                Eigen::Vector3d o_error;

                joint_velocities_temp.data = joint_velocities_cmd.data; //faccio la pseudoinversa per ottenere la cinemtaica inversa
                joint_positions_temp.data = joint_positions_.data + joint_velocities_temp.data*dt; 
                robot_temp->update(toStdVector(joint_positions_temp.data),toStdVector(joint_velocities_temp.data));
                KDL::Twist desired_vel = robot_temp->getEEVelocity();
                KDL::Frame desired_pos = robot_temp->getEEFrame();
                KDL::Frame cartpos_act=robot_->getEEFrame();
                KDL::Twist cartvel_act = robot_->getEEVelocity();
                Eigen::Vector3d error = computeLinearError(toEigen(desired_pos.p), Eigen::Vector3d(cartpos_act.p.data));
                o_error << (0-s[0]), (0-s[1]), (1-s[2]);

                Vector6d cartvel, cartacc;
                cartvel << toEigen(desired_vel.vel) + 5*error, o_error; 
                // Differenza traslazionale
                KDL::Vector translational_difference = desired_vel.vel - cartvel_act.vel;
                cartacc<< 0,0,0,0,0,0;
                Eigen::VectorXd J_dot_q_dot;
                        
                Vector6d err;
          
                err <<  error,o_error; 
                J_dot_q_dot=robot_temp->getEEJacDotqDot();
                joint_acc_.data = pseudoinverse(robot_->getEEJacobian().data)*(cartacc - J_dot_q_dot + 40*err);

                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                Eigen::VectorXd computed_torque=controller_.idCntr(joint_positions_temp, joint_velocities_temp, joint_acc_, 60,  2*sqrt(100));
                torque = toKDLJntArray(computed_torque);

                KDL::Twist desired_acc;
                desired_acc.vel = KDL::Vector(0.0, 0.0, 0.0); // Velocità lineare a zero
                desired_acc.rot = KDL::Vector(0.0, 0.0, 0.0); // Velocità angolare a zero


                Eigen::VectorXd computed_torque_new=controller_.idCntr(desired_pos, desired_vel, desired_acc, 10,10, 3, 3);
                torque_new = toKDLJntArray(computed_torque_new);


                for (int i = 0; i < joint_torques_.data.size(); ++i) 
                        joint_torques_(i) = torque_new(i);          //if we want to use the operational space controller
                

            
        }

        publishCmd();
    }

    void trajectory_look_at_point()
    {

            // Control law for look-at-point task
            cam_T_object = KDL::Frame(aruco_frame_.M, KDL::Vector(aruco_frame_.p.data[0], aruco_frame_.p.data[1], aruco_frame_.p.data[2]));
            // L Matrix(3x6) computation    
            Eigen::Matrix<double,3,1> c_Po = toEigen(cam_T_object.p);
            Eigen::Matrix<double,3,1> s = c_Po/c_Po.norm();
            //calcolo Rc
            KDL::Frame base_T_tool0 = robot_->getEEFrame(); // Base to tool0
            KDL::Frame tool0_T_cam = KDL::Frame(
            KDL::Rotation::RPY(0.0, -1.57, 3.14),  // RPY specified in URDF
            KDL::Vector(0.0, 0.0, 0.0)             // No translation
            );
            KDL::Frame base_T_cam = base_T_tool0 * tool0_T_cam;
            Eigen::Matrix<double,3,3> R_c = toEigen(base_T_cam.M);
            Eigen::Matrix<double,3,3> L_block = (-1/c_Po.norm())*(Eigen::Matrix<double,3,3>::Identity() - s*s.transpose());
            Eigen::Matrix<double,6,6> R_c_big = Eigen::Matrix<double,6,6>::Zero();
            R_c_big.block(0,0,3,3) = R_c;
            R_c_big.block(3,3,3,3) = R_c;
            Eigen::Matrix<double,3,6> L = Eigen::Matrix<double,3,6>::Zero();
            L.block(0,0,3,3) = L_block;
            L.block(0,3,3,3) = skew(s);
            L = L*(R_c_big.transpose());

            // N matrix(7x7) computation
            KDL::Jacobian J_ = robot_->getEEJacobian();
            Eigen::MatrixXd eigenJ(J_.rows(), J_.columns());
            for (unsigned int i = 0; i < J_.rows(); ++i) {
                for (unsigned int j = 0; j < J_.columns(); ++j) {
                    eigenJ(i, j) = J_(i, j);  
                }
            }
            Eigen::MatrixXd LJ = L*eigenJ;
            Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd N = Eigen::Matrix<double,7,7>::Identity() - (LJ_pinv*LJ);
            std::cout <<"s"<< s.transpose() << std::endl;
            Eigen::VectorXd eigen_qinit(qinit.rows());
            for (unsigned int i = 0; i < qinit.rows(); ++i) {
                eigen_qinit(i) = qinit(i);  
            }
            Eigen::VectorXd eigen_joint_positions(joint_positions_.rows());
            for (unsigned int i = 0; i < joint_positions_.rows(); ++i) {
                eigen_joint_positions(i) = joint_positions_(i);  
            }

            double s_error=(Eigen::Vector3d(0,0,1)-s).norm();
            //std::cout << "errore su s: "<< s_error << std::endl;
            if (s_error>0.01)
                joint_velocities_cmd.data = 1*LJ_pinv*Eigen::Vector3d(0,0,1)+0.5*N*(eigen_qinit - eigen_joint_positions);  //funziona con 1 e 0.5
            else
            {
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                    joint_velocities_cmd.data[i]=0;
                if (s_error<0.01) std::cout <<"target raggiunto" << std::endl;
            }
            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; // 
            int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            KDLController controller_(*robot_);
            KDL::Frame act_position= robot_->getEEFrame();
            //std::cout <<"errore rispetto al punto finale: "<<(end_position-toEigen(act_position.p)).norm() << std::endl;

            if (t_ < total_time  )
            {
                trajectory_point p = planner_.compute_trajectory(t_,k); 

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; 
                desFrame.M = cartpos.M; 
                desFrame.p = toKDL(p.pos); 
                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                std::cout << "The error norm is : " << error.norm() << std::endl;
                Eigen::Vector3d o_error;
                o_error << (0-s[0]), (0-s[1]), (1-s[2]);
                Vector6d cartvel, cartacc;
                cartvel << p.vel + 5*error, 0*o_error; 
                cartacc << p.acc, 0*o_error;
                Eigen::VectorXd J_dot_q_dot; 

               joint_velocities_temp.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                
                // Jacobiana del tracking
                Eigen::MatrixXd J_track = robot_->getEEJacobian().data;

                // Pseudoinversa della Jacobiana
                Eigen::MatrixXd J_track_pinv = J_track.completeOrthogonalDecomposition().pseudoInverse();

                // Proiettore nello spazio nullo
                Eigen::MatrixXd N_track = Eigen::MatrixXd::Identity(J_track.cols(), J_track.cols()) - J_track_pinv * J_track;

                KDL::JntArray joint_velocities_sum(joint_velocities_temp.rows());
                if (joint_velocities_sum.rows() != joint_velocities_temp.rows())
                    joint_velocities_sum.resize(joint_velocities_temp.rows());
            
               joint_velocities_sum.data = joint_velocities_temp.data + N_track*(joint_velocities_cmd.data);
       	       joint_positions_temp_complete.data = joint_positions_.data + joint_velocities_sum.data*dt;
               robot_temp->update(toStdVector(joint_positions_temp_complete.data),toStdVector(joint_velocities_sum.data)); 

               KDL::Twist desired_vel = robot_temp->getEEVelocity();
               KDL::Frame desired_pos = robot_temp->getEEFrame();
               KDL::Twist desired_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());

               Vector6d err;
               err << error,o_error;

               J_dot_q_dot=robot_temp->getEEJacDotqDot();
               joint_acc_.data = pseudoinverse(robot_->getEEJacobian().data)*(cartacc - J_dot_q_dot + (60, 60, 60, 15, 15, 15)*err); //clik
               robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
               
		       
              /* Eigen::VectorXd computed_torque=controller_.idCntr(joint_positions_temp_complete, joint_velocities_sum, joint_acc_, 60 , sqrt(100));
                torque = toKDLJntArray(computed_torque);
                for (int i = 0; i < joint_torques_.data.size(); ++i) 
                    joint_torques_(i) = torque(i);*/
                    
               Eigen::VectorXd computed_torque_new=controller_.idCntr(desired_pos, desired_vel, desired_acc, 50, 50, 20, 20);
               torque_new = toKDLJntArray(computed_torque_new);
               for (int i = 0; i < joint_torques_.data.size(); ++i) {
                       joint_torques_(i) = torque_new(i); 			//if we want to use the operational space controller       
                std::cout<< "torque command: "<<joint_torques_(i)<< std::endl;}  
               publishCmd();
            }
            else
            {       
                std::cout <<"TRAIETTORIA ESEGUUITA"<< std::endl;
                Vector6d cartvelf, cartaccf;
                cartvelf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                cartaccf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                Eigen::VectorXd J_dot_q_dot_;

                joint_velocities_temp.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                KDL::Frame pos_attuale= robot_->getEEFrame();
                KDL::Frame lastFrame; 
                lastFrame.p=pos_attuale.p; 
                lastFrame.M = pos_attuale.M;

                robot_->getInverseKinematics(lastFrame, joint_positions_temp);
                robot_temp->update(toStdVector(joint_positions_temp.data),toStdVector(joint_velocities_temp.data));
                
                Vector6d erro;
                Eigen::Vector3d errore_finale;
                errore_finale = computeLinearError(toEigen(lastFrame.p), Eigen::Vector3d(pos_attuale.p.data));
                erro << errore_finale, 0.0, 0.0, 0.0;
                J_dot_q_dot_=robot_->getEEJacDotqDot();
                joint_acc_.data = pseudoinverse(robot_->getEEJacobian().data)*(cartaccf - J_dot_q_dot_ + 40*erro);
                Eigen::VectorXd ddqd = joint_acc_.data;
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                Eigen::VectorXd computed_torque=controller_.idCntr(joint_positions_temp, joint_velocities_temp, joint_acc_, 60, 2*sqrt(100));
                torque = toKDLJntArray(computed_torque);
                for (int i = 0; i < joint_torques_.data.size(); ++i) 
                    joint_torques_(i) = torque(i);
                publishCmd();
            }
        }


    void publishCmd()
    {
        std_msgs::msg::Float64MultiArray cmd_msg;
        if (cmd_interface_ == "velocity"){ 
        cmd_msg.data.resize(joint_velocities_.data.size());
        for (size_t i = 0; i < joint_velocities_.data.size(); ++i) {
            cmd_msg.data[i] = joint_velocities_cmd(i);
         }
        }
        else if (cmd_interface_ == "effort"){
        cmd_msg.data.resize(joint_torques_.data.size());
        for (long int i = 0; i < joint_torques_.data.size(); i++) {
             cmd_msg.data[i] = joint_torques_(i);
          }
        }
        cmd_publisher_->publish(cmd_msg);
    }

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_subscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_handle_;    

    // Robot model and control structures
    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<KDLRobot> robot_temp;
    std::shared_ptr<KDLController> controller_;
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_velocities_sum;
    KDL::JntArray joint_positions_temp;
    KDL::JntArray joint_velocities_temp;
    KDL::JntArray joint_positions_temp_complete;
    KDL::JntArray joint_torques_;
    KDL::JntArray torque_new;
    KDL::JntArray torque;
    KDL::JntArray joint_acc_;
    KDL::JntArray joint_velocities_old;
    KDL::JntArray joint_velocities_cmd;
    KDL::JntArray qinit;
    Eigen::Vector3d end_position, aruco_position;
    KDLPlanner planner_;
    KDL::Frame base_T_offset;
    KDL::Frame cam_T_object;
    KDL::Frame cam_T_object_old = KDL::Frame::Identity();
    double t_, dt;
    unsigned int k = 3;

    // ArUco marker data
    std::vector<double> aruco_pose= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool joint_state_available_ = false;
    bool marker_state_available_ = false;
    KDL::Frame aruco_frame_;
    double aruco_x, aruco_y, aruco_z, aruco_q1, aruco_q2, aruco_q3, aruco_q4;

    // Control parameters
    std::string task_;  
    std::string cmd_interface_;
    int iteration_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 0;
}
