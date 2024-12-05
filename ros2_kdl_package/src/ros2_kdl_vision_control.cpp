#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include "math.h"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "ros2_kdl_package/msg/error.hpp"

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
            /*--- PARAMETERS DECLARATION ---*/

            // declare cmd_interface parameter (position, velocity, effort_j, effort_o)
            declare_parameter("cmd_interface", "");     //defaults to "position"
            // declare vel_prof parameter (trapezoidal, cubical)
            declare_parameter("vel_prof", "");          //defaults to "trapezoidal"
            // declare trajectory parameter (linear, circular)
            declare_parameter("trajectory", "");        //defaults to "linear"
            // declare vision task parameter
            declare_parameter("task", "");              //defaults to positioning

            declare_parameter("Kpp",   0.0);      //gain for proportional action implemented for the inverse dynamic controller in joint space
            declare_parameter("Kdd",   0.0);      //gain for derivative action implemented for the inverse dynamic controller in joint space
            declare_parameter("Kpp_o", 0.0);    //gain for proportional action for position implemented for the inverse dynamic controller in operational space
            declare_parameter("Kpo_o", 0.0);    //gain for proportional action for orientation implemented for the inverse dynamic controller in operational space
            declare_parameter("Kdp_o", 0.0);    //gain for derivative action for position implemented for the inverse dynamic controller in operational space
            declare_parameter("Kdo_o", 0.0);    //gain for derivative action for position implemented for the inverse dynamic controller in operational space

            declare_parameter("x_offset", 0.0);
            declare_parameter("y_offset", 0.0);
            declare_parameter("z_offset", 0.0);
            declare_parameter("roll_offset", 0.0);
            declare_parameter("pitch_offset", 0.0);
            declare_parameter("yaw_offset", 0.0);

            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("vel_prof", vel_prof_);
            get_parameter("trajectory", trajectory_);
            get_parameter("task", task_);

            get_parameter("Kpp", Kpp_);
            get_parameter("Kdd", Kdd_);
            get_parameter("Kpp_o", Kpp_o);
            get_parameter("Kpo_o", Kpo_o);
            get_parameter("Kdp_o", Kdp_o);
            get_parameter("Kdo_o", Kdo_o);

            get_parameter("x_offset", x_offset_);
            get_parameter("y_offset", y_offset_);
            get_parameter("z_offset", z_offset_);
            get_parameter("roll_offset", roll_offset_);
            get_parameter("pitch_offset", pitch_offset_);
            get_parameter("yaw_offset", yaw_offset_);

            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
            RCLCPP_INFO(get_logger(),"Current trajectory is: '%s'", trajectory_.c_str());
            RCLCPP_INFO(get_logger(),"Current velocity profile is: '%s'", vel_prof_.c_str());


            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort_j" || cmd_interface_ == "effort_o"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }
            if (!(vel_prof_ == "trapezoidal" || vel_prof_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected velocity profile is not valid!"); return;
            }
            if (!(trajectory_ == "linear" || trajectory_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected circular is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0.0;
            joint_state_available_ = false;
            aruco_info_ = false;

            /*--- RETRIEVING ROBOT PARAM ---*/
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            /*--- CREATING ROBOT KDL STRUCTURE ---*/
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);

            /*--- INITIALIZING JOINT ARRAYS ---*/
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;
            robot_->setJntLimits(q_min,q_max);
            joint_positions_.resize(nj);
            joint_velocities_.resize(nj);
            joint_efforts_ = Eigen::VectorXd::Zero(nj);

            joint_pos_.resize(nj);
            joint_vel_.resize(nj);
            joint_acc_.resize(nj);

            /*--- SUBSCRIBER TO JOINT STATES ---*/
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            /*--- SUBSCRIBER TO ARUCO STAMPED POSE ---*/
            arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_marker_callback, this, std::placeholders::_1));

            /*--- WAIT FOR JOINT STATES ---*/
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            /*--- WAIT FOR ARUCO POSE ---*/
            while(!aruco_info_){
                RCLCPP_INFO(this->get_logger(), "No aruco data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            /*--- UPDATE ROBOT STRUCTURE ---*/
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            /*--- ADDING THE CAMERA AS AND EFFECTOR ---*/
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            f_T_ee.M = f_T_ee.M*KDL::Rotation::RotZ(M_PI/2);
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            /*--- SAVING INITIAL JOINT POSITIONS ---*/
            initial_joint_pos_ = joint_positions_;
            joint_pos_ = joint_positions_;

            /*--- COMPUTE END EFFECTOR FRAME ---*/
            init_cart_pose_ = robot_->getEEFrame();

            /*--- COMPUTE IK ---*/
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

            /*--- CALCULATING DESIRED FRAME FOR VISION ---*/
            desired_frame_ = desiredFrame();

            /*--- INITIALIZING CONTROLLER OBJECT ---*/
            KDLController controller_(*robot_);

            /*--- INITIAL POINT OF TRAJCETORY ---*/
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));

            /*--- END POINT OF TRAJECTORY ---*/
            Eigen::Vector3d end_position; end_position << init_position[0], -0.5, init_position[2];

            /*--- END POINT OF TRAJECTORY FOR VISUAL TASK (POSITIONING) ---*/
            Eigen::Vector3d end_position_vis; end_position_vis << desired_frame_.p.data[0], desired_frame_.p.data[1], desired_frame_.p.data[2];

            //DEBUG -> RCLCPP_INFO(this->get_logger(), "End position is: %f, %f, %f", desired_frame_.p.data[0], desired_frame_.p.data[1], desired_frame_.p.data[2]);

            /*--- PLAN TRAJECTORY ---*/
            double traj_duration = 3.0, acc_duration = 0.6, radius = 0.2;

            if(trajectory_ == "circular"){
                planner_ = KDLPlanner(traj_duration, init_position, radius, vel_prof_);
            }else if(trajectory_ == "linear"){
                if(task_ == "positioning") planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position_vis, vel_prof_);
                else planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, vel_prof_);
            }


            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }else {
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                // Send joint torques commands
                for (long int i = 0; i < joint_efforts_.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }
            }

            /*--- MESSAGE CREATION AND PUBLISHING ---*/
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            publisher_ = this->create_publisher<ros2_kdl_package::msg::Error>("/error", 10);


            //DEBUG -> RCLCPP_INFO(this->get_logger(), "Robot has: %d", robot_->getNrJnts());

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 3.0;
            int trajectory_len = 300; 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;

            KDLController controller_(*robot_);

            if(t_ < total_time) {
                p = planner_.compute_trajectory(t_);
                //DEBUG -> RCLCPP_INFO(this->get_logger(), "Normal trajectory...");

            }else{
                p = p;  //planner_.compute_trajectory(total_time);
                //DEBUG -> RCLCPP_INFO(this->get_logger(), "Stop trajectory...");
            }

            //DEBUG -> RCLCPP_INFO(this->get_logger(), "The error norm is : %f", error.norm());


            /*--- POSITION ---*/
            if(cmd_interface_ == "position"){ //DEPRECATED
                // Next Frame
                //KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt;

                // Compute IK
                //robot_->getInverseKinematics(nextFrame, joint_positions_);
            }
            /*--- VELOCITY ---*/
            else if(cmd_interface_ == "velocity"){

                if(task_ == "look_at_point"){

                    joint_velocities_ = look_at_point_cl();

                }else if(task_ == "positioning"){

                    joint_velocities_ = positioning();

                }

                joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;

                //DEBUG -> RCLCPP_INFO(this->get_logger(), "The x position of the ee is: %f\n", cartpos.p.data[0]);
                //DEBUG -> RCLCPP_INFO(this->get_logger(), "The y position of the ee is: %f\n", cartpos.p.data[1]);
                //DEBUG -> RCLCPP_INFO(this->get_logger(), "The z position of the ee is: %f\n", cartpos.p.data[2]);

            }
            /*--- EFFORT JOINT SPACE ---*/
            else if(cmd_interface_ == "effort_j"){

                KDL::JntArray prev_joint_velocities; prev_joint_velocities.resize(robot_->getNrJnts());
                prev_joint_velocities.data=joint_velocities_.data;

                if(task_ == "look_at_point"){

                    joint_velocities_ = look_at_point_cl();


                }else if(task_ == "positioning"){

                    joint_velocities_ = positioning();

                }else if(task_ == "merge_j"){

                    Eigen::Vector3d cp0; cp0.setZero();
                    Eigen::Vector3d s; s.setZero();
                    Eigen::Vector3d sd; sd << 0.0, 0.0, 1.0;

                    cp0[0] = aruco_.p.data[0];
                    cp0[1] = aruco_.p.data[1];
                    cp0[2] = aruco_.p.data[2];

                    Eigen::Vector3d direction_vector = cp0.normalized();

                    Eigen::Vector3d z_axis(0, 0, 1);  //deve essere l'asse z del robot
                    Eigen::Vector3d rotation_axis = z_axis.cross(direction_vector);
                    double angle = std::acos(std::clamp(z_axis.dot(direction_vector), -1.0, 1.0));
    
                    // Desired orientation matrix
                    Eigen::Matrix3d desired_orientation = toEigen(robot_->getEEFrame().M)*Eigen::AngleAxisd(angle, rotation_axis.normalized()).toRotationMatrix();

                    Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(robot_->getEEFrame().p.data));
                    Eigen::Vector3d o_error = computeOrientationError(desired_orientation, toEigen(robot_->getEEFrame().M));

                    auto error_msg = ros2_kdl_package::msg::Error();
                    error_msg.position_error_norm = error.norm();
                    error_msg.orientation_error_norm = o_error.norm();

                    publisher_->publish(error_msg);

                    Vector6d des_cartvel; des_cartvel << p.vel + 10*error, 2*o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*des_cartvel;

                }

                joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                joint_accelerations_.data = (joint_velocities_.data-prev_joint_velocities.data)/dt;

                joint_efforts_ = controller_.idCntr(joint_positions_,joint_velocities_,joint_accelerations_,Kpp_, Kdd_);

            }
            /*--- EFFORT OPERATIONAL SPACE ---*/
            else if(cmd_interface_ == "effort_o"){
                
                if(task_ == "positioning"){
                    
                    Eigen::MatrixXd Jac = robot_->getEEJacobian().data;

                    Vector6d dxd_; dxd_ << p.vel, 0.0, 0.0, 0.0; //x_dot
                    Vector6d ddxd_; ddxd_ << p.acc, 0.0, 0.0, 0.0; //x_ddot

                    KDL::Twist desVel; desVel.vel=toKDL(p.vel);
                    KDL::Twist desAcc; desAcc.vel=toKDL(p.acc);
                    KDL::Frame desFrame; desFrame.p = toKDL(p.pos);

                    Vector6d e_, e_dot;
                    computeErrors(desired_frame_, robot_->getEEFrame(), desVel, robot_->getEEVelocity(), e_, e_dot);

                    //Second order IK
                    joint_acc_.data = pseudoinverse(Jac)*(ddxd_ - robot_->getEEJacDot() + Kp*e_ + Kd*e_dot);
                    joint_vel_.data = joint_vel_.data + joint_acc_.data*dt;
                    joint_pos_.data = joint_pos_.data + joint_vel_.data*dt + 0.5*joint_acc_.data*dt*dt;

                    desFrame.M = desired_frame_.M;

                    joint_efforts_ = controller_.idCntr_o(desFrame, desVel, desAcc, Kpp_o, Kpo_o, Kdp_o, Kdo_o);

                }else if(task_ == "look_at_point"){

                    KDL::JntArray prev_joint_velocities; prev_joint_velocities.resize(robot_->getNrJnts());
                    prev_joint_velocities.data=joint_vel_.data;

                    Eigen::Vector3d cp0; cp0.setZero();

                    cp0[0] = aruco_.p.data[0];
                    cp0[1] = aruco_.p.data[1];
                    cp0[2] = aruco_.p.data[2]; 

                    Eigen::Vector3d direction_vector = cp0.normalized();

                    Eigen::Vector3d z_axis(0, 0, 1);  //deve essere l'asse z del robot
                    Eigen::Vector3d rotation_axis = z_axis.cross(direction_vector);
                    double angle = std::acos(std::clamp(z_axis.dot(direction_vector), -1.0, 1.0));
            
                    // Desired orientation matrix
                    //Eigen::Matrix3d desired_orientation = toEigen(robot_->getEEFrame().M)*Eigen::AngleAxisd(angle, rotation_axis.normalized()).toRotationMatrix();
                            
                    // joint_vel_ = look_at_point_cl();
                    // joint_pos_.data = joint_pos_.data + joint_vel_.data*dt;
                    // joint_acc_.data = (joint_vel_.data-prev_joint_velocities.data)/dt;

                    KDL::Frame lap_frame; lap_frame.M = (robot_->getEEFrame()).M*(KDL::Rotation::Rot(toKDL(rotation_axis), angle)); lap_frame.p = robot_->getEEFrame().p;
                    KDL::Twist lap_vel; lap_vel.rot = toKDL(computeOrientationError(toEigen(lap_frame.M), toEigen(robot_->getEEFrame().M)));
                    KDL::Twist lap_acc;

                    joint_efforts_ = controller_.idCntr_o(lap_frame, lap_vel, lap_acc, Kpp_o, Kpo_o, Kdp_o, Kdo_o);
                        
                }else if(task_=="merge_o"){

                    // KDL::JntArray prev_joint_velocities; prev_joint_velocities.resize(robot_->getNrJnts());
                    // prev_joint_velocities.data=joint_vel_.data;

                    Eigen::Vector3d cp0; cp0.setZero();

                    cp0[0] = aruco_.p.data[0];
                    cp0[1] = aruco_.p.data[1];
                    cp0[2] = aruco_.p.data[2]; 

                    Eigen::Vector3d direction_vector = cp0.normalized();

                    Eigen::Vector3d z_axis(0, 0, 1);  //deve essere l'asse z del robot
                    Eigen::Vector3d rotation_axis = z_axis.cross(direction_vector);
                    double angle = std::acos(std::clamp(z_axis.dot(direction_vector), -1.0, 1.0));
                
                    // Desired orientation matrix
                    //Eigen::Matrix3d desired_orientation = toEigen(robot_->getEEFrame().M)*Eigen::AngleAxisd(angle, rotation_axis.normalized()).toRotationMatrix();
                    
                    // joint_vel_ = look_at_point_cl();
                    // joint_pos_.data = joint_pos_.data + joint_vel_.data*dt;
                    // joint_acc_.data = (joint_vel_.data-prev_joint_velocities.data)/dt;

                    KDL::Frame lap_frame; lap_frame.M= (robot_->getEEFrame()).M*(KDL::Rotation::Rot(toKDL(rotation_axis), angle)); lap_frame.p= toKDL(p.pos);
                    KDL::Twist lap_vel; //lap_vel.rot = toKDL(computeOrientationError(desired_orientation, toEigen(cartpos.M))); 
                    lap_vel.vel = toKDL(p.vel);
                    KDL::Twist lap_acc; lap_acc.vel=toKDL(p.acc);

                    Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(robot_->getEEFrame().p.data));
                    Eigen::Vector3d o_error = computeOrientationError(toEigen(lap_frame.M), toEigen(robot_->getEEFrame().M));

                    auto error_msg = ros2_kdl_package::msg::Error();
                    error_msg.position_error_norm = error.norm();
                    error_msg.orientation_error_norm = o_error.norm();

                    publisher_->publish(error_msg);

                    joint_efforts_ = controller_.idCntr_o(lap_frame, lap_vel, lap_acc, Kpp_o, Kpo_o, Kdp_o, Kdo_o);
       
                }

            }

            /*--- ROBOT STRUCTURE UPDATE ---*/
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            /*--- SENDING COMMANDS ---*/
            if(cmd_interface_ == "position"){
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if (cmd_interface_ == "velocity"){
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }else{
                // Send torque velocity commands
                for (long int i = 0; i < joint_efforts_.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }
            }

            /*--- PUBLISHING COMMANDS ---*/
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            //KDL::Frame ee_pose = robot_->getEEFrame();
            //DEBUG -> RCLCPP_INFO(this->get_logger(), "The EE pose is: %f", ee_pose.p.data[2]);

            /*--- TIME UPDATE ---*/
            t_+=dt;
        }


        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }

        }

        void aruco_marker_callback(const geometry_msgs::msg::PoseStamped& aruco_msg){

            aruco_.p.data[0] = aruco_msg.pose.position.x;
            aruco_.p.data[1] = aruco_msg.pose.position.y;
            aruco_.p.data[2] = aruco_msg.pose.position.z;

            KDL::Rotation rot;

            aruco_.M = rot.Quaternion(aruco_msg.pose.orientation.x,
                                  aruco_msg.pose.orientation.y,
                                  aruco_msg.pose.orientation.z,
                                  aruco_msg.pose.orientation.w);

            aruco_info_ = true;

        }

        KDL::JntArray look_at_point_cl(){

            KDL::JntArray q_dot; q_dot.resize(robot_->getNrJnts());

            Eigen::Vector3d cp0; cp0.setZero();
            Eigen::Vector3d s; s.setZero();
            Eigen::Vector3d sd; sd << 0.0, 0.0, 1.0;
            cp0[0] = aruco_.p.data[0];
            cp0[1] = aruco_.p.data[1];
            cp0[2] = aruco_.p.data[2];

            s = cp0/cp0.norm();

            Eigen::Matrix<double, 6, 6> rot;
            rot.block(0,0,3,3) = toEigen(robot_->getEEFrame().M);
            rot.block(3,3,3,3) = toEigen(robot_->getEEFrame().M);

            Eigen::Matrix<double,3,6> L_;

            L_.block(0,0,3,3) = -1/cp0.norm()*(Eigen::MatrixXd::Identity(3,3)-s*s.transpose());
            L_.block(0,3,3,3) = skew(s);
            L_=L_*rot.transpose();

            Eigen::Matrix<double,7,7> N_;
            N_ = Eigen::MatrixXd::Identity(7,7)-pseudoinverse(L_*robot_->getEEJacobian().data)*L_*robot_->getEEJacobian().data;

            Eigen::Vector<double,7> q0_dot;
            q0_dot = initial_joint_pos_.data - joint_positions_.data;

            q0_dot = q0_dot*1;
            q_dot.data = 5*pseudoinverse(L_*robot_->getEEJacobian().data)*sd+N_*q0_dot;

            return q_dot;
        }

        KDL::JntArray positioning(){

            KDL::JntArray q_dot; q_dot.resize(robot_->getNrJnts());

            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(robot_->getEEFrame().p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(desired_frame_.M), toEigen(robot_->getEEFrame().M));

            Vector6d cartvel; cartvel << p.vel + 10*error, 3*o_error;
            q_dot.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;

            return q_dot;
        }

        KDL::Frame desiredFrame(){
            KDL::Frame des;

            KDL::Frame aruco_world_;
            aruco_world_ = robot_->getEEFrame()*aruco_;

            RCLCPP_INFO(this->get_logger(), "Aruco pos: %f,%f,%f", aruco_world_.p.data[0],aruco_world_.p.data[1],aruco_world_.p.data[2]);

            //position with a specified offset along x
            des.p.data[0] = aruco_world_.p.data[0]+x_offset_;
            des.p.data[1] = aruco_world_.p.data[1]+y_offset_;
            des.p.data[2] = aruco_world_.p.data[2]+z_offset_;

            KDL::Rotation y_rotation = KDL::Rotation::RotX(M_PI)*KDL::Rotation::RotZ(-M_PI/2);//*KDL::Rotation::RotZ(-M_PI/2);

            KDL::Rotation offset_rotation;

            if(roll_offset_ != 0.0 || pitch_offset_ != 0.0 || yaw_offset_ != 0.0){
                offset_rotation = KDL::Rotation::RPY(roll_offset_,pitch_offset_,yaw_offset_);
            }else offset_rotation = KDL::Rotation::Identity();
    
            des.M = aruco_world_.M*y_rotation*offset_rotation;

            return des;
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::Publisher<ros2_kdl_package::msg::Error>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_accelerations_;

        KDL::JntArray joint_pos_;
        KDL::JntArray joint_vel_;
        KDL::JntArray joint_acc_;

        KDL::JntArray initial_joint_pos_;

        Eigen::Vector<double,6> x_dot_;
        Eigen::Vector<double,6> x_;

        Eigen::VectorXd joint_efforts_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        int iteration_;
        bool joint_state_available_;
        double t_;

        bool aruco_info_;

        bool condition_;

        trajectory_point p;

        //class parameters
        std::string cmd_interface_;
        std::string vel_prof_;
        std::string trajectory_;
        std::string task_;

        double Kdd_;
        double Kpp_;

        double Kpp_o;
        double Kpo_o;
        double Kdp_o;
        double Kdo_o;

        double x_offset_;
        double y_offset_;
        double z_offset_;
        double roll_offset_;
        double pitch_offset_;
        double yaw_offset_;

        //gains for second order kinematics
        Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(6,6)*2;
        Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(6,6)*10;

        //Aruco pose
        KDL::Frame aruco_;
        KDL::Frame desired_frame_;

        KDL::Frame init_cart_pose_;
        Eigen::Vector3d prev_error_ = Eigen::VectorXd::Zero(3);
};


int main( int argc, char** argv )
{       
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}