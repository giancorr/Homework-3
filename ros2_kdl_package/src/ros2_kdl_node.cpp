#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include "math.h"

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
            // declare cmd_interface parameter (position, velocity, effort_j, effort_o)
            declare_parameter("cmd_interface", ""); // defaults to "position"
            // declare vel_prof parameter (trapezoidal, cubical)
            declare_parameter("vel_prof", ""); //defaults to "trapezoidal"
            // declare trajectory parameter (linear, circular)
            declare_parameter("trajectory", ""); //defaults to "linear"

            declare_parameter("Kpp", 0.0);  //gain for proportional action implemented for the inverse dynamic controller in joint space
            declare_parameter("Kdd", 0.0);  //gain for derivative action implemented for the inverse dynamic controller in joint space
            //declare_parameter("Kp", 0.0);
            //declare_parameter("Kd", 0.0);
            declare_parameter("Kpp_o", 0.0);
            declare_parameter("Kpo_o", 0.0);
            declare_parameter("Kdp_o", 0.0);
            declare_parameter("Kdo_o", 0.0);

            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("vel_prof", vel_prof_);
            get_parameter("trajectory", trajectory_);
            get_parameter("Kpp", Kpp_);
            get_parameter("Kdd", Kdd_);
            get_parameter("Kpp_o", Kpp_o);
            get_parameter("Kpo_o", Kpo_o);
            get_parameter("Kdp_o", Kdp_o);
            get_parameter("Kdo_o", Kdo_o);
            //get_parameter("Kp", Kp_);
            //get_parameter("Kd", Kd_);

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
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; 
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;    
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 


            joint_efforts_ = Eigen::VectorXd::Zero(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];

            // Plan trajectory
            double traj_duration = 3.0, acc_duration = 0.6, radius = 0.2;

            if(trajectory_ == "circular"){ 
                planner_ = KDLPlanner(traj_duration, init_position, radius, vel_prof_);
            } else if(trajectory_ == "linear"){
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position, vel_prof_);
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
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
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

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 3.0; // 
            int trajectory_len = 300; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            
            //unsigned int nj = robot_->getNrJnts();            
            KDLController controller_(*robot_);

            // Compute EE frame and twist
            KDL::Frame cartpos = robot_->getEEFrame();           
            KDL::Twist cart_twist = robot_->getEEVelocity();

            if(t_ < total_time) { 
                p = planner_.compute_trajectory(t_);
                RCLCPP_INFO(this->get_logger(), "Normal trajectory...");

            }else 
              {
                p = p;  //planner_.compute_trajectory(total_time); 
                RCLCPP_INFO(this->get_logger(), "Stop trajectory...");
              } 


            // Compute desired Frame
            KDL::Frame desFrame; desFrame.M = init_cart_pose_.M; desFrame.p = toKDL(p.pos);
            
            //desired frame velocity
            KDL::Twist desVel; desVel.vel=toKDL(p.vel);

            //desired frame acceleration
            KDL::Twist desAcc; desAcc.vel=toKDL(p.acc);


            // compute errors 
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data)); 
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

            //compute e_dot
            Vector6d e_, e_dot;
            computeErrors(desFrame, cartpos, desVel, cart_twist, e_, e_dot);


            if (t_ <= total_time){


                RCLCPP_INFO(this->get_logger(), "The error norm is : %f", error.norm());


                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "velocity"){
 
                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;

                }else if(cmd_interface_ == "effort_j"){

                    //Compute joint vel with differential IK and then integrate and differentiate for pos and acc
                    Vector6d des_cartvel; des_cartvel << p.vel + 10*error, o_error;
                    KDL::JntArray prev_joint_velocities; prev_joint_velocities.resize(robot_->getNrJnts());
                    prev_joint_velocities.data=joint_velocities_.data;

                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*des_cartvel; 
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                    joint_accelerations_.data = (joint_velocities_.data-prev_joint_velocities.data)/dt;

                    joint_efforts_ = controller_.idCntr(joint_positions_,joint_velocities_,joint_accelerations_,Kpp_, Kdd_);
                
                }else{

                    Eigen::MatrixXd Jac = robot_->getEEJacobian().data;
                    Vector6d dxd_; dxd_ << p.vel, 0.0, 0.0, 0.0; //x_dot desired
                    Vector6d ddxd_; ddxd_ << p.acc, 0.0, 0.0, 0.0; //x_ddot desired

                    //Compute joint acc with second-order IK and then differentiate
                    joint_accelerations_.data = pseudoinverse(Jac)*(ddxd_ - robot_->getEEJacDot() + Kp*e_ + Kd*e_dot);
                    joint_velocities_.data = joint_velocities_.data + joint_accelerations_.data*dt;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt + 0.5*joint_accelerations_.data*dt*dt;

                    joint_efforts_ = controller_.idCntr_o(desFrame, desVel, desAcc, Kpp_o, Kpo_o, Kdp_o, Kdo_o);

                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

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
                    for (long int i = 0; i < joint_efforts_.size(); ++i) {
                        desired_commands_[i] = joint_efforts_(i);
                    }
                    
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                KDL::Frame ee_pose = robot_->getEEFrame();
                RCLCPP_INFO(this->get_logger(), "The EE pose is: %f", ee_pose.p.data[2]);

            }
            else{

                //When t_>total_time we land here
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                //With joint vel control we just put vel to zero 
                if(cmd_interface_ == "velocity" || cmd_interface_ == "position"){
                    
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }

                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);


                }else if(cmd_interface_ == "effort_j"){
                    RCLCPP_INFO(this->get_logger(), "Generating stop torques");

                    Vector6d des_cartvel; des_cartvel << p.vel + 10*error, o_error;
                    KDL::JntArray prev_joint_velocities; prev_joint_velocities.resize(robot_->getNrJnts());
                    prev_joint_velocities.data=joint_velocities_.data;

                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*des_cartvel; 
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                    joint_accelerations_.data = (joint_velocities_.data-prev_joint_velocities.data)/dt;

                    joint_efforts_ = controller_.idCntr(joint_positions_,joint_velocities_,joint_accelerations_,Kpp_, Kdd_);

                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                    for (long int i = 0; i < joint_efforts_.size(); ++i) {
                        desired_commands_[i] = joint_efforts_(i);
                    }

                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);

                }else{
                    //effort_o 
                    Eigen::MatrixXd Jac = robot_->getEEJacobian().data;

                    Vector6d dxd_; dxd_ << p.vel, 0.0, 0.0, 0.0; //x_dot
                    Vector6d ddxd_; ddxd_ << p.acc, 0.0, 0.0, 0.0; //x_ddot

                    //Again Second order IK
                    joint_accelerations_.data = pseudoinverse(Jac)*(ddxd_ - robot_->getEEJacDot() + Kp*e_ + Kd*e_dot);
                    joint_velocities_.data = joint_velocities_.data + joint_accelerations_.data*dt;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt + 0.5*joint_accelerations_.data*dt*dt;

                    joint_efforts_ = controller_.idCntr_o(desFrame, desVel, desAcc, Kpp_o, Kpo_o, Kdp_o, Kdo_o);

                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                    
                    for (long int i = 0; i < joint_efforts_.size(); ++i) {
                        desired_commands_[i] = joint_efforts_(i);
                    }

                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                }
                             
            }
            //Updating t_
            t_+=dt;
        }
        

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

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
        KDL::JntArray joint_accelerations_;

        Eigen::VectorXd joint_efforts_;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        int iteration_;
        bool joint_state_available_;
        double t_;

        trajectory_point p;

        //class parameters
        std::string cmd_interface_;
        std::string vel_prof_;
        std::string trajectory_;

        double Kdd_;
        double Kpp_;

        double Kpp_o; 
        double Kpo_o;
        double Kdp_o; 
        double Kdo_o;

        //gains for second order kinematics
        Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(6,6)*2;
        Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(6,6)*10;
  

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
