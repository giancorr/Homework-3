#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis()
            //+ robot_->getGravity() // due to Zero gravity world
            ;
}


Eigen::VectorXd KDLController::idCntr_o(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)

 {   

    Eigen::MatrixXd Jac = robot_->getEEJacobian().data;

    //read current cart state
    KDL::Frame cart_pos =robot_->getEEFrame();
    KDL::Twist cart_twist= robot_->getEEVelocity();             
                    

    //Define vel and pos error vectors
    Vector6d x_tilde;
    Vector6d dx_tilde;
    
    Eigen::VectorXd desAcc_; desAcc_=toEigen(_desAcc);

    //Transform Kp and Kd into matrixes
    Eigen::MatrixXd Kp(6,6);
    Eigen::MatrixXd Kd(6,6);
    Eigen::MatrixXd I= Eigen::MatrixXd::Identity(3,3);

    Kp.topLeftCorner(3,3)=I*_Kpp;
    Kp.bottomRightCorner(3,3)=I*_Kpo;
    Kd.topLeftCorner(3,3)=I*_Kdp;
    Kd.bottomRightCorner(3,3)=I*_Kdo;

    computeErrors(_desPos, cart_pos,_desVel, cart_twist, x_tilde, dx_tilde);

    Eigen::VectorXd y; y= pseudoinverse(Jac)*(desAcc_ + Kd*dx_tilde + Kp*x_tilde - robot_->getEEJacDot());
    
    return robot_->getJsim()*y + robot_->getCoriolis() 
    //+ robot_->getGravity() Zero gravity world
    ;
}

