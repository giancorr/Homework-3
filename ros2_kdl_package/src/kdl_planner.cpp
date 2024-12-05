#include "kdl_planner.h"
#include <cmath>

KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, std::string vel_prof)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    vel_prof_ = vel_prof;
    trajectory_ = "linear";
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _radius, std::string vel_prof)
{   
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    radius_ = _radius;
    vel_prof_ = vel_prof;
    trajectory_ = "circular";
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

void KDLPlanner::trapezoidal_vel(double time,
                            //double acc_time,  
                            double &s, 
                            double &s_dot, 
                            double &s_ddot)
{ 

  double acc_time = 0.5;
  double fin_time = trajDuration_;
  double s_ddot_c = 1/(acc_time*(fin_time-acc_time));

  if(time <= acc_time){
    s = 0.5*s_ddot_c*std::pow(time,2);
    s_dot = s_ddot_c * time;
    s_ddot = s_ddot_c;
    
  }
  else if(time <= fin_time - acc_time){
    s = s_ddot_c*acc_time*(time-acc_time/2);
    s_dot = s_ddot_c * acc_time;
    s_ddot = 0;
    
  } 
  else
  {
    s = 1 - 0.5*s_ddot_c*std::pow(fin_time-time,2);
    s_dot = s_ddot_c * (fin_time-time);
    s_ddot = -s_ddot_c;
    
  } 
}

void KDLPlanner::cubic_polinomial(double time,
                                  double &s, 
                                  double &s_dot, 
                                  double &s_ddot)
{
  double fin_time = trajDuration_;
  double a_0, a_1, a_2, a_3;
  a_0 = 0.0;
  a_1 = 0.0;
  a_2 = 3/std::pow(fin_time, 2);
  a_3 = -2/std::pow(fin_time, 3);

  s = a_3*std::pow(time, 3) + a_2*std::pow(time,2)+a_1*time+a_0;
  s_dot = 3*a_3*std::pow(time, 2) + 2*a_2*time + a_1;
  s_ddot = 6*a_3*time + 2*a_2;
}

void KDLPlanner::circular_trajectory(trajectory_point &traj, double s, double s_dot, double s_ddot)
{ 
  traj.pos[0] = trajInit_[0];
  traj.pos[1] = (trajInit_[1]+radius_)+radius_*cos(2*M_PI*s);
  traj.pos[2] = (trajInit_[2])+radius_*sin(2*M_PI*s);

  traj.vel[0] = 0.0;
  traj.vel[1] = 2*M_PI*radius_*sin(2*M_PI*s)*s_dot;
  traj.vel[2] = -2*M_PI*radius_*cos(2*M_PI*s)*s_dot;

  traj.acc[0] = 0.0;
  traj.acc[1] = std::pow(2*M_PI,2)*radius_*cos(2*M_PI*s)*std::pow(s_dot,2) + 2*M_PI*radius_*sin(2*M_PI*s)*s_ddot;
  traj.acc[2] = std::pow(2*M_PI,2)*radius_*sin(2*M_PI*s)*std::pow(s_dot,2) - 2*M_PI*radius_*cos(2*M_PI*s)*s_ddot;
}

void KDLPlanner::linear_trajectory(trajectory_point &traj, double s, double s_dot, double s_ddot)
{
  for(int i=0; i<3; i++){
    traj.pos[i] = trajInit_[i] + s*(trajEnd_[i]-trajInit_[i]);
  }
  for(int i=0; i<3; i++){
    traj.vel[i] = + s_dot*(trajEnd_[i]-trajInit_[i]);
  }
  for(int i=0; i<3; i++){
    traj.acc[i] = + s_ddot*(trajEnd_[i]-trajInit_[i]);
  }
  
}

// void KDLPlanner::angular_traj(trajectory_point &traj, KDL::Frame init_frame, KDL::Frame end_frame, double th, double th_dot, double th_ddot)
// { 
//   Eigen::Vector3d wi = Eigen::Vector3d::Zero();
//   Eigen::Vector3d wi_dot = Eigen::Vector3d::Zero();
//   Eigen::Vector3d r = Eigen::Vector3d::Zero();
//   Eigen::Matrix Rfi = toEigen(init_frame.M).transpose()*toEigen(end_frame.M);
//   double thetaf = acos((Rfi(0)(0)+Rfi(1)(1)+Rfi(2)(2)-1)/2);
//   if(thetaf != 0.0){
//     r[0] = (Rfi(2)(1)-Rfi(1)(2))/(2*sin(traj.theta_));
//     r[1] = (Rfi(0)(2)-Rfi(2)(0))/(2*sin(traj.theta_));
//     r[2] = (Rfi(1)(0)-Rfi(0)(1))/(2*sin(traj.theta_));
//   }
//   traj.theta_ = th * thetaf;
//   traj.theta_dot = th_dot * thetaf;
//   traj.theta_ddot = th_ddot * thetaf;

//   wi = traj.theta_dot * traj.r;
//   wi_dot = traj.theta_ddot * traj.r;
//   traj.we = init_frame.M.data*wi;
//   traj.we_dot = init_frame.M.data*wi_dot;
// }

trajectory_point KDLPlanner::compute_trajectory(double time)
{

  double s = 0.0, s_dot = 0.0, s_ddot = 0.0;
  double th = 0.0, th_dot = 0.0, th_ddot = 0.0;
  
  if(vel_prof_ == "cubic"){
    cubic_polinomial(time, s,s_dot, s_ddot);
    cubic_polinomial(time, th, th_dot, th_ddot);
  }else if (vel_prof_ == "trapezoidal"){
    trapezoidal_vel(time, s, s_dot, s_ddot);
  }

  trajectory_point traj;

  if(trajectory_ == "circular"){
    circular_trajectory(traj, s, s_dot, s_ddot);
  }
  else if(trajectory_ == "linear"){
    linear_trajectory(traj, s, s_dot, s_ddot);
  }
  
  return traj;

}


