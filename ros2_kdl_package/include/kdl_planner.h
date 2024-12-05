#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  double theta_ = 0.0, theta_dot = 0.0, theta_ddot = 0.0;
  Eigen::Vector3d we = Eigen::Vector3d::Zero();
  Eigen::Vector3d we_dot = Eigen::Vector3d::Zero();
  Eigen::Vector3d r;
};

class KDLPlanner
{

public:

    KDLPlanner();
    KDLPlanner(double _maxVel, double _maxAcc);
   
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    void trapezoidal_vel(double time, 
                         double &s, 
                         double &s_dot, 
                         double &s_ddot);

    void cubic_polinomial(double time,  
                         double &s, 
                         double &s_dot, 
                         double &s_ddot);

    //////////////////////////////////
    void circular_trajectory(trajectory_point &traj, double s, double s_dot, double s_ddot);
    void linear_trajectory(trajectory_point &traj, double s, double s_dot, double s_ddot);

    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, std::string vel_prof);

    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _radius, std::string vel_prof);
    trajectory_point compute_trajectory(double time);



private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_, accDuration_;
    Eigen::Vector3d trajInit_, trajEnd_;
    double radius_;
    std::string vel_prof_;
    std::string trajectory_;
    trajectory_point p;

};

#endif
