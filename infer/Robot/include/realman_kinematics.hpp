#include <math.h>
#include <iostream>
#include <Eigen/Dense>

class RMKinematics
{
public:
    float lsb,lse,lew,lwt;// DH Parameters
public:
    RMKinematics();
    void GetKinematics(Eigen::Matrix4d& kinematics, Eigen::Matrix<double,6,1>& joints);
    void GetJacobian(Eigen::Matrix<double,6,6>& jacobian, Eigen::Matrix<double,6,1>& joints);
    void GetNextJoints(Eigen::Matrix<double,6,1>& next_joints, Eigen::Matrix<double,6,1>& cur_joints, Eigen::Matrix4d& next_kinematics);
};