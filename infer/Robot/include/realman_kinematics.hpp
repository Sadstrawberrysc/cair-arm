#include <array>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

class RMKinematics
{
public:
    static constexpr int kJointCount = 7;
    static constexpr int kTaskDim = 6;

    std::array<double, kJointCount> mdh_a;
    std::array<double, kJointCount> mdh_alpha;
    std::array<double, kJointCount> mdh_d;
    std::array<double, kJointCount> joint_offset;
    std::array<double, kJointCount> joint_min;
    std::array<double, kJointCount> joint_max;
    Eigen::Vector3d tool_offset;

public:
    RMKinematics();
    void GetKinematics(Eigen::Matrix4d& kinematics, Eigen::Matrix<double,7,1>& joints);
    void GetJacobian(Eigen::Matrix<double,6,7>& jacobian, Eigen::Matrix<double,7,1>& joints);
    void GetNextJoints(Eigen::Matrix<double,7,1>& next_joints,
                       Eigen::Matrix<double,7,1>& cur_joints,
                       Eigen::Matrix4d& next_kinematics);

private:
    Eigen::Matrix4d GetModifiedDhTransform(double a, double alpha, double d, double theta) const;
    Eigen::Matrix<double,6,1> GetPoseError(const Eigen::Matrix4d& current,
                                           const Eigen::Matrix4d& target) const;
    void ApplyJointLimits(Eigen::Matrix<double,7,1>& joints) const;
};
