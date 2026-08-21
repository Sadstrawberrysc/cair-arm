#include <array>
#include <Eigen/Dense>

class RMKinematics
{
public:
    static constexpr int kJointCount = 7;

    RMKinematics();
    void GetKinematics(Eigen::Matrix4d& kinematics, Eigen::Matrix<double,7,1>& joints);
    void GetJacobian(Eigen::Matrix<double,6,7>& jacobian, Eigen::Matrix<double,7,1>& joints);
    const std::array<double, kJointCount>& JointMinimums() const noexcept {
        return joint_min;
    }
    const std::array<double, kJointCount>& JointMaximums() const noexcept {
        return joint_max;
    }

private:
    std::array<double, kJointCount> mdh_alpha;
    std::array<double, kJointCount> mdh_d;
    std::array<double, kJointCount> joint_min;
    std::array<double, kJointCount> joint_max;

    Eigen::Matrix4d GetModifiedDhTransform(double alpha,
                                           double d,
                                           double theta) const;
};
