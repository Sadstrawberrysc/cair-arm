#include <realman_kinematics.hpp>

namespace {

constexpr double kMmToM = 0.001;
constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

RMKinematics::RMKinematics(){
    // RM75 official MDH parameters:
    // a(i-1), alpha(i-1), d(i), offset(i). Units converted to m/rad.
    mdh_a      = {0, 0, 0, 0, 0, 0, 0};
    mdh_alpha  = {0, -90*kDegToRad, 90*kDegToRad, -90*kDegToRad,
                  90*kDegToRad, -90*kDegToRad, 90*kDegToRad};
    mdh_d      = {240.5*kMmToM, 0, 256.0*kMmToM, 0,
                  210.0*kMmToM, 0, 0};
    joint_offset = {0, 0, 0, 0, 0, 0, 0};

    joint_min = {-178*kDegToRad, -130*kDegToRad, -178*kDegToRad,
                 -135*kDegToRad, -178*kDegToRad, -128*kDegToRad,
                 -360*kDegToRad};
    joint_max = { 178*kDegToRad,  130*kDegToRad,  178*kDegToRad,
                  135*kDegToRad,  178*kDegToRad,  128*kDegToRad,
                  360*kDegToRad};

    // Controller "Arm_Tip" pose is offset from the MDH joint-7 frame.
    // The current RM75 reports zero tool frame, so keep this as the robot
    // geometric tip offset and validate it across multiple poses.
    tool_offset << 0.0, 0.0, 0.143998;
}

Eigen::Matrix4d RMKinematics::GetModifiedDhTransform(double a,
                                                     double alpha,
                                                     double d,
                                                     double theta) const {
    Eigen::Matrix4d transform;
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    transform << ct,     -st,      0,      a,
                 st*ca,   ct*ca,  -sa,    -d*sa,
                 st*sa,   ct*sa,   ca,     d*ca,
                 0,       0,       0,      1;
    return transform;
}

void RMKinematics::GetKinematics(Eigen::Matrix4d& kinematics,
                                 Eigen::Matrix<double,7,1>& joints){
    kinematics.setIdentity();
    for (int i = 0; i < kJointCount; ++i) {
        const double theta = joints[i] + joint_offset[i];
        kinematics = kinematics * GetModifiedDhTransform(mdh_a[i], mdh_alpha[i], mdh_d[i], theta);
    }
    kinematics.block<3,1>(0,3) += kinematics.block<3,3>(0,0) * tool_offset;
}

Eigen::Matrix<double,6,1> RMKinematics::GetPoseError(const Eigen::Matrix4d& current,
                                                     const Eigen::Matrix4d& target) const {
    Eigen::Matrix<double,6,1> error;
    error.head<3>() = target.block<3,1>(0,3) - current.block<3,1>(0,3);

    Eigen::Matrix3d current_rot = current.block<3,3>(0,0);
    Eigen::Matrix3d target_rot = target.block<3,3>(0,0);
    Eigen::Matrix3d delta_rot = target_rot * current_rot.transpose();
    Eigen::AngleAxisd angle_axis(delta_rot);
    error.tail<3>() = angle_axis.axis() * angle_axis.angle();
    return error;
}

void RMKinematics::GetJacobian(Eigen::Matrix<double,6,7>& jacobian,
                               Eigen::Matrix<double,7,1>& joints){
    constexpr double eps = 1e-6;
    Eigen::Matrix4d plus_kinematics;
    Eigen::Matrix4d minus_kinematics;
    jacobian.setZero();

    for (int i = 0; i < kJointCount; ++i) {
        Eigen::Matrix<double,7,1> joints_plus = joints;
        Eigen::Matrix<double,7,1> joints_minus = joints;
        joints_plus[i] += eps;
        joints_minus[i] -= eps;

        GetKinematics(plus_kinematics, joints_plus);
        GetKinematics(minus_kinematics, joints_minus);

        Eigen::Matrix<double,6,1> diff;
        diff.head<3>() = (plus_kinematics.block<3,1>(0,3)
                        - minus_kinematics.block<3,1>(0,3)) / (2.0 * eps);

        Eigen::Matrix3d rot_delta = plus_kinematics.block<3,3>(0,0)
                                  * minus_kinematics.block<3,3>(0,0).transpose();
        Eigen::AngleAxisd angle_axis(rot_delta);
        diff.tail<3>() = angle_axis.axis() * angle_axis.angle() / (2.0 * eps);
        jacobian.col(i) = diff;
    }
}

void RMKinematics::ApplyJointLimits(Eigen::Matrix<double,7,1>& joints) const {
    for (int i = 0; i < kJointCount; ++i) {
        if (joints[i] > joint_max[i]) joints[i] = joint_max[i];
        if (joints[i] < joint_min[i]) joints[i] = joint_min[i];
    }
}

void RMKinematics::GetNextJoints(Eigen::Matrix<double,7,1>& next_joints,
                                 Eigen::Matrix<double,7,1>& cur_joints,
                                 Eigen::Matrix4d& next_kinematics){
    Eigen::Matrix4d cur_kinematics;
    Eigen::Matrix<double,6,7> cur_jacobian;

    GetJacobian(cur_jacobian, cur_joints);
    GetKinematics(cur_kinematics, cur_joints);
    Eigen::Matrix<double,6,1> error_kinematics = GetPoseError(cur_kinematics, next_kinematics);

    constexpr double lambda = 0.0005;
    Eigen::Matrix<double,7,6> jacobian_pseudo_inverse =
        cur_jacobian.transpose()
        * ((cur_jacobian * cur_jacobian.transpose()
            + lambda * Eigen::Matrix<double,6,6>::Identity()).inverse());

    next_joints = cur_joints + jacobian_pseudo_inverse * error_kinematics;
    ApplyJointLimits(next_joints);
}
