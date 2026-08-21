#include <realman_kinematics.hpp>

#include <cmath>

namespace {

constexpr double kMmToM = 0.001;
constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

RMKinematics::RMKinematics(){
    mdh_alpha  = {0, -90*kDegToRad, 90*kDegToRad, -90*kDegToRad,
                  90*kDegToRad, -90*kDegToRad, 90*kDegToRad};
    mdh_d      = {240.5*kMmToM, 0, 256.0*kMmToM, 0,
                  210.0*kMmToM, 0, 144*kMmToM};
    // RM75 MDH table
    //
    // | joint | a(i-1) m | alpha(i-1) deg | d(i) mm | offset deg |
    // | ----- | -------- | -------------- | ------ | ---------- |
    // | J1    | 0        | 0              | 240.5  | 0          |
    // | J2    | 0        | -90            | 0      | 0          |
    // | J3    | 0        | 90             | 256.0  | 0          |
    // | J4    | 0        | -90            | 0      | 0          |
    // | J5    | 0        | 90             | 210.0  | 0          |
    // | J6    | 0        | -90            | 0      | 0          |
    // | J7    | 0        | 90             | 144.0  | 0          |
    joint_min = {-178*kDegToRad, -130*kDegToRad, -178*kDegToRad,
                 -135*kDegToRad, -178*kDegToRad, -128*kDegToRad,
                 -360*kDegToRad};
    joint_max = { 178*kDegToRad,  130*kDegToRad,  178*kDegToRad,
                  135*kDegToRad,  178*kDegToRad,  128*kDegToRad,
                  360*kDegToRad};

}

Eigen::Matrix4d RMKinematics::GetModifiedDhTransform(double alpha,
                                                     double d,
                                                     double theta) const {
    Eigen::Matrix4d transform;
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    transform << ct,     -st,      0,      0,
                 st*ca,   ct*ca,  -sa,    -d*sa,
                 st*sa,   ct*sa,   ca,     d*ca,
                 0,       0,       0,      1;
    return transform;
}

void RMKinematics::GetKinematics(Eigen::Matrix4d& kinematics,
                                 Eigen::Matrix<double,7,1>& joints){
    kinematics.setIdentity();
    for (int i = 0; i < kJointCount; ++i) {
        kinematics = kinematics
            * GetModifiedDhTransform(mdh_alpha[i], mdh_d[i], joints[i]);
    }
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
