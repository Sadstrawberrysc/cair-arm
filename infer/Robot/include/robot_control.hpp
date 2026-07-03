#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

class RobotCtr
{
public:
    // Calculate real force and torque (ft) and kalman filter parameters
    int kalman_filter_time_ft;
    Eigen::Matrix<double,6,1> xk_last_ft, pk_last_ft;
    Eigen::Matrix<double,6,1> sensor_ft, real_ft, gravity_error, tool_center;
    
    // Main control parameters
    float cycle_time;
    Eigen::Matrix3d cur_rot, next_rot;
    // Reference rotation (fixed) used to keep x/y translations aligned to initial direction
    Eigen::Matrix3d ref_rot;
    Eigen::Vector3d cur_pos, next_pos;

    // Force control parameters (Z axis)
    float desired_force, force_m, force_d, cur_z_velo, next_z_velo, delta_z;

    // Rotation control parameters ( RX & RY axis) and kalman filter parameters
    int kalman_filter_time_ics;
    Eigen::Vector3d xk_last_ics, pk_last_ics;
    Eigen::Vector3d desired_contact_point, cur_contact_point;
    std::vector<double> key_control; // Key control vector for manual control
    float rx_m, rx_d, cur_rx_velo, next_rx_velo, delta_rx; // RX -- Out of plane scan
    float ry_m, ry_d, cur_ry_velo, next_ry_velo, delta_ry; // RY -- In plane scan

    // Model control parameters (Y & Rz axis)
    double model_x_pos, model_y_pos, model_rz_pos, kp_y, kp_rz;
    double delta_y, delta_rz;

    // Scan trajectory tracking parameters (X axis)
    double delta_x;
    int scan_time; // For judge when to stop scan
    
    // End-effector length
    Eigen::Vector3d tool_length;

    // Scan finish flag
    bool flag_scaning;
    int step_count;


public:
    RobotCtr();
    void updateForceSensorZero();
    void calRealFT();
    void ForceCtr();
    void ModelCtr();
    void RotationCtr();
    void TrajectoryCtr();
    void mainCtr();
};

