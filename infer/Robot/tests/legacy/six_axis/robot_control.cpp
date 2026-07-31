#include <robot_control.hpp>

RobotCtr::RobotCtr(){
    // Kalman Filter parameters for gravity compensation
    kalman_filter_time_ft = 0;
    
    // Admittance control time
    cycle_time = 0.01;

    // Force control parameters
    force_m = 3;
    force_d = 20;

    desired_force = -1.5;

    // Rotation control parameters
    kalman_filter_time_ics = 0;

    rx_m = 2;
    rx_d = 10;

    ry_m = 2;
    ry_d = 10; 
    
    desired_contact_point << 0,0,0;

    // Model control parameters
    kp_y = 0.5;
    kp_rz = 0.05;
    model_y_pos = 0;
    model_x_pos = 0;
    model_rz_pos = 0;

    // Scan trajectory tracking parameters
    scan_time = 0;

    // End-effector length
    tool_length << 0, 0, 0.2305;

    // Scan finish flag
    step_count = 0;
    flag_scaning = true;

}

void RobotCtr::updateForceSensorZero(){
    Eigen::Vector3d sensor_f, sensor_t, zero_f, zero_t, gravity_sensor, moment_gravity, moment_tool;
    sensor_f << sensor_ft[0],sensor_ft[1],sensor_ft[2];
    sensor_t << sensor_ft[3],sensor_ft[4],sensor_ft[5];

    // Check if F/T is out of sensor range
    double f_max = std::max(std::max(fabs(sensor_ft[0]),fabs(sensor_ft[1])),fabs(sensor_ft[2]));
    double t_max = std::max(std::max(fabs(sensor_ft[3]),fabs(sensor_ft[4])),fabs(sensor_ft[5]));
    if (f_max > 50 || t_max > 5) {
        std::cout << "Warning! Force is beyond F/T sensor's range" << std::endl;
        std::exit(0);
    }

    // Gravity compensation
    gravity_sensor = cur_rot.inverse()*gravity_error.head(3);
    moment_gravity << gravity_sensor(2)*tool_center(1,0) - gravity_sensor(1)*tool_center(2,0),
                      gravity_sensor(0)*tool_center(2,0) - gravity_sensor(2)*tool_center(0,0),
                      gravity_sensor(1)*tool_center(0,0) - gravity_sensor(0)*tool_center(1,0);
    zero_f = sensor_f - gravity_sensor;
    zero_t = sensor_t - moment_gravity;
    
    // Update gravity compensation parameters
    Eigen::Vector3d zero_t_para;
    zero_t_para << zero_t(0) - zero_f(2)*tool_center(1,0) + zero_f(1)*tool_center(2,0),
                   zero_t(1) - zero_f(0)*tool_center(2,0) + zero_f(2)*tool_center(0,0),
                   zero_t(2) - zero_f(1)*tool_center(0,0) + zero_f(0)*tool_center(1,0);
    gravity_error << gravity_error.head(3),zero_f;
    tool_center   << tool_center.head(3),zero_t_para;
    
    std::cout << "gravity_error = " << gravity_error.transpose() << std::endl;
    std::cout << "tool_center = " << tool_center.transpose() << std::endl;
}

void RobotCtr::calRealFT(){
    Eigen::Vector3d sensor_f, sensor_t, real_f, real_t, gravity_sensor, moment_gravity, moment_tool;
    sensor_f << sensor_ft[0],sensor_ft[1],sensor_ft[2];
    sensor_t << sensor_ft[3],sensor_ft[4],sensor_ft[5];
    
    // Check if F/T is out of sensor range
    double f_max = std::max(std::max(fabs(sensor_ft[0]),fabs(sensor_ft[1])),fabs(sensor_ft[2]));
    double t_max = std::max(std::max(fabs(sensor_ft[3]),fabs(sensor_ft[4])),fabs(sensor_ft[5]));
    if (f_max > 50 || t_max > 5) {
        std::cout << "Warning! Force is beyond F/T sensor's range" << std::endl;
        std::exit(0);
    }

    // Gravity compensation
    gravity_sensor = cur_rot.inverse()*gravity_error.head(3);
    moment_gravity << gravity_sensor(2)*tool_center(1,0) - gravity_sensor(1)*tool_center(2,0),
                      gravity_sensor(0)*tool_center(2,0) - gravity_sensor(2)*tool_center(0,0),
                      gravity_sensor(1)*tool_center(0,0) - gravity_sensor(0)*tool_center(1,0);
    moment_tool << tool_center(3,0) - gravity_error(4,0)*tool_center(2,0) + gravity_error(5,0)*tool_center(1,0),
                   tool_center(4,0) - gravity_error(5,0)*tool_center(0,0) + gravity_error(3,0)*tool_center(2,0),
                   tool_center(5,0) - gravity_error(3,0)*tool_center(1,0) + gravity_error(4,0)*tool_center(0,0);
    real_f = sensor_f - (gravity_sensor + gravity_error.tail(3));
    real_t = sensor_t - (moment_gravity + moment_tool);
    
    real_ft << real_f, real_t;

    // Kalman Filter
    float Rk = 0.0002;
    float Bk = 0.000001;
    float xk_bar, pk_bar;
    if (kalman_filter_time_ft == 0){
        xk_last_ft = real_ft;
        pk_last_ft << Bk,Bk,Bk,Bk,Bk,Bk;
    }
    for (int i=0; i<6; i++){
        xk_bar = xk_last_ft[i];
        pk_bar = pk_last_ft[i] + Bk;
        real_ft[i] = xk_bar + (pk_bar/(pk_bar+Rk))*(real_ft[i]-xk_bar);
        // Update state
        xk_last_ft[i] = real_ft[i];
        pk_last_ft[i] = pk_bar - (pk_bar/(pk_bar+Rk))*pk_bar;
    }
    kalman_filter_time_ft ++;
}

void RobotCtr::ForceCtr(){
    float acc;
    // Check if contact occurs
    if (fabs(real_ft[2]) > 0.99){ // when desired_force = 1, not enter force ctr loop
        acc = (real_ft[2] - desired_force - force_d*cur_z_velo)/force_m;
        delta_z = cur_z_velo*cycle_time + 0.5*acc*cycle_time*cycle_time;
        // Update velocity and limitation
        next_z_velo = cur_z_velo + acc*cycle_time;
        if(next_z_velo >  0.5)   next_z_velo =  0.5;
        if(next_z_velo < -0.5)   next_z_velo = -0.5;
    }else{
        next_z_velo = 0;
        delta_z = 0.003; // Unit : m
    }
    cur_z_velo = next_z_velo;

    // Security protect
    if (fabs(delta_z) > 0.01){
        std::cout << "Warning: X axis too fast!" << std::endl;
        // std::exit(0);
        delta_z = 0.003;
    }
}

void RobotCtr::RotationCtr(){
    Eigen::Vector3d contact_point;

    // Kalman filter
    float Rk = 0.0002;
    float Bk = 0.0000005;
    float xk_bar,pk_bar;
    if (kalman_filter_time_ics == 0){ // Initialzation
        xk_last_ics = cur_contact_point;
        pk_last_ics << Bk,Bk,Bk;
    }
    for (int i=0; i<3; i++){
        xk_bar = xk_last_ics[i];
        pk_bar = pk_last_ics[i] + Bk;
        contact_point[i] = xk_bar + (pk_bar/(pk_bar+Rk))*(cur_contact_point[i]-xk_bar);
        // Update state
        xk_last_ics[i] = contact_point[i];
        pk_last_ics[i] = pk_bar - (pk_bar/(pk_bar+Rk))*pk_bar;
    }
    kalman_filter_time_ics ++;

    // Admittance control
    double rx_acc;
    if (contact_point[0] == 0){
        delta_rx = 0; cur_rx_velo = 0;
    }else{
        rx_acc = (1000*contact_point[0] - 1000*desired_contact_point[0] - rx_d * cur_rx_velo) / rx_m;
        delta_rx = cur_rx_velo * cycle_time + 0.5 * rx_acc * cycle_time * cycle_time;
        // Update velocity and limitation
        next_rx_velo = cur_rx_velo + rx_acc * cycle_time;
        if (next_rx_velo > 1.0) next_rx_velo =  1.0;
        if (next_rx_velo <-1.0) next_rx_velo = -1.0;
        cur_rx_velo = next_rx_velo;
    }

    double ry_acc;
    if (contact_point[1] == 0){
        delta_ry = 0; cur_ry_velo = 0;
    }else{
        ry_acc = (1000*contact_point[1] - 1000*desired_contact_point[1] - ry_d * cur_ry_velo) / ry_m;
        delta_ry = cur_ry_velo * cycle_time + 0.5 * ry_acc * cycle_time * cycle_time;
        // Update velocity and limitation
        next_ry_velo = cur_ry_velo + ry_acc * cycle_time;
        if (next_ry_velo > 1.0) next_ry_velo =  1.0;
        if (next_ry_velo <-1.0) next_ry_velo = -1.0;
        cur_ry_velo = next_ry_velo;
    }

    // Scale
    delta_rx = 0*delta_rx;
    delta_ry =  1.5*delta_ry;

    // Security protect
    if (fabs(delta_rx) > 0.5 || fabs(delta_ry) > 0.5){
        std::cout << "Warning: RX & RY axis too fast!" << std::endl;
        std::exit(0);
    }
    cur_contact_point = contact_point;
}

void RobotCtr::ModelCtr(){
    
    delta_x = model_x_pos;
    const double MAX_STEP_Y = 0.02;  
    delta_y = kp_y * model_y_pos;   

    if (delta_y >  MAX_STEP_Y) delta_y =  MAX_STEP_Y;
    if (delta_y < -MAX_STEP_Y) delta_y = -MAX_STEP_Y;
    
    model_y_pos = 0;

    delta_rz = kp_rz * (model_rz_pos) * M_PI/180;
    // std::cout << "delta_rz = " << delta_rz << std::endl;
    if (delta_rz <  0.05 && delta_rz > 0) delta_rz =  0.04;
    if (delta_rz > -0.05 && delta_rz < 0) delta_rz = -0.04;
    delta_rz = abs(delta_rz);
}

void RobotCtr::TrajectoryCtr(){
    if (fabs(real_ft[2]) > 0.99){
        if (step_count < 500){
            delta_x = 0.001; // Unit : m
            step_count++;
        }else{
            delta_x = 0;
            flag_scaning = false;
        }
    }
}

void RobotCtr::mainCtr(){
    // Initialization
    delta_x = 0; delta_y = 0; delta_z = 0;
    delta_rx = 0; delta_ry = 0; delta_rz = 0;
    
    // Control Cycle
    ForceCtr();
    ModelCtr();

    RotationCtr();
    // TrajectoryCtr();

    delta_rx = 0;
    // delta_rz = 0;

    // Calculate next kinemtaics
    Eigen::Vector3d delta_pos;
    delta_pos << delta_x, delta_y, delta_z;
    // std::cout << "delta_pos = " << delta_pos.transpose() << std::endl;
    Eigen::Matrix3d tmp_rot_x, tmp_rot_y, tmp_rot_z;

    tmp_rot_x << std::cos(delta_rx),0,std::sin(delta_rx),
                 0,                 1,                  0,
                 -std::sin(delta_rx),0,std::cos(delta_rx);
    
    tmp_rot_y << 1,                 0,                  0,
                 0,std::cos(delta_ry),-std::sin(delta_ry),
                 0,std::sin(delta_ry),std::cos(delta_ry);

    tmp_rot_z << std::cos(delta_rz),-std::sin(delta_rz),0,
                 std::sin(delta_rz),std::cos(delta_rz),0,
                 0,               0,                   1;
    next_rot = cur_rot*tmp_rot_x*tmp_rot_y*tmp_rot_z;

    // Rotation based on TCP
    // Keep translations along the fixed initial direction using ref_rot, independent of current rotation
    next_pos = next_rot * tool_length - cur_rot * tool_length + cur_pos + ref_rot * delta_pos;

    // std::cout << "next_pos = " << next_pos.transpose() << std::endl;
    // std::cout << "next_rot = " << next_rot << std::endl;
}

