#include <realman_command.hpp>
#include <algorithm>
#include <cmath>

namespace {

constexpr double kMilliDegreeToRad = M_PI / 180000.0;
constexpr double kRadToMilliDegree = 180000.0 / M_PI;

bool SendAndParse(RMCommand& rm, const nlohmann::json& request, const char* label) {
    rm.cmd_str = request.dump() + "\r\n";
    memset(rm.send_msg, 0, 1000);
    strcpy(rm.send_msg, rm.cmd_str.c_str());

    if (send(rm.rlm_socket, rm.send_msg, strlen(rm.send_msg), 0) < 0) {
        std::cout << "ERROR! Can't send message in " << label << "!" << std::endl;
        std::exit(0);
    }

    memset(rm.recv_msg, 0, 1000);
    rm.recv_times = 0;
    while (recv(rm.rlm_socket, rm.recv_msg, 1000, 0) < 10 && rm.recv_times < 3) {
        rm.recv_times++;
    }
    if (rm.recv_times == 3) {
        std::cout << "ERROR! Can't receive message in " << label << "!" << std::endl;
        std::exit(0);
    }

    rm.return_msg.clear();
    rm.return_msg = nlohmann::json::parse(rm.recv_msg, nullptr, false);
    if (rm.return_msg.is_discarded()) {
        std::cout << "WARNING! Missing a return message in " << label << ". "
                  << rm.return_msg.dump() << std::endl;
        return false;
    }
    return true;
}

template <int JointCount>
void FillJointCommand(nlohmann::json& msg, const Eigen::Matrix<double, JointCount, 1>& joints) {
    for (int i = 0; i < joints.size(); i++) {
        msg["joint"][i] = int(kRadToMilliDegree * joints[i]);
    }
}

}  // namespace


RMCommand::RMCommand(){
    rlm_port = 8080;
    std::string string_ip = "192.168.50.254";
    strcpy(rlm_ip, string_ip.c_str());
    cmd_joints.setZero();
    cmd_pose.setZero();
    last_joint_count = 0;
    arm_err = 0;
    sys_err = 0;
}


void RMCommand::ConnectTCPSocket(){
    struct sockaddr_in server_addr;
    rlm_socket = socket(AF_INET,SOCK_STREAM, 0);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(rlm_ip);
    server_addr.sin_port = htons(rlm_port);
    rlm_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(connect(rlm_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
        std::cout << "ERROR! Can't connect robot!" << std::endl;
        std::exit(0);
    }else{
        std::cout << "Robot connect!" << std::endl;
    }
}


void RMCommand::SetHighSpeedEth(){
    // Open high speed ethernet
    command_msg.clear();
    command_msg["command"] = "set_high_speed_eth";
    command_msg["mode"] = 1;
    cmd_str = command_msg.dump()+"\r\n";
    // std::cout << cmd_str << std::endl;
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    }else{
        memset(recv_msg, 0, 1000);recv_times = 0;
        while(recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't recive message! " << std::endl;
            std::exit(0);
        }else{
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
            }else{
                std::cout << return_msg.dump() << std::endl;
            }
        }
    }

    // Set IP
    command_msg.clear();
    command_msg["command"] = "set_high_ethernet";
    command_msg["ip"] = "192.168.1.18";
    command_msg["mask"] = "255.255.255.0";
    command_msg["gateway"] = "192.168.1.1";
    cmd_str = command_msg.dump()+"\r\n";
    // std::cout << cmd_str << std::endl;
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    }else{
        memset(recv_msg, 0, 1000);recv_times = 0;
        while(recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't recive message! " << std::endl;
            std::exit(0);
        }else{
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
            }else{
                std::cout << return_msg.dump() << std::endl;
            }
        }
    }

    // Save info
    command_msg.clear();
    command_msg["command"] = "save_device_info_all";
    cmd_str = command_msg.dump()+"\r\n";
    // std::cout << cmd_str << std::endl;
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    }else{
        memset(recv_msg, 0, 1000);recv_times = 0;
        while(recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't recive message! " << std::endl;
            std::exit(0);
        }else{
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
            }else{
                std::cout << return_msg.dump() << std::endl;
            }
        }
    }
    
    std::cout << "Successfully set high speed ethernet! Change port and restart the robot." << std::endl;
}

void RMCommand::ReadL(Eigen::Matrix<double,6,1>& pose){
    command_msg.clear();
    command_msg["command"] = "get_current_arm_state";
    cmd_str = command_msg.dump() + "\r\n";
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    } else {
        memset(recv_msg, 0, 1000); 
        recv_times = 0;
        while (recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) 
            recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't receive message! " << std::endl;
            std::exit(0);
        } else {
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message ReadL." << return_msg.dump() << std::endl;
            } else {
                // 假设返回格式为 return_msg["arm_state"]["pose"]
                auto &p = return_msg["arm_state"]["pose"];
                // pose[0]: x, pose[1]: y, pose[2]: z (0.001mm)
                for(int i = 0; i < 3; ++i)
                    pose(i) = p[i].get<double>() / 1e6;  // 0.001mm -> m
                // pose[3]: rx, pose[4]: ry, pose[5]: rz (0.001rad)
                for(int i = 3; i < 6; ++i)
                    pose(i) = p[i].get<double>() / 1000.0; // 0.001rad -> rad
            }
        }
    }
}

void RMCommand::ReadJ(Eigen::Matrix<double,7,1>& joints){
    command_msg.clear();
    command_msg["command"] = "get_joint_degree";
    if (SendAndParse(*this, command_msg, "ReadJ")) {
        if (!return_msg.contains("joint") || !return_msg["joint"].is_array()) {
            std::cout << "WARNING! ReadJ return has no joint array: "
                      << return_msg.dump() << std::endl;
        } else {
            last_joint_count = return_msg["joint"].size();
            if (last_joint_count != cmd_joints.size()) {
                std::cout << "WARNING! ReadJ received " << last_joint_count
                          << " joints; expected 7 for RM75." << std::endl;
            }
            int n = std::min<int>(last_joint_count, cmd_joints.size());
            for (int i = 0; i < n; i++){
                cmd_joints[i] = return_msg["joint"][i].get<double>();
            }
        }
    }
    joints = cmd_joints * kMilliDegreeToRad;
}

void RMCommand::ReadArmState(Eigen::Matrix<double,7,1>& joints,
                             Eigen::Matrix<double,6,1>& pose,
                             int& arm_err_out,
                             int& sys_err_out){
    command_msg.clear();
    command_msg["command"] = "get_current_arm_state";
    if (SendAndParse(*this, command_msg, "ReadArmState")) {
        if (!return_msg.contains("arm_state") || !return_msg["arm_state"].is_object()) {
            std::cout << "WARNING! ReadArmState return has no arm_state object: "
                      << return_msg.dump() << std::endl;
        } else {
            auto& state = return_msg["arm_state"];
            arm_err = state.value("arm_err", arm_err);
            sys_err = state.value("sys_err", sys_err);

            if (state.contains("joint") && state["joint"].is_array()) {
                last_joint_count = state["joint"].size();
                if (last_joint_count != cmd_joints.size()) {
                    std::cout << "WARNING! ReadArmState received " << last_joint_count
                              << " joints; expected 7 for RM75." << std::endl;
                }
                int n = std::min<int>(last_joint_count, cmd_joints.size());
                for (int i = 0; i < n; i++) {
                    cmd_joints[i] = state["joint"][i].get<double>();
                }
            }

            if (state.contains("pose") && state["pose"].is_array()) {
                auto& p = state["pose"];
                int n = std::min<int>(p.size(), cmd_pose.size());
                for (int i = 0; i < n; i++) {
                    cmd_pose[i] = p[i].get<double>();
                }
            }
        }
    }

    joints = cmd_joints * kMilliDegreeToRad;
    for (int i = 0; i < pose.size(); i++) {
        pose[i] = (i < 3) ? cmd_pose[i] / 1000000.0 : cmd_pose[i] / 1000.0;
    }
    arm_err_out = arm_err;
    sys_err_out = sys_err;
}


void RMCommand::MoveJ(Eigen::Matrix<double,7,1>& joints, int velo){
    command_msg.clear();
    command_msg["command"] = "movej";
    FillJointCommand(command_msg, joints);
    command_msg["v"] = velo;
    command_msg["r"] = 0;
    cmd_str = command_msg.dump()+"\r\n";
    std::cout << cmd_str << std::endl;
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    }else{
        memset(recv_msg, 0, 1000); recv_times=0;
        while (recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't recive message! " << std::endl;
            std::exit(0);
        }else{
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
            }else{
                if(return_msg["trajectory_state"].get<bool>()){
                    std::cout << "MoveJ OK!\t" << return_msg.dump() << std::endl;
                }else{
                    std::cout << "ERROR! MoveJ False!\t" << return_msg.dump() << std::endl;
                    std::exit(0);
                }
            }
        }
    }
}


void RMCommand::MoveL(Eigen::Matrix<double,6,1>& pose, int velo){
    command_msg.clear();
    command_msg["command"] = "movel";
    for(int i = 0; i < pose.size(); i++){
        if(i < 3){
            command_msg["pose"][i] = int(1000*1000*pose[i]); // 0.001 mm
        }else{
            command_msg["pose"][i] = int(1000*pose[i]); // 0.001 rad
        }
    }
    command_msg["v"] = velo;
    command_msg["r"] = 0;
    cmd_str = command_msg.dump()+"\r\n";
    std::cout << cmd_str << std::endl;
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    }else{
        memset(recv_msg, 0, 1000); recv_times=0;
        while (recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't recive message! " << std::endl;
            std::exit(0);
        }else{
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
            }else{
                if(return_msg["trajectory_state"].get<bool>()){
                    std::cout << "MoveL OK!\t" << return_msg.dump() << std::endl;
                }else{
                    std::cout << "ERROR! MoveL False!\t" << return_msg.dump() << std::endl;
                    std::exit(0);
                }
            }
        }
    }
}


void RMCommand::MoveJP(Eigen::Matrix<double,6,1>& pose, int velo){
    command_msg.clear();
    command_msg["command"] = "movej_p";
    for(int i = 0; i < pose.size(); i++){
        if(i < 3){
            command_msg["pose"][i] = int(1000*1000*pose[i]); // 0.001 mm
        }else{
            command_msg["pose"][i] = int(1000*pose[i]); // 0.001 rad
        }
    }
    command_msg["v"] = velo;
    command_msg["r"] = 0;
    cmd_str = command_msg.dump()+"\r\n";
    std::cout << cmd_str << std::endl;
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    }else{
        memset(recv_msg, 0, 1000); recv_times=0;
        while (recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't recive message! " << std::endl;
            std::exit(0);
        }else{
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
            }else{
                if(return_msg["trajectory_state"].get<bool>()){
                    std::cout << "MoveJ_P OK!\t" << return_msg.dump() << std::endl;
                }else{
                    std::cout << "ERROR! MoveJ_P False!\t" << return_msg.dump() << std::endl;
                    std::exit(0);
                }
            }
        }
    }
}

void RMCommand::ServoJ(Eigen::Matrix<double,7,1>& joints, bool follow){
    command_msg.clear();
    command_msg["command"] = "movej_canfd";
    FillJointCommand(command_msg, joints);
    command_msg["follow"] = follow;
    cmd_str = command_msg.dump()+"\r\n";
    memset(send_msg, 0, 1000);
    strcpy(send_msg, cmd_str.c_str());
    if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
        std::cout << "ERROR! Can't send message! " << std::endl;
        std::exit(0);
    }else{
        memset(recv_msg, 0, 1000); recv_times=0;
        while (recv(rlm_socket, recv_msg, 1000, 0) < 10 && recv_times < 3) recv_times++;
        if(recv_times == 3){
            std::cout << "ERROR! Can't recive message! " << std::endl;
            std::exit(0);
        }else{
            return_msg.clear();
            return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
            if (return_msg.is_discarded()){
                std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
            }else{
                if(return_msg["arm_err"].get<int>() != 0){
                    std::cout << "WARNING! ServoJ False!\t" << return_msg.dump() << std::endl;
                }
            }
        }
    }
}


// Solution 1
// void RMCommand::MoveL(Eigen::Matrix<double,6,1>& pose){
//     command_msg.clear();
//     command_msg["command"] = "movel";
//     for(int i = 0; i < pose.size(); i++){
//         if(i < 3){
//             command_msg["pose"][i] = int(1000*1000*pose[i]);
//         }else{
//             command_msg["pose"][i] = int(1000*pose[i]);
//         }
//     }
//     command_msg["v"] = 20;
//     command_msg["r"] = 0;
//     cmd_str = command_msg.dump()+"\r\n";
//     std::cout << cmd_str << std::endl;
//     memset(send_msg, 0, 1000);
//     strcpy(send_msg, cmd_str.c_str());
//     if(send(rlm_socket, send_msg, strlen(send_msg), 0) < 0){
//         std::cout << "ERROR! Can't send message! " << std::endl;
//         std::exit(0);
//     }else{
//         memset(recv_msg, 0, 1000);
//         if(recv(rlm_socket, recv_msg, 1000, 0) < 0){
//             std::cout << "ERROR! Can't recive message! " << std::endl;
//             std::exit(0);
//         }else{
//             return_msg.clear();
//             return_msg = nlohmann::json::parse(recv_msg, nullptr, false);
//             if (return_msg.is_discarded()){
//                 std::cout << "WARNING! Missing a return message." << return_msg.dump() << std::endl;
//             }else{
//                 if(return_msg["trajectory_state"].get<bool>()){
//                     std::cout << "MoveL OK!\t" << return_msg.dump() << std::endl;
//                 }else{
//                     std::cout << "ERROR! MoveL False!\t" << return_msg.dump() << std::endl;
//                     std::exit(0);
//                 }
//             }
//         }
//     }
// }
