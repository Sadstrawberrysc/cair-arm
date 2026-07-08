#include <realman_command.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <sys/select.h>

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

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(rm.rlm_socket, &read_fds);
    timeval timeout;
    timeout.tv_sec = 3;
    timeout.tv_usec = 0;

    int ready = select(rm.rlm_socket + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        std::cout << "ERROR! Can't receive message in " << label << "!" << std::endl;
        std::exit(1);
    }

    memset(rm.recv_msg, 0, 1000);
    int received = recv(rm.rlm_socket, rm.recv_msg, 999, 0);
    if (received <= 0) {
        std::cout << "ERROR! Can't receive message in " << label << "!" << std::endl;
        std::exit(1);
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

bool ReceiveAndParse(RMCommand& rm, const char* label, int timeout_ms) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(rm.rlm_socket, &read_fds);
    timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ready = select(rm.rlm_socket + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready < 0) {
        std::cout << "ERROR! select failed in " << label << ": "
                  << std::strerror(errno) << std::endl;
        return false;
    }
    if (ready == 0) {
        return false;
    }

    memset(rm.recv_msg, 0, 1000);
    int received = recv(rm.rlm_socket, rm.recv_msg, 999, 0);
    if (received <= 0) {
        std::cout << "ERROR! recv failed in " << label << std::endl;
        return false;
    }

    rm.return_msg.clear();
    rm.return_msg = nlohmann::json::parse(rm.recv_msg, nullptr, false);
    if (rm.return_msg.is_discarded()) {
        std::cout << "WARNING! Invalid JSON in " << label << ": "
                  << rm.recv_msg << std::endl;
        return false;
    }
    return true;
}

void SendRequest(RMCommand& rm, const nlohmann::json& request, const char* label) {
    rm.cmd_str = request.dump() + "\r\n";
    memset(rm.send_msg, 0, 1000);
    strcpy(rm.send_msg, rm.cmd_str.c_str());

    if (send(rm.rlm_socket, rm.send_msg, strlen(rm.send_msg), 0) < 0) {
        std::cout << "ERROR! Can't send message in " << label << "!" << std::endl;
        std::exit(1);
    }
}

template <int JointCount>
void FillJointCommand(nlohmann::json& msg, const Eigen::Matrix<double, JointCount, 1>& joints) {
    for (int i = 0; i < joints.size(); i++) {
        msg["joint"][i] = int(kRadToMilliDegree * joints[i]);
    }
}

void WaitTrajectoryResponse(RMCommand& rm, const char* label) {
    bool saw_receive_ack = false;
    constexpr int kMaxMessages = 30;

    for (int i = 0; i < kMaxMessages; ++i) {
        if (!ReceiveAndParse(rm, label, 1000)) {
            continue;
        }

        const auto& response = rm.return_msg;
        if (!rm.quiet) {
            std::cout << label << " response:\t" << response.dump() << std::endl;
        }

        if (response.contains("arm_err") && response["arm_err"].is_number_integer()
            && response["arm_err"].get<int>() != 0) {
            std::cout << "ERROR! " << label << " arm_err is non-zero!" << std::endl;
            std::exit(1);
        }

        if (response.contains("receive_state")) {
            if (response["receive_state"].is_boolean() && !response["receive_state"].get<bool>()) {
                std::cout << "ERROR! " << label << " receive_state is false!" << std::endl;
                std::exit(1);
            }
            saw_receive_ack = true;
            continue;
        }

        if (response.contains("trajectory_state")) {
            const auto& trajectory_state = response["trajectory_state"];
            if (trajectory_state.is_boolean()) {
                if (trajectory_state.get<bool>()) {
                    if (!rm.quiet) {
                        std::cout << label << " OK!" << std::endl;
                    }
                    return;
                }
                std::cout << "ERROR! " << label << " trajectory_state is false!" << std::endl;
                std::exit(1);
            } else {
                std::cout << "WARNING! " << label
                          << " trajectory_state is not a boolean; keep waiting."
                          << std::endl;
            }
        }
    }

    std::cout << "ERROR! " << label
              << " did not receive trajectory completion after "
              << (saw_receive_ack ? "ack." : "sending command.")
              << std::endl;
    std::exit(1);
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
    quiet = false;
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
        if (!quiet) {
            std::cout << "Robot connect!" << std::endl;
        }
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
    command_msg["ip"] = "192.168.50.254";
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
    SendRequest(*this, command_msg, "ReadArmState");
    for (int attempt = 0; attempt < 6; ++attempt) {
        if (!ReceiveAndParse(*this, "ReadArmState", 1000)) {
            continue;
        }
        if (!return_msg.contains("arm_state") || !return_msg["arm_state"].is_object()) {
            std::cout << "WARNING! ReadArmState return has no arm_state object: "
                      << return_msg.dump() << std::endl;
            continue;
        }

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
        break;
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
    if (!quiet) {
        std::cout << cmd_str << std::endl;
    }
    SendRequest(*this, command_msg, "MoveJ");
    WaitTrajectoryResponse(*this, "MoveJ");
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
    if (!quiet) {
        std::cout << cmd_str << std::endl;
    }
    SendRequest(*this, command_msg, "MoveL");
    WaitTrajectoryResponse(*this, "MoveL");
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
    if (!quiet) {
        std::cout << cmd_str << std::endl;
    }
    SendRequest(*this, command_msg, "MoveJ_P");
    WaitTrajectoryResponse(*this, "MoveJ_P");
}

void RMCommand::ServoJ(Eigen::Matrix<double,7,1>& joints, bool follow){
    command_msg.clear();
    command_msg["command"] = "movej_canfd";
    FillJointCommand(command_msg, joints);
    command_msg["follow"] = follow;

    if (!quiet) {
        std::cout << command_msg.dump() << std::endl;
    }
    SendRequest(*this, command_msg, "ServoJ");
    if (!ReceiveAndParse(*this, "ServoJ", 0)) {
        return;
    }

    if (!quiet) {
        std::cout << "ServoJ response:\t" << return_msg.dump() << std::endl;
    }

    if (return_msg.contains("arm_err") && return_msg["arm_err"].is_number_integer()) {
        if (return_msg["arm_err"].get<int>() != 0) {
            std::cout << "WARNING! ServoJ arm_err is non-zero:\t"
                      << return_msg.dump() << std::endl;
        }
        return;
    }

    if (return_msg.contains("receive_state") && return_msg["receive_state"].is_boolean()) {
        if (!return_msg["receive_state"].get<bool>()) {
            std::cout << "WARNING! ServoJ receive_state is false:\t"
                      << return_msg.dump() << std::endl;
        }
        return;
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
