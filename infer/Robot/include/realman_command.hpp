#include <memory>
#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <json.hpp>
#include <sys/shm.h>
#include <sys/types.h>
#include <Eigen/Dense>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

class RMCommand
{
public:
    int rlm_port, rlm_socket, recv_times;
    char rlm_ip[16], send_msg[1000], recv_msg[1000];
    std::string cmd_str;
    nlohmann::json command_msg, return_msg;
    // Realman robot may not return message or return a "\n", which is valid json. 
    // To prevent this happens when readj and return a wrong joint, store last time readj joint
    Eigen::Matrix<double, 7, 1> cmd_joints;
    Eigen::Matrix<double, 6, 1> cmd_pose;
    int last_joint_count;
    int arm_err, sys_err;

public:
    RMCommand();
    void ConnectTCPSocket();
    void SetHighSpeedEth();
    void ReadJ(Eigen::Matrix<double,7,1>& joints);
    void ReadArmState(Eigen::Matrix<double,7,1>& joints, Eigen::Matrix<double,6,1>& pose,
                      int& arm_err_out, int& sys_err_out);
    void MoveJ(Eigen::Matrix<double,7,1>& joints, int velo);
    void MoveL(Eigen::Matrix<double,6,1>& pose, int velo);
    void MoveJP(Eigen::Matrix<double,6,1>& pose, int velo);
    void ServoJ(Eigen::Matrix<double,7,1>& joints, bool follow);
    void ReadL(Eigen::Matrix<double,6,1>& pose);
};
