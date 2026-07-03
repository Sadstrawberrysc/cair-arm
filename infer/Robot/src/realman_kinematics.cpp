#include <realman_kinematics.hpp>

RMKinematics::RMKinematics(){
    //DHparametersd3=0
    lsb=0.2405;
    lse=0.2560;
    lew=0.2100;
    lwt=0.1439;
}

void RMKinematics::GetKinematics(Eigen::Matrix4d& kinematics, Eigen::Matrix<double,6,1>& joints){
    double x1, x2, x3, x4, x5, x6;
    x1=joints[0];
    x2=joints[1];
    x3=joints[2];
    x4=joints[3];
    x5=joints[4];
    x6=joints[5];

    kinematics.fill(0);
    kinematics<<cos(x6)*(cos(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-sin(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2)))+sin(x6)*(cos(x4)*sin(x1)-sin(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))),
    cos(x6)*(cos(x4)*sin(x1)-sin(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-sin(x6)*(cos(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-sin(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))),
    sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2)),
    lew*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))+lwt*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lse*cos(x1)*cos(x2+M_PI/2),
    -cos(x6)*(sin(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))+cos(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))))-sin(x6)*(sin(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))+cos(x1)*cos(x4)),
    sin(x6)*(sin(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))+cos(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))))-cos(x6)*(sin(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))+cos(x1)*cos(x4)),
    cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))),
    lew*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-lwt*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)))+lse*cos(x2+M_PI/2)*sin(x1),
    cos(x6)*(sin(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))+cos(x4)*cos(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))-sin(x4)*sin(x6)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)),
    -sin(x6)*(sin(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))+cos(x4)*cos(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))-cos(x6)*sin(x4)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)),
    cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2))-cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2)),
    lsb-lwt*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))-lew*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))+lse*sin(x2+M_PI/2),
    0, 0, 0, 1;

    // std::cout << kinematics << std::endl;
}

void RMKinematics::GetJacobian(Eigen::Matrix<double,6,6>& jacobian, Eigen::Matrix<double,6,1>& joints){
    double x1, x2, x3, x4, x5, x6;
    x1=joints[0];
    x2=joints[1];
    x3=joints[2];
    x4=joints[3];
    x5=joints[4];
    x6=joints[5];

    jacobian.fill(0);
    jacobian << lwt*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)))-lew*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-lse*cos(x2+M_PI/2)*sin(x1),
        cos(x1)*(lwt*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lew*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-lse*sin(x2+M_PI/2)),
        cos(x1)*(lwt*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lew*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))),
        (lew*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-lwt*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))))*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-(lwt*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lew*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2)))*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)),
        lwt*(sin(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))+cos(x1)*cos(x4))*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))-lwt*sin(x4)*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)))*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)),
        0,
        lew*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))+lwt*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lse*cos(x1)*cos(x2+M_PI/2),
        sin(x1)*(lwt*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lew*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-lse*sin(x2+M_PI/2)),
        sin(x1)*(lwt*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lew*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))),
        (lwt*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lew*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2)))*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))-(lew*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))+lwt*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))))*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2)),
        lwt*(cos(x4)*sin(x1)-sin(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))*(cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)))-lwt*sin(x4)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2))*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))),
        0,
        0,
        sin(x1)*(lew*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-lwt*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)))+lse*cos(x2+M_PI/2)*sin(x1))+cos(x1)*(lew*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))+lwt*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2)))+lse*cos(x1)*cos(x2+M_PI/2)),
        cos(x1)*(lew*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))+lwt*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))))+sin(x1)*(lew*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-lwt*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)))),
        (lew*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-lwt*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))))*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))-(lew*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))+lwt*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2))))*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)),
        lwt*(sin(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))+cos(x1)*cos(x4))*(sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2)))-lwt*(sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))-cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2)))*(cos(x4)*sin(x1)-sin(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))),
        0,
        0,
        sin(x1),
        sin(x1),
        cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2),
        cos(x4)*sin(x1)-sin(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)),
        sin(x5)*(sin(x1)*sin(x4)+cos(x4)*(cos(x1)*cos(x2+M_PI/2)*cos(x3+M_PI/2)-cos(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2)))+cos(x5)*(cos(x1)*cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x1)*cos(x3+M_PI/2)*sin(x2+M_PI/2)),
        0,
        -cos(x1),
        -cos(x1),
        cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2),
        -sin(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))-cos(x1)*cos(x4),
        cos(x5)*(cos(x2+M_PI/2)*sin(x1)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x1)*sin(x2+M_PI/2))-sin(x5)*(cos(x1)*sin(x4)-cos(x4)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)*sin(x1)-sin(x1)*sin(x2+M_PI/2)*sin(x3+M_PI/2))),
        1,
        0,
        0,
        sin(x2+M_PI/2)*sin(x3+M_PI/2)-cos(x2+M_PI/2)*cos(x3+M_PI/2),
        -sin(x4)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2)),
        cos(x4)*sin(x5)*(cos(x2+M_PI/2)*sin(x3+M_PI/2)+cos(x3+M_PI/2)*sin(x2+M_PI/2))-cos(x5)*(cos(x2+M_PI/2)*cos(x3+M_PI/2)-sin(x2+M_PI/2)*sin(x3+M_PI/2));

    // std::cout << jacobian << std::endl;
}

void RMKinematics::GetNextJoints(Eigen::Matrix<double,6,1>& next_joints, Eigen::Matrix<double,6,1>& cur_joints, Eigen::Matrix4d& next_kinematics){
    Eigen::Matrix4d cur_kinematics;
    Eigen::Matrix<double,6,6> cur_jacobian;
    Eigen::Matrix<double,6,1> error_kinematics;

    // Calculate the cartesian position error
    GetJacobian(cur_jacobian,cur_joints);
    GetKinematics(cur_kinematics,cur_joints);
    Eigen::AngleAxisd rotation_axis;
    Eigen::Matrix3d cur_rot,next_rot,change_rot;
    cur_rot = cur_kinematics.block(0,0,3,3);
    next_rot = next_kinematics.block(0,0,3,3);
    change_rot = cur_rot*next_rot.inverse();
    rotation_axis.fromRotationMatrix(change_rot);
    for (int i=0; i < 6; i++){
        if (i<3){
            error_kinematics[i] = next_kinematics(i,3)-cur_kinematics(i,3);
        }else{
            error_kinematics[i] = rotation_axis.axis()(i-3)*rotation_axis.angle();
        }
    }
    // std::cout << "error_kinematics = " << error_kinematics << std::endl;

    // Using damped joints method to get next joints
    double lamba = 0.0005;
    Eigen::Matrix<double,6,6> jacobian_pseudo_inverse;
    jacobian_pseudo_inverse = (cur_jacobian.transpose())*((cur_jacobian*cur_jacobian.transpose() + lamba*Eigen::MatrixXd::Identity(6,6)).inverse());
    next_joints = cur_joints + (jacobian_pseudo_inverse*error_kinematics);

    // Joints limitation
    if (next_joints[0] >  M_PI) next_joints[0] =  M_PI;
    if (next_joints[0] < -M_PI) next_joints[0] = -M_PI;

    if (next_joints[1] >  130*M_PI/180) next_joints[1] =  130*M_PI/180;
    if (next_joints[1] < -130*M_PI/180) next_joints[1] = -130*M_PI/180;

    if (next_joints[2] >  135*M_PI/180) next_joints[2] =  135*M_PI/180;
    if (next_joints[2] < -135*M_PI/180) next_joints[2] = -135*M_PI/180;
    
    if (next_joints[3] >  M_PI) next_joints[3] =  M_PI;
    if (next_joints[3] < -M_PI) next_joints[3] = -M_PI;

    if (next_joints[4] >  128*M_PI/180) next_joints[4] =  128*M_PI/180;
    if (next_joints[4] < -128*M_PI/180) next_joints[4] = -128*M_PI/180;

    if (next_joints[5] >  2*M_PI) next_joints[5] =  2*M_PI;
    if (next_joints[5] < -2*M_PI) next_joints[5] = -2*M_PI;
    
}