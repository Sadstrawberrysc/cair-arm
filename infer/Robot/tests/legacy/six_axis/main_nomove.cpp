#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <ctime>
#include <sys/time.h>

// Force sensor package
#include <force_sensor.hpp>

// Intrinsic contact sensing package
#include <contact_sensing.hpp>

// Realman robot package
#include <robot_control.hpp>
#include <realman_command.hpp>
#include <realman_kinematics.hpp>

// Redis and JSON packages
#include <hiredis/hiredis.h>
#include <json.hpp>


// Get current time as formatted string
thread_local char __timebuf[64] = {0x00};
const char *curtime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *lt = localtime(&tv.tv_sec);
    snprintf(__timebuf, sizeof(__timebuf) - 1,
            "%d-%02d-%02d %02d:%02d:%02d.%03d", 
            lt->tm_year + 1900,
            lt->tm_mon + 1,
            lt->tm_mday,
            lt->tm_hour,
            lt->tm_min,
            lt->tm_sec,
            (int)(tv.tv_usec / 1000));
    return __timebuf;
}

// Get timestamp string for file naming
std::string getTimeStampMain() {
    // Get local time
    std::time_t now = std::time(nullptr);
    
    // Format time as YYYYMMDDHHMMSS
    std::tm* localTime = std::localtime(&now);
    std::stringstream ss;
    ss << 1900 + localTime->tm_year;
    ss << std::setfill('0') << std::setw(2) << (localTime->tm_mon + 1);
    ss << std::setfill('0') << std::setw(2) << localTime->tm_mday;
    ss << std::setfill('0') << std::setw(2) << localTime->tm_hour;
    ss << std::setfill('0') << std::setw(2) << localTime->tm_min;
    ss << std::setfill('0') << std::setw(2) << localTime->tm_sec;
    
    return ss.str();
}

// Force sensor reading thread - reads F/T data as [fx,fy,fz,tx,ty,tz]
void ForceSensor(double sensor[], std::atomic<bool>* scan_flag_ptr) {
    CLinuxSerial com(0, 115200);
    if (!com.IsOpen()) {
        std::cerr << "ERROR: Failed to open serial port for force sensor." << std::endl;
        return;
    }
    while(true){
        com.ProcessSensorData();
        for (int index = 0; index < 6; index++){
            sensor[index] = com.sensor[index];
        }
        usleep(1000);
    }
    return;
}

// Namespace and constants
using json = nlohmann::json;
const char* COMMAND_KEY = "robot:command";
const char* STATUS_KEY = "robot:status";

// Atomic variables for thread-safe communication
std::atomic<bool> new_command_received{false};
std::atomic<double> target_y{0.0};
std::atomic<double> target_rz{0.0};
std::atomic<bool> terminate_received{false};
std::atomic<int> phase_index{0};
std::atomic<int> action_state{0};
std::atomic<bool> scan_flag{true};

void redis_communication_worker() {
    redisContext *context = redisConnect("127.0.0.1", 7777);
    if (context == NULL || context->err) {
        std::cerr << "[Redis Thread] Connection error: " << context->errstr << std::endl;
        if (context) redisFree(context);
        return;
    }
    std::cout << "[Redis Thread] Successfully connected to Redis!" << std::endl;

    // Subscribe to command channel
    redisReply *reply = (redisReply*)redisCommand(context, "SUBSCRIBE robot:command:channel");
    freeReplyObject(reply);
    std::cout << "[Redis Thread] Subscribed to robot:command:channel" << std::endl;

    while (true) {
        // Wait for published messages
        redisReply *reply;
        if (redisGetReply(context, (void**)&reply) == REDIS_OK) {
            if (reply->type == REDIS_REPLY_ARRAY && reply->elements >= 3) {
                if (strcmp(reply->element[0]->str, "message") == 0) {
                    std::string channel = reply->element[1]->str;
                    // Fix typo and read message payload
                    std::string msg = reply->element[2]->str;
                    
                    // Parse JSON and update atomic variables
                    try {
                        auto command_data = json::parse(msg);
                        
                        // Atomically store new values
                        target_y.store(command_data.at("parameters").at("y"));
                        target_rz.store(command_data.at("parameters").at("rz"));
                        int terminate = command_data.at("terminate");
                        action_state.store(command_data.at("action_state"));
                        // Receive phase_idx (top-level) with default -1 if absent
                        int phase = -1;
                        if (command_data.contains("phase_idx")) {
                            phase = command_data.at("phase_idx");
                        }
                        phase_index.store(phase);
                        
                        terminate_received.store(terminate); // Invert logic: true means stop
                        
                        // Set flag to notify main thread
                        new_command_received.store(true);
                        
                        std::cout << "[Redis Thread] Received command - y: " << target_y.load() 
                                  << ", rz: " << target_rz.load() 
                                  << ", phase_idx: " << phase_index.load()
                                  << ", terminate: " << terminate << std::endl;

                    } catch (const std::exception& e) {
                        std::cerr << "[Redis Thread] JSON parsing error: " << e.what() << std::endl;
                    }
                }
            }
            freeReplyObject(reply);
        }
        
        usleep(10000); // 10ms
    }

    // Unsubscribe before closing
    reply = (redisReply*)redisCommand(context, "UNSUBSCRIBE robot:command:channel");
    freeReplyObject(reply);
    
    redisFree(context);
    std::cout << "[Redis Thread] Thread terminated." << std::endl;
}


int main() {   

    // Force sensor data array
    double sensor[6];

    // Connect to Redis for main thread
    redisContext *main_redis_context = redisConnect("127.0.0.1", 7777);
    if (main_redis_context == NULL || main_redis_context->err) {
        std::cerr << "[Main Thread] Connection error: " << main_redis_context->errstr << std::endl;
        if(main_redis_context) redisFree(main_redis_context);
        return -1;
    }
    std::cout << "[Main Thread] Successfully connected to Redis!" << std::endl;

    // Start Redis communication thread
    std::thread redis_thread(redis_communication_worker);
    redis_thread.detach();

    // Start force sensor reading thread
    std::thread task0(ForceSensor, sensor, &scan_flag);
    task0.detach();

    // Load probe STL file for contact sensing
    ContactLocation ICS;
    ICS.LoadSTL("../model/Lprobe-IFS.STL");

    // Initialize robot control objects
    RobotCtr Rctr;
    RMCommand RMcmd;
    RMKinematics RMkine;

    // Connect to Realman robot
    RMcmd.ConnectTCPSocket();

    // Load gravity compensation parameters
    std::ifstream gravity_file("../data/gc_parameters.txt", std::ios::in);
    if (!gravity_file) {
        std::cout << "Error! Cannot load gravity compensation parameters." << std::endl;
        std::exit(0);
    }
    for (int i = 0; i < 12; i++) {
        if (i < 6) {
            gravity_file >> Rctr.gravity_error(i, 0);
        } else {
            gravity_file >> Rctr.tool_center(i - 6, 0);
        }
    }
    gravity_file.close();
    std::cout << "Gravity error = " << Rctr.gravity_error.transpose() << std::endl;
    std::cout << "Tool center = " << Rctr.tool_center.transpose() << "\n" << std::endl;
    
    // Initialize robot position
    Eigen::Matrix<double, 6, 1> start_joints;


    RMcmd.ReadJ(start_joints);
    std::cout << "Current Joints (rad) = " << start_joints.transpose() << std::endl;

    // start_joints << 0.131668,   0.013631,    1.77919, -0.0741067,    1.13162,  -0.260752;
    // // start_joints << -17.594*M_PI/180, 50.236*M_PI/180, 84.27*M_PI/180, 8.291*M_PI/180, -41.958*M_PI/180, 153.106*M_PI/180;
    
    // RMcmd.MoveJ(start_joints, 20);
    sleep(2);

    std::cout << "+++++++++++++++++++++ Initialization Complete +++++++++++++++++++++\n" << std::endl;
    
    std::cout << "+++++++++++++++++++++ Scanning Started +++++++++++++++++++++\n" << std::endl;
    while (true)
    {
     if(new_command_received.load())
        {
            break;
        }   
        sleep(1);
    }
    
    // start_joints << -0.784752,  0.639942,   2.31055, -0.224798,   -1.4725, -0.104807;
    // RMcmd.MoveJ(start_joints, 20);
    sleep(2);


    // Robot motion variables
    Eigen::Matrix<double, 6, 1> cur_joints, next_joints;
    Eigen::Matrix4d cur_kinematics, next_kinematics;

    // Contact sensing variables
    Eigen::Vector3d contact_point;

    // Control timing variables
    std::chrono::duration<double, std::milli> fp_ms_ctr, fp_ms_ics;
    auto ctr_cycle_start = std::chrono::high_resolution_clock::now();
    auto ctr_cycle_end = std::chrono::high_resolution_clock::now();
    
    // Calibrate force sensor zero
    RMcmd.ReadJ(cur_joints);
    RMkine.GetKinematics(cur_kinematics, cur_joints);
    Rctr.cur_rot = cur_kinematics.block(0, 0, 3, 3);
    Rctr.cur_pos = cur_kinematics.block(0, 3, 3, 1);
    // Fix translation direction to the initial orientation
    Rctr.ref_rot = Rctr.cur_rot;
    Eigen::Matrix<double, 6, 1> tmp_sensor_ft;
    tmp_sensor_ft.setZero();
    
    // Average 100 readings for zero calibration
    for(int i = 0; i < 100; i++) {
        Rctr.sensor_ft << sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5];
        tmp_sensor_ft += Rctr.sensor_ft;
        usleep(10000);
    }
    Rctr.sensor_ft = tmp_sensor_ft / 100;
    Rctr.updateForceSensorZero();
    std::cout << "Force sensor zero calibration complete" << std::endl;
    sleep(1);

    // Scan control variables
    Eigen::Matrix4d base_kinematics, delta_kinematics;
    base_kinematics.setZero();

    bool flag_moving = true;
    bool flag_sending = false;
    bool flag_rotation = false;
    int flag_rotation_count = 0;

    double local_target_y = 0.0;
    double local_target_rz = 0.0;
    bool local_terminate = false;
    int local_phase_idx = -1;
    int local_action_state = 0;

    // Save command and status to file
    std::string timestamp = getTimeStampMain();
    std::string command_filename = "../data/command_" + timestamp + ".txt";
    std::ofstream command_file(command_filename, std::ios::out);
    if (!command_file) {
        std::cout << "Error! Cannot create command file." << std::endl;
        std::exit(0);
    }


    // Main control loop
    while (Rctr.flag_scaning == true) {

        Rctr.desired_force = -3;

        ctr_cycle_start = std::chrono::high_resolution_clock::now();
        // Update robot state
        RMcmd.ReadJ(cur_joints);
        RMkine.GetKinematics(cur_kinematics, cur_joints);
        Rctr.cur_rot = cur_kinematics.block(0, 0, 3, 3);
        Rctr.cur_pos = cur_kinematics.block(0, 3, 3, 1);

        // Apply gravity compensation
        Rctr.sensor_ft << sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5];
        Rctr.calRealFT();
        std::cout << "Contact Force = " << Rctr.real_ft[2] << std::endl;
        
        // Safety check for sensor range
        if (fabs(Rctr.real_ft[0]) > 20) {
            std::cout << "Error! Z-axis force exceeds sensor range." << std::endl;
            std::exit(0);
        }
        if (fabs(Rctr.real_ft[1]) > 20) {
            std::cout << "Error! Z-axis force exceeds sensor range." << std::endl;
            std::exit(0);
        }

        if (fabs(Rctr.real_ft[2]) > 50) {
            std::cout << "Error! Z-axis force exceeds sensor range." << std::endl;
            std::exit(0);
        }

        // Process new command if received
        if (new_command_received.load()) {
            std::cout << "[Main Thread] New command detected, starting execution." << std::endl;
            
            // Atomically read target values
            local_target_y = target_y.load();
            local_target_rz = target_rz.load();
            local_terminate = terminate_received.load();
            local_phase_idx = phase_index.load();
            local_action_state = action_state.load();

            std::cout << "Target Y = " << local_target_y << std::endl;
            std::cout << "Target RZ = " << local_target_rz << std::endl;
            std::cout << "Phase Idx = " << local_phase_idx << std::endl;
            std::cout << "Rotation count = " << flag_rotation_count << std::endl;
            std::cout <<"Terminate = " << local_terminate << std::endl;
            std::cout << "Action State = " << local_action_state << std::endl;

            // check for terminate command and force
            if((local_terminate == 1 && fabs(Rctr.real_ft[2])>2)){
                Rctr.model_y_pos = -0.0;
                if(std::fabs(local_target_rz)<13){
                    local_target_rz = 0;
                    flag_rotation = false;
                    std::cout<<"###Terminate with small RZ adjustment.#####" << std::endl;
                    Rctr.flag_scaning = false;
                }
                else
                {
                    local_target_rz = local_target_rz*0.5;
            
                }
                    std::cout << "###Terminate command received.#####" << std::endl;
            }
            flag_moving = true;
            new_command_received.store(false);
        }
        
        if (flag_moving &&fabs(Rctr.real_ft[2])>2 ) {
                if((local_phase_idx==0 || local_phase_idx==2) && fabs(local_target_y)<0.0008 && local_action_state==1 && flag_rotation==false)
                {
                    Rctr.model_x_pos = -0.003;
                    std::cout<< "Phase 0: Moving forward." << std::endl;
                }
                else
                {
                    Rctr.model_x_pos = 0.000;
                }

                if (fabs(local_target_y)<0.008 && local_phase_idx==1){
                    flag_rotation_count ++;
                }else{
                    flag_rotation_count = 0;
                }

                if (flag_rotation_count >= 20 && local_action_state==1) {
                    flag_rotation = true;
                    std::cout << "Rotation adjustment mode enabled." << std::endl;
                }
                
                if(fabs(Rctr.real_ft[2])>2)
                {
                    Rctr.model_y_pos = local_target_y;
                }else{
                    Rctr.model_y_pos = 0.0;
                }

                
                base_kinematics = cur_kinematics;

                // Log command to file
                command_file << "Command - Y: " << local_target_y 
                            << ", RZ: " << local_target_rz 
                            << ", PhaseIdx: " << local_phase_idx
                            << ", Time: " << curtime() << std::endl;

                delta_kinematics = base_kinematics.inverse() * cur_kinematics;

                std::cout << "Delta Y = " << delta_kinematics(1, 3) << std::endl;
                std::cout << "Delta RZ = " << std::atan2(delta_kinematics(1, 0), delta_kinematics(0, 0))*180/M_PI << std::endl;

                if (flag_rotation == true){
                    Rctr.model_y_pos = local_target_y*0.3;
                    std::cout<< "Rotation adjustment mode active." << std::endl;
                    Rctr.model_rz_pos = local_target_rz - std::atan2(delta_kinematics(1, 0), delta_kinematics(0, 0))*180/M_PI;
                }else{
                    Rctr.model_rz_pos = 0;
                    std::cout<<"**********ROTATION ADJUSTMENT INACTIVE**********"<<std::endl;
                }
        }
        else {
            // Reset movement flags
            flag_rotation = false;
            flag_rotation_count = 0;
            Rctr.model_x_pos = 0.0;
            Rctr.model_y_pos = 0.0;
            Rctr.model_rz_pos = 0.0;
            std::cout << "#######No active movement command.###########" << std::endl;
            flag_sending = true; // Set flag to send completion status
        }

        // Send completion status to Redis
        if (flag_sending == true) {
            json response_data;
            response_data["status"] = "success";
            response_data["message"] = "Movement completed";
            std::string response_msg = response_data.dump();
            
            // Publish to status channel
            redisReply* reply = (redisReply*)redisCommand(
                main_redis_context, 
                "PUBLISH robot:status:channel %s", 
                response_msg.c_str()
            );

            if (reply) {
                int subscribers = reply->integer;
                freeReplyObject(reply);
                std::cout << "[Main Thread] Status published to " << subscribers << " subscribers" << std::endl;
            } else {
                std::cerr << "[Main Thread] PUBLISH command failed: " << main_redis_context->errstr << std::endl;
            }
            
            flag_sending = false;
        }

        // Check for termination
        // if (terminate_received.load()) {
        //     std::cout << "[Main Thread] Termination command received, stopping scan." << std::endl;
        //     Rctr.flag_scaning = false;
        // }

        // Calculate contact point
        contact_point.setZero();
        if (fabs(Rctr.real_ft[2]) > 0.99) {
            ICS.calContactPoint(Rctr.real_ft, contact_point);
        }
        std::cout << "Contact point = " << contact_point.transpose() << std::endl;
        std::stringstream ics_ss;
        ics_ss << "[" 
           << std::fixed << std::setprecision(4) << contact_point(0) << ", " 
           << std::fixed << std::setprecision(4) << contact_point(1) << ", " 
           << std::fixed << std::setprecision(4) << contact_point(2) << ", "
           << std::fixed << std::setprecision(4) << Rctr.real_ft[2]
           << "]";
        std::string message = ics_ss.str();
        void *reply = redisCommand(main_redis_context, "PUBLISH %s %s", "sensor_data", message.c_str());
        
        if (reply == NULL) {
            std::cerr << "Error: Failed to execute PUBLISH command." << std::endl;
            // 可以在这里尝试重连逻辑
            break; 
        }
        
        freeReplyObject(reply);

        // Execute robot control
        Rctr.cur_contact_point = contact_point;
        Rctr.mainCtr();

        // Calculate next robot position
        next_kinematics << Rctr.next_rot, Rctr.next_pos, 0, 0, 0, 1;
        RMkine.GetNextJoints(next_joints, cur_joints, next_kinematics);
        
        // Safety check for joint velocity
        for(int i = 0; i < 6; i++) {
            if(fabs(next_joints[i] - cur_joints[i]) > 15*M_PI/180) {
                std::cout << "ERROR! Robot servo velocity too high!" << std::endl;
                std::exit(0);
            }
        }

        // Send servo command
        RMcmd.ServoJ(next_joints, false);
 
        // Wait for servo cycle time (50ms)
        while (true) {
            ctr_cycle_end = std::chrono::high_resolution_clock::now();
            fp_ms_ctr = ctr_cycle_end - ctr_cycle_start;
            if (fp_ms_ctr.count() > 20) {
                break;
            }
            usleep(10);
        }

        scan_flag.store(Rctr.flag_scaning);
        
        std::cout << std::endl;
    }
    command_file.close();
    std::cout << "+++++++++++++++++++++ Scanning Finished +++++++++++++++++++++\n" << std::endl;

    while (true)
    {   
        // Update robot state
        RMcmd.ReadJ(cur_joints);
        RMkine.GetKinematics(cur_kinematics, cur_joints);
        Rctr.cur_rot = cur_kinematics.block(0,0,3,3);
        Rctr.cur_pos = cur_kinematics.block(0,3,3,1);
        
        // Gravity compensation and kalman filter
        Rctr.sensor_ft << sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5];
        Rctr.calRealFT();
        std::cout << "Contact Force = " << Rctr.real_ft[2] << std::endl;
        if (fabs(Rctr.real_ft[2]) > 50){
            std::cout << "Error! Z-axis force beyond sensor range." << std::endl;
            std::exit(0);
        }

        // Contact sensing
        contact_point.setZero();
        if (fabs(Rctr.real_ft[2]) > 0.99) {
            ICS.calContactPoint(Rctr.real_ft, contact_point);
        }
        std::cout << "Contact point = " << contact_point.transpose() << std::endl;
        std::stringstream ics_ss;
        ics_ss << "[" 
           << std::fixed << std::setprecision(4) << contact_point(0) << ", " 
           << std::fixed << std::setprecision(4) << contact_point(1) << ", " 
           << std::fixed << std::setprecision(4) << contact_point(2) << ", "
           << std::fixed << std::setprecision(4) <<  1.0
           << "]";
        std::string message = ics_ss.str();
        void *reply = redisCommand(main_redis_context, "PUBLISH %s %s", "sensor_data", message.c_str());
        
        if (reply == NULL) {
            std::cerr << "Error: Failed to execute PUBLISH command." << std::endl;
            // 可以在这里尝试重连逻辑
            break; 
        }
        
        freeReplyObject(reply);
        usleep(10000);
    }
    
    return 0;
}
