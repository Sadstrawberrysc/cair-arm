// // #include <chrono>
// // #include <thread>
// // #include <iostream>
// // #include <iomanip>
// // #include <cmath>

// // #include <force_haptron.hpp>

// // float g_sensor[6] = {0};

// // // 传感器线程：串口读取六维力/矩
// // void ForceSensor()
// // {
// //     float sensor_force_offset[3] = {0};
// //     float sensor_torque_offset[3] = {0};
// //     int iniCount = 0;

// //     CLinuxSerial com(1, 115200);
// //     while (true)
// //     {
// //         if (com.InitPort(1, 115200))
// //         {
// //             unsigned char data[] = {0x01, 0x04, 0x00, 0x38, 0x00, 0x0C, 0x71, 0xC2};
// //             UINT result = com.WriteData(data, sizeof(data));
// //             if (result > 0)
// //             {
// //                 unsigned char buf[31];
// //                 com.ReadData(buf, 31);
// //                 if ((buf[0] == 0x01) && (buf[1] == 0x04) && (buf[2] == 0x18))
// //                 {
// //                     unsigned char fx_tmp[4] = {buf[6], buf[5], buf[4], buf[3]};
// //                     unsigned char fy_tmp[4] = {buf[10], buf[9], buf[8], buf[7]};
// //                     unsigned char fz_tmp[4] = {buf[14], buf[13], buf[12], buf[11]};
// //                     unsigned char tx_tmp[4] = {buf[18], buf[17], buf[16], buf[15]};
// //                     unsigned char ty_tmp[4] = {buf[22], buf[21], buf[20], buf[19]};
// //                     unsigned char tz_tmp[4] = {buf[26], buf[25], buf[24], buf[23]};
// //                     float fx = *((float *)(fx_tmp));
// //                     float fy = *((float *)(fy_tmp));
// //                     float fz = *((float *)(fz_tmp));
// //                     float tx = *((float *)(tx_tmp));
// //                     float ty = *((float *)(ty_tmp));
// //                     float tz = *((float *)(tz_tmp));

// //                     constexpr int INIT_NUM = 100;
// //                     if (iniCount < INIT_NUM)
// //                     {
// //                         // 可做零漂统计
// //                         iniCount++;
// //                         sensor_force_offset[0] += fx;
// //                         sensor_force_offset[1] += fy;
// //                         sensor_force_offset[2] += fz;
// //                         sensor_torque_offset[0] += tx;
// //                         sensor_torque_offset[1] += ty;
// //                         sensor_torque_offset[2] += tz;
// //                     }
// //                     else
// //                     {
// //                         float s[6];
// //                         s[0] = fx - sensor_force_offset[0]/INIT_NUM;
// //                         s[1] = fy - sensor_force_offset[1]/INIT_NUM;
// //                         s[2] = fz - sensor_force_offset[2]/INIT_NUM;
// //                         s[3] = tx - sensor_torque_offset[0]/INIT_NUM;
// //                         s[4] = ty - sensor_torque_offset[1]/INIT_NUM;
// //                         s[5] = tz - sensor_torque_offset[2]/INIT_NUM;
// //                         for (int i=0;i<6;i++)
// //                             g_sensor[i] = (std::fabs(s[i]) > 0.0f) ? s[i] : 0.0f;
// //                     }
// //                 }
// //             }
// //         }
// //         // 可视情况 sleep 降低CPU占用
// //         // std::this_thread::sleep_for(std::chrono::milliseconds(1));
// //     }
// // }

// // int main()
// // {
// //     std::thread th_sensor(ForceSensor);
// //     th_sensor.detach();

// //     std::cout << "Force sensor reader started. Printing Fx Fy Fz Tx Ty Tz..." << std::endl;

// //     while (true)
// //     {
// //         std::cout << std::fixed << std::setprecision(6)
// //                   << "F/T: "
// //                   << g_sensor[0] << "  "
// //                   << g_sensor[1] << "  "
// //                   << g_sensor[2] << "  "
// //                   << g_sensor[3] << "  "
// //                   << g_sensor[4] << "  "
// //                   << g_sensor[5] << std::endl;

// //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
// //     }

// //     return 0;
// // }








// #include <chrono>
// #include <thread>
// #include <iostream>
// #include <iomanip>
// #include <cmath>
// #include <fstream>
// #include <atomic>
// #include <mutex>
// #include <cstdlib>
// #include <string>

// #include <force_haptron.hpp>

// float g_sensor[6] = {0};
// std::atomic<bool> g_running(true);
// std::mutex g_mutex;

// void ForceSensor()
// {
//     float sensor_force_offset[3] = {0};
//     float sensor_torque_offset[3] = {0};
//     int iniCount = 0;

//     CLinuxSerial com(1, 115200);
//     while (g_running)
//     {
//         if (com.InitPort(1, 115200))
//         {
//             unsigned char data[] = {0x01, 0x04, 0x00, 0x38, 0x00, 0x0C, 0x71, 0xC2};
//             UINT result = com.WriteData(data, sizeof(data));
//             if (result > 0)
//             {
//                 unsigned char buf[31];
//                 com.ReadData(buf, 31);
//                 if ((buf[0] == 0x01) && (buf[1] == 0x04) && (buf[2] == 0x18))
//                 {
//                     unsigned char fx_tmp[4] = {buf[6], buf[5], buf[4], buf[3]};
//                     unsigned char fy_tmp[4] = {buf[10], buf[9], buf[8], buf[7]};
//                     unsigned char fz_tmp[4] = {buf[14], buf[13], buf[12], buf[11]};
//                     unsigned char tx_tmp[4] = {buf[18], buf[17], buf[16], buf[15]};
//                     unsigned char ty_tmp[4] = {buf[22], buf[21], buf[20], buf[19]};
//                     unsigned char tz_tmp[4] = {buf[26], buf[25], buf[24], buf[23]};
//                     float fx = *((float *)(fx_tmp));
//                     float fy = *((float *)(fy_tmp));
//                     float fz = *((float *)(fz_tmp));
//                     float tx = *((float *)(tx_tmp));
//                     float ty = *((float *)(ty_tmp));
//                     float tz = *((float *)(tz_tmp));

//                     constexpr int INIT_NUM = 100;
//                     if (iniCount < INIT_NUM)
//                     {
//                         iniCount++;
//                         sensor_force_offset[0] += fx;
//                         sensor_force_offset[1] += fy;
//                         sensor_force_offset[2] += fz;
//                         sensor_torque_offset[0] += tx;
//                         sensor_torque_offset[1] += ty;
//                         sensor_torque_offset[2] += tz;
//                     }
//                     else
//                     {
//                         float s[6];
//                         s[0] = fx - sensor_force_offset[0]/INIT_NUM;
//                         s[1] = fy - sensor_force_offset[1]/INIT_NUM;
//                         s[2] = fz - sensor_force_offset[2]/INIT_NUM;
//                         s[3] = tx - sensor_torque_offset[0]/INIT_NUM;
//                         s[4] = ty - sensor_torque_offset[1]/INIT_NUM;
//                         s[5] = tz - sensor_torque_offset[2]/INIT_NUM;

//                         std::lock_guard<std::mutex> lock(g_mutex);
//                         for (int i=0;i<6;i++)
//                             g_sensor[i] = (std::fabs(s[i]) > 0.0f) ? s[i] : 0.0f;
//                     }
//                 }
//             }
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }
// }

// void KeyboardMonitor()
// {
//     char c;
//     while (g_running)
//     {
//         if (std::cin.get(c))
//         {
//             if (c == 'q' || c == 'Q')
//             {
//                 g_running = false;
//                 break;
//             }
//         }
//     }
// }

// void GenerateAndRunPlotScript()
// {
//     std::ofstream pyScript("plot_data.py");
//     if (pyScript.is_open())
//     {
//         pyScript << "import pandas as pd\n"
//                  << "import matplotlib.pyplot as plt\n"
//                  << "try:\n"
//                  << "    df = pd.read_csv('sensor_data.csv')\n"
//                  << "    plt.figure(figsize=(12, 8))\n"
//                  << "    for col in df.columns[1:]:\n"
//                  << "        plt.plot(df['Time'], df[col], label=col)\n"
//                  << "    plt.xlabel('Time (s)')\n"
//                  << "    plt.ylabel('Sensor Value')\n"
//                  << "    plt.title('Sensor Measurements over Time')\n"
//                  << "    plt.legend()\n"
//                  << "    plt.grid(True)\n"
//                  << "    plt.savefig('sensor_plot.png')\n"
//                  << "    print('Plot saved to sensor_plot.png')\n"
//                  << "except Exception as e:\n"
//                  << "    print('Error plotting data:', e)\n";
//         pyScript.close();
//         std::system("python3 plot_data.py");
//     }
// }

// int main()
// {
//     std::ofstream dataFile("sensor_data.csv");
//     if (!dataFile.is_open())
//     {
//         std::cerr << "Failed to open sensor_data.csv for writing." << std::endl;
//         return 1;
//     }
//     dataFile << "Time,Fx,Fy,Fz,Tx,Ty,Tz\n";

//     std::thread th_sensor(ForceSensor);
//     th_sensor.detach();

//     std::thread th_keyboard(KeyboardMonitor);

//     std::cout << "Force sensor reader started. Press 'q' and Enter to stop and generate plot." << std::endl;

//     auto start_time = std::chrono::steady_clock::now();
//     auto last_save_time = start_time;

//     while (g_running)
//     {
//         auto current_time = std::chrono::steady_clock::now();
//         auto elapsed_since_save = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_save_time).count();

//         if (elapsed_since_save >= 3)
//         {
//             float current_sensor[6];
//             {
//                 std::lock_guard<std::mutex> lock(g_mutex);
//                 for (int i = 0; i < 6; i++) current_sensor[i] = g_sensor[i];
//             }

//             auto elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

//             dataFile << elapsed_total << ","
//                      << current_sensor[0] << "," << current_sensor[1] << "," << current_sensor[2] << ","
//                      << current_sensor[3] << "," << current_sensor[4] << "," << current_sensor[5] << "\n";
//             dataFile.flush();

//             std::cout << "Saved data at " << elapsed_total << "s" << std::endl;
//             last_save_time = current_time;
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }

//     dataFile.close();
//     std::cout << "Data collection stopped. Generating plot..." << std::endl;

//     GenerateAndRunPlotScript();

//     if (th_keyboard.joinable())
//     {
//         th_keyboard.join();
//     }

//     return 0;
// }



#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <atomic>
#include <mutex>
#include <cstdlib>
#include <string>

#include <force_haptron.hpp>

float g_sensor[6] = {0};
std::atomic<bool> g_running(true);
std::mutex g_mutex;

void ForceSensor()
{
    float sensor_force_offset[3] = {0};
    float sensor_torque_offset[3] = {0};
    int iniCount = 0;

    CLinuxSerial com(1, 115200);
    while (g_running)
    {
        if (com.InitPort(1, 115200))
        {
            unsigned char data[] = {0x01, 0x04, 0x00, 0x38, 0x00, 0x0C, 0x71, 0xC2};
            UINT result = com.WriteData(data, sizeof(data));
            if (result > 0)
            {
                unsigned char buf[31];
                com.ReadData(buf, 31);
                if ((buf[0] == 0x01) && (buf[1] == 0x04) && (buf[2] == 0x18))
                {
                    unsigned char fx_tmp[4] = {buf[6], buf[5], buf[4], buf[3]};
                    unsigned char fy_tmp[4] = {buf[10], buf[9], buf[8], buf[7]};
                    unsigned char fz_tmp[4] = {buf[14], buf[13], buf[12], buf[11]};
                    unsigned char tx_tmp[4] = {buf[18], buf[17], buf[16], buf[15]};
                    unsigned char ty_tmp[4] = {buf[22], buf[21], buf[20], buf[19]};
                    unsigned char tz_tmp[4] = {buf[26], buf[25], buf[24], buf[23]};
                    float fx = *((float *)(fx_tmp));
                    float fy = *((float *)(fy_tmp));
                    float fz = *((float *)(fz_tmp));
                    float tx = *((float *)(tx_tmp));
                    float ty = *((float *)(ty_tmp));
                    float tz = *((float *)(tz_tmp));

                    constexpr int INIT_NUM = 100;
                    if (iniCount < INIT_NUM)
                    {
                        iniCount++;
                        sensor_force_offset[0] += fx;
                        sensor_force_offset[1] += fy;
                        sensor_force_offset[2] += fz;
                        sensor_torque_offset[0] += tx;
                        sensor_torque_offset[1] += ty;
                        sensor_torque_offset[2] += tz;
                    }
                    else
                    {
                        float s[6];
                        s[0] = fx - sensor_force_offset[0]/INIT_NUM;
                        s[1] = fy - sensor_force_offset[1]/INIT_NUM;
                        s[2] = fz - sensor_force_offset[2]/INIT_NUM;
                        s[3] = tx - sensor_torque_offset[0]/INIT_NUM;
                        s[4] = ty - sensor_torque_offset[1]/INIT_NUM;
                        s[5] = tz - sensor_torque_offset[2]/INIT_NUM;

                        std::lock_guard<std::mutex> lock(g_mutex);
                        for (int i=0;i<6;i++)
                            g_sensor[i] = (std::fabs(s[i]) > 0.0f) ? s[i] : 0.0f;
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void KeyboardMonitor()
{
    std::string input;
    while (g_running)
    {
        std::cin >> input;
        if (input == "q" || input == "Q")
        {
            g_running = false;
            break;
        }
    }
}

void GenerateAndRunPlotScript()
{
    std::ofstream pyScript("plot_data.py");
    if (pyScript.is_open())
    {
        pyScript << "import pandas as pd\n"
                 << "import matplotlib.pyplot as plt\n"
                 << "try:\n"
                 << "    df = pd.read_csv('sensor_data.csv')\n"
                 << "    plt.figure(figsize=(12, 8))\n"
                 << "    for col in df.columns[1:]:\n"
                 << "        plt.plot(df['Time'], df[col], label=col)\n"
                 << "    plt.xlabel('Time (s)')\n"
                 << "    plt.ylabel('Sensor Value')\n"
                 << "    plt.title('Sensor Measurements over Time')\n"
                 << "    plt.legend()\n"
                 << "    plt.grid(True)\n"
                 << "    plt.savefig('sensor_plot.png')\n"
                 << "    print('Plot saved to sensor_plot.png')\n"
                 << "except Exception as e:\n"
                 << "    print('Error plotting data:', e)\n";
        pyScript.close();
        std::system("python3 plot_data.py");
    }
}

int main()
{
    std::ofstream dataFile("sensor_data.csv");
    if (!dataFile.is_open())
    {
        std::cerr << "Failed to open sensor_data.csv for writing." << std::endl;
        return 1;
    }
    dataFile << "Time,Fx,Fy,Fz,Tx,Ty,Tz\n";

    std::thread th_sensor(ForceSensor);
    th_sensor.detach();

    std::thread th_keyboard(KeyboardMonitor);

    std::cout << "Force sensor reader started. Press 'q' and Enter to stop and generate plot." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    auto last_save_time = start_time;

    while (g_running)
    {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_since_save = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_save_time).count();

        if (elapsed_since_save >= 30)
        {
            float current_sensor[6];
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                for (int i = 0; i < 6; i++) current_sensor[i] = g_sensor[i];
            }

            double elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count()/3600;

            dataFile << elapsed_total << ","
                     << current_sensor[0] << "," << current_sensor[1] << "," << current_sensor[2] << ","
                     << current_sensor[3] << "," << current_sensor[4] << "," << current_sensor[5] << "\n";
            dataFile.flush();

            std::cout << "Saved data at " << elapsed_total << "s" << std::endl;
            last_save_time = current_time;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    dataFile.close();
    std::cout << "Data collection stopped. Generating plot..." << std::endl;

    GenerateAndRunPlotScript();

    if (th_keyboard.joinable())
    {
        th_keyboard.join();
    }

    return 0;
}
