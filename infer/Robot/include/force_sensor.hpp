#pragma once

#include <vector> // 为了使用 std::vector 作为缓冲区

#ifndef UINT
typedef unsigned int UINT;
#endif

#ifndef UCHAR
typedef unsigned char UCHAR;
#endif

/**
* @brief: CLinuxSerial class for handling serial communication with the sensor.
*         This modified version uses a buffer to handle continuous data streams robustly.
*/
class CLinuxSerial
{
public:
    float sensor[6]; // 存储解析后的6轴传感器数据

    // 构造函数与析构函数
    CLinuxSerial();
    CLinuxSerial(UINT portNo = 0, UINT baudRate = 115200);
    ~CLinuxSerial();

    // 核心功能函数
    bool InitPort(UINT portNo = 0, UINT baudRate = 115200);
    void ProcessSensorData(); // 新的、健壮的数据处理函数

    // 公共工具函数
    bool IsOpen();
    unsigned char CheckSum(unsigned char *buf, const int len);

    // 底层数据读写
    UINT ReadData(UCHAR *data, UINT length);
    UINT WriteData(UCHAR *data, UINT length);
    UINT GetBytesInCom();

private:
    int m_iSerialID; // 串口文件描述符
    std::vector<unsigned char> m_receiveBuffer; // 用于处理数据流的内部接收缓冲区

    // 私有辅助函数
    bool OpenPort(UINT portNo);
    void ClosePort();
};