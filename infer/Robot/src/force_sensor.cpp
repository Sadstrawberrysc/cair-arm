#include "force_sensor.hpp"
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <iostream>
#include <algorithm> // 需要此头文件以使用 std::search

// 默认构造函数
CLinuxSerial::CLinuxSerial()
{
    m_iSerialID = -1; // 必须初始化为无效值
}

// 带参数的构造函数
CLinuxSerial::CLinuxSerial(UINT portNo /*=0*/, UINT baudRate /*= 115200*/)
{
    m_iSerialID = -1; // 初始化为无效值
    InitPort(portNo, baudRate);
}

CLinuxSerial::~CLinuxSerial()
{
    ClosePort();
}

bool CLinuxSerial::OpenPort(UINT portNo)
{
    char portStr[20];
    sprintf(portStr, "/dev/ttyUSB%d", portNo);
    // 使用 O_NOCTTY: 不让该串口成为进程的控制终端
    // 使用 O_NDELAY: open调用非阻塞
    m_iSerialID = open(portStr, O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_iSerialID < 0)
    {
        perror("OpenPort: Unable to open serial port");
        return false;
    }

    // 将文件描述符设置为阻塞模式，以便read可以等待数据
    fcntl(m_iSerialID, F_SETFL, 0);
    return true;
}

void CLinuxSerial::ClosePort()
{
    if (IsOpen())
    {
        close(m_iSerialID);
        m_iSerialID = -1;
    }
}

bool CLinuxSerial::IsOpen()
{
    return m_iSerialID >= 0;
}

bool CLinuxSerial::InitPort(UINT portNo /*= 0*/, UINT baudRate /*= 115200*/)
{
    if (IsOpen())
    {
        ClosePort();
    }

    if (!OpenPort(portNo))
    {
        return false;
    }

    struct termios options;
    // 1. 获取当前串口属性
    if (tcgetattr(m_iSerialID, &options) != 0)
    {
        perror("InitPort: tcgetattr");
        return false;
    }

    // 2. 设置波特率
    int st_baud[] = {B4800, B9600, B19200, B38400, B57600, B115200, B230400, B1000000, B1152000, B3000000};
    int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 1000000, 1152000, 3000000};
    bool baud_found = false;
    for (size_t i = 0; i < sizeof(std_rate) / sizeof(int); i++)
    {
        if (std_rate[i] == baudRate)
        {
            cfsetispeed(&options, st_baud[i]);
            cfsetospeed(&options, st_baud[i]);
            baud_found = true;
            break;
        }
    }
    if (!baud_found)
    {
        fprintf(stderr, "InitPort: Unsupported baud rate\n");
        return false;
    }

    // 3. 设置串口模式为 "raw" 模式
    // 控制模式: 8N1, 忽略调制解调器状态线, 开启接收
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;

    // 输入模式: 关闭软件流控, 关闭所有特殊字符转换
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);

    // 本地模式: 非标准模式(raw), 关闭回显和信号
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 输出模式: raw输出
    options.c_oflag &= ~OPOST;

    // 4. 设置read()的超时行为
    // VMIN = 0, VTIME = 1: read()将等待最多0.1秒。如果在此期间有数据到达，它会立即返回收到的字节数。
    // 如果没有数据，它会在0.1秒后返回0。这对于轮询非常理想。
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    // 5. 清空输入输出缓冲区
    tcflush(m_iSerialID, TCIOFLUSH);

    // 6. 应用新的设置
    if (tcsetattr(m_iSerialID, TCSANOW, &options) != 0)
    {
        perror("InitPort: tcsetattr");
        return false;
    }

    std::cout << "Serial port initialized successfully on /dev/ttyUSB" << portNo << std::endl;
    return true;
}

UINT CLinuxSerial::ReadData(UCHAR *data, UINT length)
{
    if (!IsOpen()) return 0;
    int ret = read(m_iSerialID, data, length);
    return (ret > 0) ? ret : 0;
}

UINT CLinuxSerial::WriteData(UCHAR *data, UINT length)
{
    if (!IsOpen()) return 0;
    return write(m_iSerialID, data, length);
}

UINT CLinuxSerial::GetBytesInCom()
{
    if (!IsOpen()) return 0;
    int bytes_avail = 0;
    ioctl(m_iSerialID, FIONREAD, &bytes_avail);
    return bytes_avail;
}

unsigned char CLinuxSerial::CheckSum(unsigned char *buf, const int len)
{
    unsigned char ret = 0;
    for (int i = 0; i < len; i++)
    {
        ret += buf[i];
    }
    return ret;
}

// 这是新的、健壮的数据处理函数
void CLinuxSerial::ProcessSensorData()
{
    if (!IsOpen())
    {
        return;
    }

    // 步骤 1: 从串口读取任何可用的新数据，并将其追加到内部缓冲区
    unsigned char temp_buf[256];
    int bytes_read = ReadData(temp_buf, 256);
    if (bytes_read > 0)
    {
        m_receiveBuffer.insert(m_receiveBuffer.end(), temp_buf, temp_buf + bytes_read);
    }

    // 定义数据包结构常量
    const unsigned char packet_header[] = {0xaa, 0x55};
    const int PACKET_SIZE = 31;
    const int PAYLOAD_OFFSET = 6;
    const int PAYLOAD_SIZE = 24;
    const int CHECKSUM_OFFSET = 30;

    // 步骤 2: 在缓冲区中循环查找并处理所有完整的数据包
    while (true)
    {
        // 步骤 3: 在缓冲区中查找数据包的帧头 (0xaa 0x55)
        auto it = std::search(m_receiveBuffer.begin(), m_receiveBuffer.end(), packet_header, packet_header + 2);

        // 如果找不到帧头，说明当前缓冲区没有有效数据包的开始，退出循环，等待更多数据
        if (it == m_receiveBuffer.end())
        {
            break;
        }

        // 步骤 4: 移除帧头前的所有无效数据
        m_receiveBuffer.erase(m_receiveBuffer.begin(), it);

        // 步骤 5: 检查剩余数据是否足够构成一个完整的数据包
        if (m_receiveBuffer.size() < PACKET_SIZE)
        {
            // 数据不够长，退出循环，等待下一次读取更多数据
            break;
        }

        // 步骤 6: 提取数据包并进行校验和检查
        unsigned char* payload = m_receiveBuffer.data() + PAYLOAD_OFFSET;
        unsigned char calculated_checksum = CheckSum(payload, PAYLOAD_SIZE);
        unsigned char received_checksum = m_receiveBuffer[CHECKSUM_OFFSET];

        if (calculated_checksum == received_checksum)
        {
            // 步骤 7: 校验成功，解析数据
            for (int index = 0; index < 6; index++)
            {
                // 使用 reinterpret_cast 直接从缓冲区转换，高效且安全（因为我们已确保数据完整）
                float* ret = reinterpret_cast<float*>(payload + 4 * index);
                sensor[index] = *ret;
                // std::cout << sensor[index] << " ";
            }
            // std::cout << std::endl; // 打印完一组数据后换行

            // 步骤 8: 从缓冲区中移除已成功处理的数据包
            m_receiveBuffer.erase(m_receiveBuffer.begin(), m_receiveBuffer.begin() + PACKET_SIZE);
        }
        else
        {
            // 校验失败，这可能是一个“假”的帧头。
            // 丢弃这个错误的帧头（只丢弃第一个字节），以便下次循环可以从下一个字节开始重新搜索。
            // 这可以防止因数据内容恰好为0xaa 0x55而导致的死循环。
            // std::cerr << "Checksum error. Discarding invalid packet header." << std::endl;
            m_receiveBuffer.erase(m_receiveBuffer.begin());
        }
    } // while 循环结束，继续处理缓冲区中可能存在的下一个数据包
}