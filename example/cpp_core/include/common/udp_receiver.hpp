#ifndef UDP_RECEIVER_HPP
#define UDP_RECEIVER_HPP

#include <thread>
#include <mutex>
#include <string>
// 🌟 必须引入这个，不然编译器不认识 VisionData
#include "common/data_types.hpp" 

// 引入你们的数据结构空间
using namespace ruikang;

class UdpReceiver {
public:
    UdpReceiver(int port);
    ~UdpReceiver();

    void start();
    void stop();
    // 🌟 统一使用你们原有的驼峰命名
    VisionData getLatestData(); 

private:
    void receiveLoop();

    int sockfd_;
    bool running_;
    std::thread recv_thread_;

    std::mutex data_mutex_;
    VisionData latest_data_;
};

#endif // UDP_RECEIVER_HPP