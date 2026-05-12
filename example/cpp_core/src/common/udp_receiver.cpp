#include "common/udp_receiver.hpp"
#include "common/data_types.hpp"
#include "common/json.hpp" 
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

using json = nlohmann::json;
using namespace ruikang;

UdpReceiver::UdpReceiver(int port) : running_(false) {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "[UDP] Error creating socket" << std::endl;
        return;
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1
    servaddr.sin_port = htons(port);

    if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "[UDP] Error binding socket" << std::endl;
    }

    // 初始化默认安全距离，后续会被 main.cpp 里的 C++ 雷达强制覆盖
    latest_data_.obstacle_distance = 9.99f;
}

UdpReceiver::~UdpReceiver() {
    stop();
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

void UdpReceiver::start() {
    running_ = true;
    recv_thread_ = std::thread(&UdpReceiver::receiveLoop, this);
}

void UdpReceiver::stop() {
    running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

VisionData UdpReceiver::getLatestData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
}

void UdpReceiver::receiveLoop() {
    char buffer[1024];
    while (running_) {
        int n = recvfrom(sockfd_, (char *)buffer, 1024, MSG_DONTWAIT, NULL, NULL);
        if (n > 0) {
            buffer[n] = '\0';

            try {
                json j = json::parse(buffer);

                std::lock_guard<std::mutex> lock(data_mutex_);

                // 视觉
                if (j.contains("line_offset"))
                    latest_data_.line_offset = j["line_offset"].get<float>();
                if (j.contains("platform_tag")) {
                    // platform_tag 可能是 int 也可能是 str（兼容）
                    auto& v = j["platform_tag"];
                    if (v.is_number())
                        latest_data_.platform_tag = v.get<int>();
                }
                if (j.contains("warning_sign"))
                    latest_data_.warning_sign = j["warning_sign"].get<std::string>();
                if (j.contains("aruco_detected"))
                    latest_data_.aruco_detected = j["aruco_detected"].get<bool>();
                // ArUco 码中心坐标（State04 台阶对齐用，Y 发 JSON 时需同步加字段）
                if (j.contains("aruco_center_x"))
                    latest_data_.aruco_center_x = j["aruco_center_x"].get<float>();
                if (j.contains("aruco_center_y"))
                    latest_data_.aruco_center_y = j["aruco_center_y"].get<float>();

                // ★ 深度相机字段（方案三 V2）
                if (j.contains("depth_front"))
                    latest_data_.depth_front = j["depth_front"].get<float>();
                if (j.contains("depth_left"))
                    latest_data_.depth_left  = j["depth_left"].get<float>();
                if (j.contains("depth_right"))
                    latest_data_.depth_right = j["depth_right"].get<float>();
                if (j.contains("turn_trend"))
                    latest_data_.turn_trend = j["turn_trend"].get<float>();
                if (j.contains("red_dot_detected"))
                    latest_data_.red_dot_detected = j["red_dot_detected"].get<bool>();
                if (j.contains("red_dot_center_x"))
                    latest_data_.red_dot_center_x = j["red_dot_center_x"].get<float>();
                if (j.contains("is_sharp_turn"))
                    latest_data_.is_sharp_turn = j["is_sharp_turn"].get<bool>();

            } catch (const json::parse_error& e) {
                std::cerr << "[UDP] JSON Parse Error: " << e.what() << std::endl;
            }
        }
        usleep(2000);
    }
}