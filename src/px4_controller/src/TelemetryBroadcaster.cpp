#include "TelemetryBroadcaster.h"
#include "main.h"
#include "TelemetrySnapshot.h"
#include "Drone.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <mutex>

#include "ros/ros.h"

TelemetryBroadcaster::TelemetryBroadcaster()
    : sock_fd_(-1)
    , running_(false)
{
}

TelemetryBroadcaster::~TelemetryBroadcaster()
{
    stop();
}

void TelemetryBroadcaster::start()
{
    if (running_) {
        ROS_WARN("TelemetryBroadcaster already running.");
        return;
    }

    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        ROS_ERROR("TelemetryBroadcaster: Failed to create UDP socket.");
        return;
    }

    int broadcast_enable = 1;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_BROADCAST,
                   &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        ROS_ERROR("TelemetryBroadcaster: Failed to enable broadcast.");
        close(sock_fd_);
        sock_fd_ = -1;
        return;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    running_ = true;
    broadcast_thread_ = std::thread(&TelemetryBroadcaster::broadcast_loop, this);

    ROS_INFO("TelemetryBroadcaster started (UDP broadcast on port %d, %d Hz).",
             TELEMETRY_UDP_PORT, TELEMETRY_RATE_HZ);
}

void TelemetryBroadcaster::stop()
{
    if (!running_) return;

    running_ = false;

    if (broadcast_thread_.joinable()) {
        broadcast_thread_.join();
    }

    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }

    ROS_INFO("TelemetryBroadcaster stopped.");
}

void TelemetryBroadcaster::broadcast_loop()
{
    struct sockaddr_in broadcast_addr;
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(TELEMETRY_UDP_PORT);
    broadcast_addr.sin_addr.s_addr = inet_addr(TELEMETRY_BROADCAST_IP);

    auto interval = std::chrono::milliseconds(1000 / TELEMETRY_RATE_HZ);
    auto next_send = std::chrono::steady_clock::now() + interval;

    while (running_) {
        std::string telemetry;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex);

            // 更新 drone 状态到快照
            extern Drone drone;
            telemetry_snapshot.drone_state = static_cast<int>(drone.GetState());
            telemetry_snapshot.drone_state_name = drone.GetStateName();

            const TelemetrySnapshot& s = telemetry_snapshot;

            std::ostringstream json;
            json << std::fixed << std::setprecision(4);
            json << "{";

            json << "\"ts\":" << ros::Time::now().toSec() << ",";

            // MAVROS 连接状态
            json << "\"connected\":" << (s.connected ? 1 : 0) << ",";
            json << "\"armed\":" << (s.armed ? 1 : 0) << ",";
            json << "\"mode\":\"" << s.mode << "\",";

            // 无人机状态机
            json << "\"state\":" << s.drone_state << ",";
            json << "\"state_name\":\"" << s.drone_state_name << "\",";

            // 位置/速度 (机体坐标系)
            json << "\"pos_x\":" << s.pos_x << ",";
            json << "\"pos_y\":" << s.pos_y << ",";
            json << "\"pos_z\":" << s.pos_z << ",";
            json << "\"vel_x\":" << s.vel_x << ",";
            json << "\"vel_y\":" << s.vel_y << ",";
            json << "\"vel_z\":" << s.vel_z << ",";

            // 偏航角
            json << "\"initial_yaw\":" << s.initial_yaw;

            json << "}";
            telemetry = json.str();
        }
        // 锁已释放，以下为纯 I/O

        // 4 字节魔数头 "TEL\x00" 过滤非遥测流量 (MAVLink 等)
        // 注意：不能用 std::string("TEL\0") 因为 \0 会被截断
        std::string framed = std::string("TEL\x00", 4) + telemetry;

        ssize_t sent = sendto(sock_fd_, framed.c_str(), framed.size(), 0,
                              (struct sockaddr*)&broadcast_addr,
                              sizeof(broadcast_addr));

        if (sent < 0) {
            ROS_WARN_THROTTLE(5.0, "TelemetryBroadcaster: sendto failed.");
        }

        std::this_thread::sleep_until(next_send);
        next_send += interval;

        auto now = std::chrono::steady_clock::now();
        if (next_send < now) {
            next_send = now + interval;
        }
    }
}