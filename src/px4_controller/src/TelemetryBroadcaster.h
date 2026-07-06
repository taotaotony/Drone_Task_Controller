#ifndef TELEMETRY_BROADCASTER_H
#define TELEMETRY_BROADCASTER_H

#include <atomic>
#include <thread>
#include <string>

#define TELEMETRY_UDP_PORT     18881    // UDP 广播端口（兼容 QGroundControl 默认端口）
#define TELEMETRY_BROADCAST_IP "255.255.255.255"
#define TELEMETRY_RATE_HZ      20       // 广播频率

class TelemetryBroadcaster {
public:
    TelemetryBroadcaster();
    ~TelemetryBroadcaster();

    // 禁止拷贝
    TelemetryBroadcaster(const TelemetryBroadcaster&) = delete;
    TelemetryBroadcaster& operator=(const TelemetryBroadcaster&) = delete;

    // 启动广播线程
    void start();

    // 停止广播
    void stop();

    // 检查是否正在运行
    bool is_running() const { return running_; }

private:
    // 广播线程函数（包含快照更新 + JSON 构建 + UDP 发送）
    void broadcast_loop();

    int sock_fd_;                       // UDP socket 文件描述符
    std::atomic<bool> running_;         // 运行标志
    std::thread broadcast_thread_;      // 广播线程
};

#endif // TELEMETRY_BROADCASTER_H