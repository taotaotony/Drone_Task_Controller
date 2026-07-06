#ifndef TELEMETRY_SNAPSHOT_H
#define TELEMETRY_SNAPSHOT_H

#include <mutex>
#include <string>

// 遥测数据快照 — 由 ROS 回调更新，由广播线程读取
// 所有读写受 telemetry_mutex 保护
struct TelemetrySnapshot
{
    // MAVROS 连接状态
    bool connected = false;
    bool armed = false;
    std::string mode;

    // 位置 (机体坐标系)
    double pos_x = 0.0;
    double pos_y = 0.0;
    double pos_z = 0.0;

    // 速度 (机体坐标系)
    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_z = 0.0;

    // 初始偏航角
    double initial_yaw = 0.0;

    // 无人机状态机
    int drone_state = -1;
    std::string drone_state_name;
};

// 全局遥测快照实例
extern TelemetrySnapshot telemetry_snapshot;

// 互斥锁：所有对 telemetry_snapshot 的读写必须持有此锁
extern std::mutex telemetry_mutex;

#endif // TELEMETRY_SNAPSHOT_H