#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <atomic>
#include <thread>
#include <chrono>
#include "httplib.h"
#include "Drone.h"      // 确保 Drone 类完整定义
#include "main.h"       // 提供 pos_pid_xy 等全局变量声明

#define REQ_MAX_VEL_X        3        // 允许远程控制的最大X方向速度值(m/s)
#define REQ_MAX_VEL_Y        3        // 允许远程控制的最大Y方向速度值(m/s)
#define REQ_MAX_POS_X        10       // 允许远程控制的最大X方向坐标值(m)
#define REQ_MAX_POS_Y        100      // 允许远程控制的最大Y方向坐标值(m)
#define REQ_MAX_POS_Z        5        // 允许远程控制的最大Z方向坐标值(m)

class WebServer {
public:
    // 构造函数，绑定一个 Drone 实例的引用（必须保证 Drone 生命周期长于本对象）
    explicit WebServer(Drone& drone);

    // 禁止拷贝和移动（因为持有引用和线程）
    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    // 启动 PID Web 调参服务（内部创建线程）
    void start();

    // 停止服务（可选，根据需求实现）
    void stop();

private:
    // 路由处理函数
    void handle_pid(const httplib::Request& req, httplib::Response& res);
    void handle_cmd(const httplib::Request& req, httplib::Response& res);

    // 服务器运行函数（在独立线程中执行）
    void run_server();

    Drone& drone_;                      // Drone 引用，用于路由回调
    std::atomic<bool> server_running_;  // 服务器运行状态标志
    std::thread server_thread_;         // 服务器线程
};

#endif // WEBSERVER_H