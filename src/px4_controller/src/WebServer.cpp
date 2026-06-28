#include "WebServer.h"
#include <iostream>
#include <string>
#include "ros/ros.h"   // ROS_INFO 需要

WebServer::WebServer(Drone& drone)
    : drone_(drone)
    , server_running_(false)
{
}

void WebServer::start()
{
    // 如果线程已经运行或服务器已启动，则避免重复启动
    if (server_running_ || server_thread_.joinable()) {
        ROS_WARN("WebServer already started.");
        return;
    }

    // 在新线程中启动 HTTP 服务器，使用 std::ref 避免拷贝
    server_thread_ = std::thread(&WebServer::run_server, this);

    // 等待服务器真正开始监听（原子标志变为 true）
    while (!server_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 分离线程，让其独立运行
    server_thread_.detach();
    ROS_INFO("WebServer started on port 8880.");
}

void WebServer::stop()
{
    // 简单实现：不做强制停止，服务器需自行监听终止信号
    // 实际项目中可通过 httplib::Server::stop() 实现
    if (server_running_) {
        // 此处可向服务器发送停止信号，例如设置一个原子标志
        // 由于 httplib::Server::listen 是阻塞的，通常需要从其他线程调用 svr.stop()
        // 示例中未实现详细停止逻辑，可自行扩展
    }
}

void WebServer::handle_pid(const httplib::Request& req, httplib::Response& res)
{
    if (!req.has_param("channel") || !req.has_param("p") ||
        !req.has_param("i") || !req.has_param("d") || !req.has_param("limit")) {
        res.set_content("Param ERROR!", "text/plain");
        std::cout << "\nerror!\n";
        res.status = 400;
        return;
    }

    try {
        int channel = std::stoi(req.get_param_value("channel"));
        double p = std::stod(req.get_param_value("p"));
        double i = std::stod(req.get_param_value("i"));
        double d = std::stod(req.get_param_value("d"));
        double limit = std::stod(req.get_param_value("limit"));

        // pos_pid_xy 是全局 PID 结构体，定义在 main.h 中
        pos_pid_xy.kp = p;
        pos_pid_xy.ki = i;
        pos_pid_xy.kd = d;
        ROS_INFO("已更新瞄准PID参数 Kp = %.6f Ki = %.6f Kd = %.6f",
                 pos_pid_xy.kp, pos_pid_xy.ki, pos_pid_xy.kd);

        res.set_content("已更新PID参数", "text/plain");
        res.status = 200;
    } catch (...) {
        res.set_content("Param ERROR!", "text/plain");
        res.status = 400;
    }
}

void WebServer::handle_cmd(const httplib::Request& req, httplib::Response& res)
{
    // 预留接口，以后扩展
    res.set_content("Command not implemented", "text/plain");
}

void WebServer::run_server()
{
    httplib::Server svr;

    // 绑定成员函数作为路由处理器（需要捕获 this）
    svr.Get("/setpid", [this](const httplib::Request& req, httplib::Response& res) {
        this->handle_pid(req, res);
    });

    svr.Get("/setstat", [this](const httplib::Request& req, httplib::Response& res) {
        drone_.UpdateState(req, res);   // 使用存储的 drone_ 引用
    });
    
    svr.Get("/setgoal", [this](const httplib::Request& req, httplib::Response& res) {
        drone_.UpdateGoal(req, res);   // 使用存储的 drone_ 引用
    });

    svr.Get("/", [](const httplib::Request& req, httplib::Response& res) {
        res.set_content("Hello! Welcome to Web PID Tuner.\n"
                        "Use 'PID Tuner for QFOC.exe' to set PID Params!",
                        "text/plain");
    });

    // 通知主线程服务器已启动
    server_running_ = true;
    ROS_INFO("HTTP server listening on port 8880...");
    svr.listen("0.0.0.0", 8880);

    // 服务器退出后重置标志
    server_running_ = false;
}