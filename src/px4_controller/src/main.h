#ifndef __main_H
#define __main_H

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <string>
#include <cstdlib>
#include <px4_controller/position.h>
#include <px4_controller/throwcmd.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <px4_controller/tbag.h>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <thread>
#include <httplib.h>

// ── 宏定义 ──────────────────────────────────
#define TakeofftoThrow 32.5  // 起飞点到投放中心的距离 (32.5m)
#define TakeofftoDetect 57.5 // 起飞点到侦察中心的距离 (57.5m)
#define Throw1_X 660         // 1号舵机能投放进的像素x
#define Throw1_Y 288
#define Throw2_X 660
#define Throw2_Y 515
#define DetectHeight 3.5     // 侦察高度
#define InitialHeight 3.5    // 初始起飞高度
#define ThrowHeight 0.8      // 投放时高度
#define ThrowMidHeight 2.1   // 投放二级瞄准高度

// PID 参数默认值
#define Kp_H 0.01
#define Ki_H 0
#define Kd_H 0
#define Kp_L 0.001
#define Ki_L 0
#define Kd_L 0
#define MaxVel_H 0.15
#define MaxVel_L 0.05
#define DeadZone 0.01

// 摄像头参数
#define CamX 1280
#define CamY 800
#define MinPx 15

// 时间限制参数
#define MaxThrowDuration 120

// ── 类型定义 ────────────────────────────────
struct PIDController
{
    double kp, ki, kd;
    double ans_error;
    double previous_error;
};

struct Position
{
    double x;
    double y;
    double z;
};

struct GoalPosVel  //存储目标位速信息
{
    int  mode;
    double px;
    double py;
    double pz;
    double vx;
    double vy;
};

struct Velocity
{
    double vx;
    double vy;
    double vz;
};

// ── 全局变量 extern 声明 ─────────────────────
extern struct Position PX4_Position;
extern struct Velocity PX4_Velocity;
extern PIDController pos_pid_xy;
extern double initial_yaw;

// ── 函数声明 ────────────────────────────────
bool PX4_SetMode(std::string);
bool SetMode(std::string);
bool Arm();
bool SetPoint(double, double, double);
bool SetVel(double, double, double);
bool CheckPosition(float x, float y, float z);

bool ThrowBottle(int cmd);

void ShowPosition(int delay);
bool Positioning(double pZ, int tim, int CenterX, int CenterY,
                 double MaxVel, int BottleLabel);
void Locating();
void Detect();
void GoHome();
void Land();
void TakeOff(double);
void SlowDescend(double, double, double, double);
void SlowMoveForward(double, double, double, double);

void Start_PID_WebTune();

// ── 内联工具函数 ─────────────────────────────
// 机体坐标系 -> ENU坐标系
inline void bodyToENU(double x_body, double y_body,
                      double& x_enu, double& y_enu)
{
    x_enu = x_body * sin(initial_yaw) + y_body * cos(initial_yaw);
    y_enu = -x_body * cos(initial_yaw) + y_body * sin(initial_yaw);
}

// ENU坐标系 -> 机体坐标系
inline void ENUToBody(double x_enu, double y_enu,
                      double& x_body, double& y_body)
{
    x_body = x_enu * sin(initial_yaw) - y_enu * cos(initial_yaw);
    y_body = x_enu * cos(initial_yaw) + y_enu * sin(initial_yaw);
}

#endif // __main_H