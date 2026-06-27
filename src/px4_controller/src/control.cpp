// control.cpp — 飞控指令与控制逻辑实现
#include "control.h"

ros::Time last_request;

// ── 模式设置 ──────────────────────────────────
bool SetMode(std::string md)
{
    ROS_INFO("Attempt to set %s mode", md.c_str());
    while (current_state.mode != md)
    {
        if (ros::Time::now() - last_request > ros::Duration(1.0))
        {
            PX4_SetMode(md);
            last_request = ros::Time::now();
        }
        ros::spinOnce();
    }
    ROS_INFO("Set %s mode Successfully", md.c_str());
    return true;
}

bool PX4_SetMode(std::string md)
{
    mavros_msgs::SetMode target_mode;
    target_mode.request.custom_mode = md;
    if (set_mode_client.call(target_mode) && target_mode.response.mode_sent)
    {
        return true;
    }
    else
        return false;
}

// ── 解锁 ──────────────────────────────────────
bool Arm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ROS_INFO("Attempt to Arm");
    last_request = ros::Time::now();
    while (!current_state.armed)
    {
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            ROS_INFO("Attempt to Arm");
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                ros::spinOnce();
            }
            last_request = ros::Time::now();
        }
    }
    ROS_INFO("Arm Successfully");
    return true;
}

// ── 位置/速度设定 ─────────────────────────────
bool SetPoint(double x, double y, double z)
{
    ROS_INFO("发布BODY-Position: %f %f %f", x, y, z);
    px4_controller::position pos;
    tf2::Quaternion q;
    q.setRPY(0, 0, initial_yaw);
    pos.request.mode = 0;
    double ENU_x, ENU_y;
    bodyToENU(x, y, ENU_x, ENU_y);
    pos.request.x = ENU_x;
    pos.request.y = ENU_y;
    pos.request.z = z;
    pos.request.qw = q.w();
    pos.request.qx = q.x();
    pos.request.qy = q.y();
    pos.request.qz = q.z();
    pos.request.initial_yaw = initial_yaw;
    if (set_position_client.call(pos))
    {
        return true;
    }
    else
    {
        ROS_INFO("Failed to Set Point!");
        return false;
    }
}

bool SetVel(double vx, double vy, double pz)
{
    ROS_INFO("发布BODY-Velocity: %f %f Height %f", vx, vy, pz);
    px4_controller::position vel;
    vel.request.mode = 1;
    tf2::Quaternion q;
    q.setRPY(0, 0, initial_yaw);
    double ENU_vx, ENU_vy;
    bodyToENU(vx, vy, ENU_vx, ENU_vy);
    vel.request.vx = ENU_vx;
    vel.request.vy = ENU_vy;
    vel.request.z = pz;
    vel.request.qw = q.w();
    vel.request.qx = q.x();
    vel.request.qy = q.y();
    vel.request.qz = q.z();
    if (set_position_client.call(vel))
    {
        return true;
    }
    else
    {
        ROS_INFO("Failed to Set Velocity!");
        return false;
    }
}

// ── 位置检查 ──────────────────────────────────
bool CheckPosition(float x, float y, float z)
{
    int sametimes = 0;
    while (sametimes < 100)
    {
        if (PX4_Position.x > (x - 0.1) && PX4_Position.x < (x + 0.1) &&
            PX4_Position.y > (y - 0.1) && PX4_Position.y < (y + 0.1) &&
            PX4_Position.z > (z - 0.1) && PX4_Position.z < (z + 0.1))
        {
            sametimes++;
        }
        else
            sametimes = 0;
    }
    ROS_INFO("Position Stablized %f %f %f", x, y, z);
    return true;
}

// ── 起飞流程 ──────────────────────────────────
void TakeOff(double waittime)
{
    while (ros::ok() && !current_state.connected)
    {
        ROS_INFO("Attempt To Connect PX4");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ros::Duration(1).sleep();
    SetPoint(0, 0, InitialHeight);
    ros::spinOnce();
    SetMode("POSCTL");
    ros::Duration(1).sleep();
    ros::spinOnce();
    SetMode("OFFBOARD");

    if (abs(PX4_Position.z) <= 0.2 && abs(PX4_Position.x) <= 0.4 &&
        abs(PX4_Position.y) <= 0.4)
    {
        ROS_WARN("起飞偏差较小,X:%.2f Y:%.2f Z:%.2f",
                 PX4_Position.x, PX4_Position.y, PX4_Position.z);
        ROS_WARN("2秒后起飞");
        ros::Duration(2.0).sleep();
        Arm();
        ros::spinOnce();
    }
    else if (abs(PX4_Position.x) <= 0.8 && abs(PX4_Position.y) <= 0.8)
    {
        ROS_WARN("初始位置偏差较大，慎飞,X:%.2f Y:%.2f Z:%.2f",
                 PX4_Position.x, PX4_Position.y, PX4_Position.z);
        ROS_WARN("5秒后起飞");
        ros::Duration(5.0).sleep();
        Arm();
        ros::spinOnce();
    }
    else
    {
        ROS_ERROR("偏差过大，禁止飞行，重启飞控!!!!!");
        ROS_WARN("如不采取行动将在10s后起飞");
        ros::Duration(10).sleep();
        Arm();
    }
    ROS_INFO("FLY OK");
    ShowPosition(waittime);
}

// ── 辅助函数 ──────────────────────────────────
void ShowPosition(int delay)
{
    while (delay)
    {
        ros::spinOnce();
        ROS_INFO("Current Position %f %f %f",
                 PX4_Position.x, PX4_Position.y, PX4_Position.z);
        ros::Duration(1.0).sleep();
        delay--;
    }
}

bool ThrowBottle(int cmd)
{
    px4_controller::throwcmd command;
    command.request.cmd = cmd;
    if (throw_client.call(command))
    {
        return true;
    }
    else
    {
        ROS_ERROR("Failed to Throw!");
        return false;
    }
}

// ── 侦察流程 ──────────────────────────────────
void Detect()
{
    SetPoint(0, TakeofftoThrow, DetectHeight);
    ros::Duration(2.0).sleep();
    SetVel(0, 4, DetectHeight);
    ros::Duration(5.5).sleep();
    SetPoint(0, TakeofftoDetect, DetectHeight);
    ROS_INFO("侦察开始");
    ShowPosition(5);
    SetVel(-0.5, 0, DetectHeight);
    ShowPosition(6);
    SetVel(0, 0, DetectHeight);
    ros::Duration(2.0).sleep();
    SetVel(0.5, 0, DetectHeight);
    ShowPosition(12);
    SetVel(-0.5, 0, DetectHeight);
    ShowPosition(6);
    SetPoint(0, TakeofftoDetect, DetectHeight);
    ShowPosition(2);
    ROS_INFO("侦察结束");
}

// ── 返航 ──────────────────────────────────────
void GoHome()
{
    ROS_INFO("返航");
    SetVel(0, -5, DetectHeight);
    ShowPosition(11);
    SetPoint(0, 0, DetectHeight);
    ShowPosition(4);
}

// ── 降落 ──────────────────────────────────────
void Land()
{
    SetMode("AUTO.LAND");
    ROS_INFO("LANDING.....");
    ros::Duration(10).sleep();
}

// ── 缓慢下降 ──────────────────────────────────
void SlowDescend(double px, double py, double Lh, double Nh)
{
    for (float t = Lh; t >= Nh; t -= 0.1)
    {
        SetPoint(px, py, t);
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
}

// ── 缓慢前移 ──────────────────────────────────
void SlowMoveForward(double px, double pz, double Ly, double Ny)
{
    for (float t = Ly; t <= Ny; t += 3)
    {
        SetPoint(px, t, pz);
        ros::Duration(0.3).sleep();
        ros::spinOnce();
    }
}