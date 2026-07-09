# control.cpp 逐行注释详解

---

## 文件概述

`control.cpp` 是**飞控控制指令的核心实现**，包含了所有与 PX4 飞控交互的函数。它是整个系统的"驱动器"层，负责：

1. **模式管理**：连接飞控、切换飞行模式（POSCTL/OFFBOARD/AUTO.LAND）
2. **电机控制**：解锁/上锁
3. **运动控制**：通过 ROS 服务发送位置/速度指令到 SetPoint 节点
4. **任务流程**：起飞、侦察、返航、降落、缓慢移动/下降
5. **状态检查**：检查位置是否到达、显示当前位置

---

## 第1行：文件注释

```cpp
// control.cpp — 飞控指令与控制逻辑实现
```

---

## 第2行：包含头文件

```cpp
#include "control.h"
```

---

## 第4行：全局变量定义

```cpp
ros::Time last_request;
```

**作用**：记录上次发送请求的时间戳，用于控制请求频率。

**对应关系**：`control.h` 中 `extern ros::Time last_request;` 的实际定义。

---

## 第7-21行：SetMode（带重试的模式切换）

```cpp
bool SetMode(std::string md)
{
    ROS_INFO("[PX4] 尝试设置 %s 模式", md.c_str());
    while (current_state.mode != md)
    {
        if (ros::Time::now() - last_request > ros::Duration(1.0))
        {
            PX4_SetMode(md);
            last_request = ros::Time::now();
        }
        ros::spinOnce();
    }
    ROS_INFO_STREAM("\033[32m" << "[PX4] 成功设置模式" << md.c_str() << "\033[0m");
    return true;
}
```

### 第9行：日志输出

```cpp
ROS_INFO("[PX4] 尝试设置 %s 模式", md.c_str());
```

- `md.c_str()`：将 `std::string` 转换为 C 风格字符串，因为 ROS_INFO 的格式化参数不支持 `std::string`

### 第10行：等待模式切换

```cpp
while (current_state.mode != md)
```

- `current_state` 是 `mavros_msgs::State` 类型，由 `state_cb` 回调更新
- 循环等待直到飞控的当前模式与目标模式一致

### 第12-15行：频率控制

```cpp
if (ros::Time::now() - last_request > ros::Duration(1.0))
{
    PX4_SetMode(md);
    last_request = ros::Time::now();
}
```

- 每次发送请求间隔至少 1 秒
- 防止在飞控未响应时以毫秒级频率疯狂发送请求

### 第17行：处理回调

```cpp
ros::spinOnce();
```

- 处理 ROS 回调，更新 `current_state`
- 如果缺少这一行，`current_state.mode` 永远不会更新，循环永远不会退出

---

## 第23-33行：PX4_SetMode（一次性模式切换）

```cpp
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
```

### 第25-26行：构建请求

```cpp
mavros_msgs::SetMode target_mode;
target_mode.request.custom_mode = md;
```

- `mavros_msgs::SetMode` 是 ROS 服务类型
- `custom_mode` 字段是要切换到的模式名称

### 第27行：调用服务

```cpp
if (set_mode_client.call(target_mode) && target_mode.response.mode_sent)
```

- `set_mode_client.call(target_mode)`：调用 MAVROS 的 `/mavros/set_mode` 服务
- `target_mode.response.mode_sent`：检查飞控是否确认接受了模式切换
- 两者都为 true 才返回成功

---

## 第36-57行：Arm（解锁）

```cpp
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
                ROS_INFO_STREAM("\033[32m" << "[PX4] Vehicle Armed!" << "\033[0m");
                ros::spinOnce();
            }
            last_request = ros::Time::now();
        }
    }
    ROS_INFO_STREAM("\033[32m" << "[PX4] Arm Successfully" << "\033[0m");
    return true;
}
```

### 第39行：解锁请求

```cpp
arm_cmd.request.value = true;
```

- `true` 表示解锁，`false` 表示上锁

### 第42行：等待解锁

```cpp
while (!current_state.armed)
```

- 循环等待直到 `current_state.armed` 变为 `true`

### 第44行：5 秒间隔

```cpp
if (ros::Time::now() - last_request > ros::Duration(5.0))
```

- 解锁请求的间隔是 **5 秒**（比模式切换的 1 秒更长）
- 因为解锁是安全关键操作，不应该过于频繁地尝试

### 第47行：调用服务

```cpp
if (arming_client.call(arm_cmd) && arm_cmd.response.success)
```

- 调用 MAVROS 的 `/mavros/cmd/arming` 服务
- 检查飞控是否回复 success

---

## 第60-86行：SetPoint（发送位置指令）

```cpp
bool SetPoint(double x, double y, double z)
{
    ROS_INFO("[SETPOINT] 发布BODY-Position: %f %f %f", x, y, z);
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
        ROS_ERROR("[SETPOINT] Failed to Set Point!");
        return false;
    }
}
```

### 第63行：创建服务请求

```cpp
px4_controller::position pos;
```

- `px4_controller::position` 是自定义的服务类型（定义在 `srv/position.srv`）

### 第65行：创建姿态四元数

```cpp
tf2::Quaternion q;
q.setRPY(0, 0, initial_yaw);
```

- roll=0, pitch=0, yaw=initial_yaw
- 飞机在瞄准过程中保持水平，只改变偏航角
- `setRPY` 从欧拉角创建四元数

### 第66行：设置模式

```cpp
pos.request.mode = 0;
```

- `mode = 0` 表示位置控制模式
- `SetPoint.cpp` 根据 `mode` 选择发布位置还是速度

### 第68行：坐标转换

```cpp
double ENU_x, ENU_y;
bodyToENU(x, y, ENU_x, ENU_y);
```

- 输入的 `(x, y)` 是机体坐标系
- 飞控使用 ENU 坐标系
- `bodyToENU()` 函数将机体坐标转为 ENU 坐标

### 第72-76行：设置四元数

```cpp
pos.request.qw = q.w();
pos.request.qx = q.x();
pos.request.qy = q.y();
pos.request.qz = q.z();
```

- 将 `tf2::Quaternion` 的四个分量分别赋值给服务请求

### 第77行：调用服务

```cpp
if (set_position_client.call(pos))
```

- 调用 `SendPosition` 服务
- `SetPoint.cpp` 节点接收这个请求后，发布到 MAVROS

---

## 第88-113行：SetVel（发送速度指令）

```cpp
bool SetVel(double vx, double vy, double pz)
{
    ROS_INFO("[SETVEL] 发布BODY-Velocity: %f %f Height %f", vx, vy, pz);
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
        ROS_ERROR("[SETVEL] Failed to Set Velocity!");
        return false;
    }
}
```

**与 SetPoint 的区别**：
- `mode = 1`（速度模式）
- 设置 `vx/vy` 而不是 `x/y`
- 高度用 `z` 字段

---

## 第116-132行：CheckPosition（位置检查）

```cpp
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
```

### 设计思路

- 检查位置是否在目标 ±0.1m 范围内
- 需要**连续 100 次**（约 100 帧）都在范围内才算到达
- 如果任何一帧超出范围，计数器归零
- 这种"连续确认"机制防止了位置抖动导致的误判

---

## 第134-149行：ConnectPX4（连接飞控）

```cpp
void ConnectPX4()
{
    while (ros::ok() && !current_state.connected)
    {
        ROS_INFO("[PX4] Attempt To Connect PX4");
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
}
```

### 第136-141行：等待连接

```cpp
while (ros::ok() && !current_state.connected)
{
    ROS_INFO("[PX4] Attempt To Connect PX4");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
}
```

- 每隔 1 秒检查一次飞控连接状态
- `ros::ok()` 确保 ROS 还在运行

### 第143行：发送初始位置

```cpp
SetPoint(0, 0, InitialHeight);
```

- 发送初始悬停指令到 (0, 0, 3.5m)
- 这样飞控在切换到 OFFBOARD 时就有目标位置可用

### 第145-148行：两步切换

```cpp
SetMode("POSCTL");
ros::Duration(1).sleep();
ros::spinOnce();
SetMode("OFFBOARD");
```

**为什么需要两步**：
- PX4 的安全设计：直接从某些模式切 OFFBOARD 可能失败
- POSCTL（位置控制模式）是一个安全的中间模式
- 两步切换（当前模式 → POSCTL → OFFBOARD）更可靠

---

## 第152-183行：TakeOff（起飞）

```cpp
void TakeOff(double waittime)
{
    SetPoint(0, 0, InitialHeight);
    if (abs(PX4_Position.z) <= 0.2 && abs(PX4_Position.x) <= 0.4 &&
        abs(PX4_Position.y) <= 0.4)
    {
        ROS_WARN("[PX4] 起飞偏差较小,X:%.2f Y:%.2f Z:%.2f", ...);
        ROS_WARN("[PX4] 2秒后起飞");
        ros::Duration(2.0).sleep();
        Arm();
        ros::spinOnce();
    }
    else if (abs(PX4_Position.x) <= 0.8 && abs(PX4_Position.y) <= 0.8)
    {
        ROS_WARN("[PX4] 初始位置偏差较大，慎飞,...");
        ROS_WARN("[PX4] 5秒后起飞");
        ros::Duration(5.0).sleep();
        Arm();
        ros::spinOnce();
    }
    else
    {
        ROS_ERROR("[PX4] 偏差过大，禁止飞行，重启飞控!!!!!");
        ROS_WARN("[PX4] 如不采取行动将在10s后起飞");
        ros::Duration(10).sleep();
        Arm();
    }
    ROS_INFO("FLY OK");
    ShowPosition(waittime);
}
```

### 分级安全检查逻辑

| 偏差范围 | 等待时间 | 日志级别 |
|---------|---------|---------|
| \|z\|≤0.2m 且 \|x\|,\|y\|≤0.4m | 2 秒 | WARN（黄色） |
| \|x\|,\|y\|≤0.8m | 5 秒 | WARN（黄色） |
| 其他（偏差过大） | 10 秒 | ERROR（红色） |

**设计意图**：
- 偏差小时快速起飞
- 偏差大时给操作员更多时间判断
- 即使偏差过大也不会禁止起飞，只是等待更久（"如不采取行动将在10s后起飞"）

---

## 第186-196行：ShowPosition（显示位置）

```cpp
void ShowPosition(int delay)
{
    while (delay)
    {
        ros::spinOnce();
        ROS_INFO("[PX4] Current Position %f %f %f",
                 PX4_Position.x, PX4_Position.y, PX4_Position.z);
        ros::Duration(1.0).sleep();
        delay--;
    }
}
```

- 每隔 1 秒打印一次当前位置
- 打印次数由 `delay` 控制
- 用于调试和状态监控

---

## 第198-212行：ThrowBottle（投放）

```cpp
bool ThrowBottle(int cmd)
{
    px4_controller::throwcmd command;
    command.request.cmd = cmd;
    if (throw_client.call(command))
    {
        ROS_INFO_STREAM("\033[32m" << "[Throw] Throw Successfully!" << "\033[0m");
        return true;
    }
    else
    {
        ROS_ERROR("[Throw] Failed to Throw!");
        return false;
    }
}
```

- 调用 `ThrowCmd` 服务
- `throw.cpp` 节点接收后通过 GPIO 控制舵机

---

## 第215-235行：Detect（侦察）

```cpp
void Detect()
{
    SetPoint(0, TakeofftoThrow, DetectHeight);
    ros::Duration(2.0).sleep();
    SetVel(0, 4, DetectHeight);
    ros::Duration(5.5).sleep();
    SetPoint(0, TakeofftoDetect, DetectHeight);
    ROS_INFO_STREAM("\033[32m" << "[Detect] 侦察开始" << "\033[0m");
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
    ROS_INFO_STREAM("\033[32m" << "[Detect] 侦察结束" << "\033[0m");
}
```

**飞行路径**：
```
① 飞到投放区上方 (0, 32.5m, 3.5m) → 2秒
② 全速前飞 (4m/s) → 5.5秒（约22米）
③ 到达侦察区 (0, 57.5m, 3.5m)
④ 悬停5秒确认位置
⑤ 左移 (-0.5m/s) → 6秒
⑥ 悬停2秒
⑦ 右移 (+0.5m/s) → 12秒
⑧ 左移 (-0.5m/s) → 6秒
⑨ 回到侦察区起点
```

**S 形扫描**：
- 左右移动形成 S 形航线，覆盖侦察区域
- 速度较慢（0.5m/s），确保视觉识别有足够时间

---

## 第238-245行：GoHome（返航）

```cpp
void GoHome()
{
    ROS_INFO_STREAM("\033[32m" << "[PX4] 返航" << "\033[0m");
    SetVel(0, -5, DetectHeight);
    ShowPosition(11);
    SetPoint(0, 0, DetectHeight);
    ShowPosition(4);
}
```

**流程**：
1. 以 5m/s 向后飞（Y 负方向）11 秒（约 55 米）
2. 飞到起飞点 (0, 0) 4 秒确认

---

## 第248-253行：Land（降落）

```cpp
void Land()
{
    SetMode("AUTO.LAND");
    ROS_INFO_STREAM("\033[32m" << "[PX4] LANDING....." << "\033[0m");
    ros::Duration(10).sleep();
}
```

- 切换到 PX4 的自动降落模式
- 等待 10 秒让飞机落地

---

## 第256-264行：SlowDescend（缓慢下降）

```cpp
void SlowDescend(double px, double py, double Lh, double Nh)
{
    for (float t = Lh; t >= Nh; t -= 0.1)
    {
        SetPoint(px, py, t);
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
}
```

- 从 `Lh`（高）逐级下降到 `Nh`（低）
- 步长 0.1m，步间隔 0.5 秒
- 每步都发送位置指令保持 `(px, py)` 位置

---

## 第267-275行：SlowMoveForward（缓慢前移）

```cpp
void SlowMoveForward(double px, double pz, double Ly, double Ny)
{
    for (float t = Ly; t <= Ny; t += 3)
    {
        SetPoint(px, t, pz);
        ros::Duration(0.3).sleep();
        ros::spinOnce();
    }
}
```

- 从 `Ly` 逐级前移到 `Ny`
- 步长 3m，步间隔 0.3 秒
- **注意**：当前在 `Locating()` 中被注释掉，未使用