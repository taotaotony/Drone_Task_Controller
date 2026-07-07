# PX4_Controller 项目 ROS 通信详解

## 目录

1. [头文件保护宏](#一头文件保护宏)
2. [项目库文件分类](#二项目库文件分类)
3. [ROS 通信基础概念](#三ros-通信基础概念)
4. [项目中所有通信分类](#四项目中所有通信分类)
5. [自定义通信详解](#五自定义通信详解)
6. [代码架构与数据流](#六代码架构与数据流)
7. [完整调用链示例](#七完整调用链示例)
8. [常见问题解答](#八常见问题解答)

---

## 一、头文件保护宏

### 1.1 代码示例

```cpp
#ifndef __main_H
#define __main_H

// ... 头文件内容 ...

#endif // __main_H
```

### 1.2 作用

**防止头文件被重复包含**，避免编译错误。

### 1.3 工作原理

- **第一次包含**：`__main_H` 未定义 → 执行 `#define __main_H` → 编译头文件内容
- **第二次包含**：`__main_H` 已定义 → 跳过整个头文件内容

### 1.4 命名规范

- 文件名大写：`main` → `MAIN`
- 用下划线替换点号：`main.h` → `MAIN_H`
- 前后加下划线避免命名冲突：`_MAIN_H_` 或 `__MAIN_H`

### 1.5 现代替代方案

C++17 以后可以使用更简洁的 `#pragma once`：

```cpp
#pragma once  // 放在文件最开头

#include <ros/ros.h>
// ... 其他内容
```

**优点**：更简洁，避免命名冲突  
**缺点**：部分老旧编译器不支持

---

## 二、项目库文件分类

### 2.1 ROS 系统库（现成的）

```cpp
#include <ros/ros.h>           // ROS 核心功能
#include <ros/package.h>       // ROS 包路径管理
```

### 2.2 MAVROS 飞控通信库（现成的）

```cpp
#include <mavros_msgs/SetMode.h>      // 设置飞行模式
#include <mavros_msgs/CommandBool.h>   // 解锁/上锁命令
#include <mavros_msgs/State.h>         // 飞控状态信息
```

**说明**：这些是 PX4 飞控的 ROS 接口，用于与飞控通信。

### 2.3 标准 C++ 库（现成的）

```cpp
#include <string>      // 字符串处理
#include <cstdlib>     // C 标准库
#include <sstream>     // 字符串流
#include <random>      // 随机数
#include <thread>      // 多线程
```

### 2.4 自定义消息/服务（你自己写的）

```cpp
#include <px4_controller/position.h>    // 自定义服务（position.srv）
#include <px4_controller/throwcmd.h>    // 自定义服务（throwcmd.srv）
#include <px4_controller/tbag.h>        // 自定义消息（tbag.msg）
```

**说明**：这些是在 `src/px4_controller/srv/` 和 `src/px4_controller/msg/` 目录下定义的文件。

### 2.5 ROS 标准消息类型（现成的）

```cpp
#include <std_msgs/String.h>              // 字符串消息
#include <geometry_msgs/PoseStamped.h>    // 位姿消息
#include <sensor_msgs/Imu.h>              // IMU 数据
#include <sensor_msgs/Range.h>            // 距离传感器
#include <geometry_msgs/TwistStamped.h>   // 速度消息
```

### 2.6 TF2 坐标变换库（现成的）

```cpp
#include <tf2/LinearMath/Quaternion.h>      // 四元数
#include <tf2/LinearMath/Matrix3x3.h>       // 3x3 矩阵
```

**说明**：用于坐标系转换（如机体坐标系 ↔ ENU 坐标系）。

### 2.7 第三方库（现成的）

```cpp
#include <httplib.h>  // HTTP 客户端/服务器库（单头文件库）
```

**说明**：用于 Web 服务器通信（PID 调参界面）。

### 2.8 总结

| 类型 | 文件 | 说明 |
|------|------|------|
| **你自己写的** | `position.h`、`throwcmd.h`、`tbag.h` | 自定义消息/服务 |
| **现成的库** | ROS 系统库、MAVROS 库、标准 C++ 库、ROS 消息类型、TF2、httplib | 第三方或系统提供 |

---

## 三、ROS 通信基础概念

### 3.1 Topic（话题）- 发布/订阅模式

```
发布者（Publisher） → 话题（Topic） → 订阅者（Subscriber）
```

**特点**：
- 异步通信
- 一对多
- 持续数据流

**类比**：广播电台
- 电台持续广播（发布者）
- 多个收音机可以同时收听（订阅者）

**用途**：传感器数据、状态信息、持续更新的数据

**示例**：
```cpp
// 发布者
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
pub.publish(msg);

// 订阅者
ros::Subscriber sub = nh.subscribe("chatter", 10, callback);
void callback(const std_msgs::String::ConstPtr& msg) {
    // 处理消息
}
```

### 3.2 Service（服务）- 请求/响应模式

```
客户端（Client） → 服务（Service） → 服务器（Server）
```

**特点**：
- 同步通信
- 一对一
- 请求-响应式

**类比**：打电话
- 你打过去（客户端）
- 对方接听并回复（服务端）

**用途**：控制指令、需要确认的操作

**示例**：
```cpp
// 客户端
ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("set_bool");
std_srvs::SetBool srv;
srv.request.data = true;
if (client.call(srv)) {
    // 处理响应
    bool success = srv.response.success;
}

// 服务端
ros::ServiceServer server = nh.advertiseService("set_bool", callback);
bool callback(std_srvs::SetBool::Request& req,
              std_srvs::SetBool::Response& res) {
    // 处理请求
    res.success = true;
    return true;
}
```

### 3.3 Action（动作）- 带反馈的目标任务

**特点**：
- 异步通信
- 可取消
- 有进度反馈

**用途**：长时间运行的任务（如导航到目标点）

**说明**：你的代码中未使用 Action

### 3.4 Topic vs Service 对比

| 特性 | Topic（话题） | Service（服务） |
|------|--------------|----------------|
| **模式** | 发布/订阅 | 客户端/服务端 |
| **通信方式** | 异步 | 同步 |
| **数据流** | 持续流 | 一次性请求-响应 |
| **类比** | 广播电台 | 打电话 |
| **你的代码中的例子** | IR（视觉数据） | SendPosition、ThrowCmd |

---

## 四、项目中所有通信分类

### 4.1 MAVROS 自带的通信（你直接使用）

这些是 MAVROS 包提供的标准接口，你只需要调用即可。

#### 4.1.1 服务（Services）- 你主动调用

| 服务名称 | 话题路径 | 消息类型 | 用途 | 调用位置 |
|---------|---------|---------|------|---------|
| 解锁服务 | `/mavros/cmd/arming` | `mavros_msgs::CommandBool` | 解锁/上锁无人机电机 | `Arm()` |
| 模式设置 | `/mavros/set_mode` | `mavros_msgs::SetMode` | 切换飞行模式 | `SetMode()` |

**调用示例**（`control.cpp` 第 23-33 行）：
```cpp
bool PX4_SetMode(std::string md)
{
    mavros_msgs::SetMode target_mode;
    target_mode.request.custom_mode = md;  // 请求设置模式
    if (set_mode_client.call(target_mode) && target_mode.response.mode_sent)
    {
        return true;  // 飞控回复：模式设置成功
    }
}
```

#### 4.1.2 话题（Topics）- 你被动接收

| 话题名称 | 消息类型 | 数据内容 | 回调函数 | 存储变量 |
|---------|---------|---------|---------|---------|
| `mavros/state` | `mavros_msgs::State` | 连接状态、 armed、 mode | `state_cb()` | `current_state` |
| `/mavros/local_position/pose` | `geometry_msgs::PoseStamped` | 位置 x,y,z + 四元数 | `pos_cb()` | `PX4_Position` |
| `/mavros/local_position/velocity_local` | `geometry_msgs::TwistStamped` | 速度 vx,vy,vz | `vel_cb()` | `PX4_Velocity` |
| `/mavros/imu/data` | `sensor_msgs::Imu` | IMU 数据（加速度、角速度、姿态） | `imu_cb()` | `imu_msg` |
| `/mavros/distance_sensor/hrlv_ez4_pub` | `sensor_msgs::Range` | 激光雷达测距 | `lidar_cb()` | - |

**数据流示例**（`communication.cpp` 第 42-71 行）：
```cpp
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // MAVROS 发布位置数据 → 你的回调函数接收
    double bodyX, bodyY;
    ENUToBody(msg->pose.position.x, msg->pose.position.y, bodyX, bodyY);
    
    // 存储到全局变量
    PX4_Position.x = bodyX;
    PX4_Position.y = bodyY;
    PX4_Position.z = msg->pose.position.z;
}
```

### 4.2 你自己定义的自定义通信

#### 4.2.1 自定义服务（Services）

##### ① 位置/速度设置服务 - `SendPosition`

**服务定义**（`srv/position.srv`）：
```
# 请求（Request）
int32 mode          # 0=位置模式, 1=速度模式
float64 x           # X坐标或速度
float64 y           # Y坐标或速度
float64 z           # Z坐标或高度
float64 qw          # 四元数 w
float64 qx          # 四元数 x
float64 qy          # 四元数 y
float64 qz          # 四元数 z
float64 vx          # X速度（可选）
float64 vy          # Y速度（可选）
float64 initial_yaw # 初始偏航角
---
# 响应（Response）
bool success        # 是否成功
```

##### ② 投放服务 - `ThrowCmd`

**服务定义**（`srv/throwcmd.srv`）：
```
# 请求
int32 cmd    # 投放命令
---
# 响应
bool success
```

#### 4.2.2 自定义消息（Message）

##### 视觉识别结果消息 - `tbag`

**消息定义**（`msg/tbag.msg`）：
```
# 目标1
int32 Target1_Exist      # 是否存在
float64 Target1_PR      # PR值（识别置信度）
int32 Target1_LU_x       # 左上角 x
int32 Target1_LU_y       # 左上角 y
int32 Target1_RU_x       # 右上角 x
int32 Target1_RU_y       # 右上角 y
int32 Target1_RD_x       # 右下角 x
int32 Target1_RD_y       # 右下角 y
int32 Target1_LD_x       # 左下角 x
int32 Target1_LD_y       # 左下角 y

# 目标2（同上）
# 目标3（同上）
```

### 4.3 HTTP 网络通信（非 ROS）

使用 `httplib` 库在端口 8880 提供 Web 服务：

| URL 路径 | 用途 | 处理函数 |
|---------|------|---------|
| `/setpid?p=0.01&i=0&d=0&limit=0.15&deadzone=0.01` | PID 调参 | `handle_pid()` |
| `/setstat?state=xxx` | 状态机控制 | `drone_.UpdateState()` |
| `/setgoal?x=0&y=10&z=3` | 设置目标点 | `drone_.UpdateGoal()` |
| `/throw?cmd=1` | 投放控制 | `drone_.Throw()` |

**这不是 ROS 通信**，而是独立的 HTTP 服务器，用于远程调试。

---

## 五、自定义通信详解

### 5.1 SendPosition 服务（位置/速度设置）

#### 5.1.1 通信关系

```
main 节点（客户端） → SetPoint 节点（服务端）
```

#### 5.1.2 服务端（发布者/服务器）

**文件**：`src/px4_controller/src/SetPoint.cpp`

**节点名**：`SetPoint`

**服务注册位置**（第 48 行）：
```cpp
ros::ServiceServer server = nh.advertiseService("SendPosition", SetPosition);
```

**回调函数**：`SetPosition()`（第 15-34 行）
```cpp
bool SetPosition(px4_controller::position::Request& req,
                 px4_controller::position::Response& resp)
{
    pub_mode = req.mode;  // 0=位置模式, 1=速度模式
    pose.pose.position.x = req.x;
    pose.pose.position.y = req.y;
    pose.pose.position.z = req.z;
    pose.pose.orientation.w = req.qw;
    pose.pose.orientation.x = req.qx;
    pose.pose.orientation.y = req.qy;
    pose.pose.orientation.z = req.qz;
    vel_msg.velocity.x = req.vx;
    vel_msg.velocity.y = req.vy;
    vel_msg.velocity.z = 0;
    vel_msg.position.z = req.z;
    initial_yaw = req.initial_yaw;
    
    ROS_INFO("发布ENU-Position: %f %f %f, Vel %f %f",
             req.x, req.y, req.z, req.vx, req.vy);
    
    resp.success = true;
    return true;
}
```

**主循环**（第 63-77 行）：
```cpp
while(ros::ok())
{
    if(pub_mode == 0)  // 位置模式
    {
        local_pose_pub.publish(pose);  // 发布到 /mavros/setpoint_position/local
    }
    else  // 速度模式
    {
        vel_msg.header.stamp = ros::Time::now();
        vel_pub.publish(vel_msg);  // 发布到 /mavros/setpoint_raw/local
    }
    
    ros::spinOnce();  // 处理 Service 请求
    loop_rate.sleep();
}
```

#### 5.1.3 客户端（订阅者/调用者）

**文件**：`src/px4_controller/src/main.cpp`

**节点名**：`main`

**创建客户端**（第 46 行）：
```cpp
set_position_client = nh.serviceClient<px4_controller::position>("SendPosition");
```

**调用位置**：`control.cpp` 中的 `SetPoint()` 和 `SetVel()` 函数

```cpp
bool SetPoint(double x, double y, double z)
{
    px4_controller::position pos;  // ← 创建 Request
    
    pos.request.mode = 0;  // 位置模式
    pos.request.x = ENU_x;
    pos.request.y = ENU_y;
    pos.request.z = z;
    pos.request.qw = q.w();
    pos.request.qx = q.x();
    pos.request.qy = q.y();
    pos.request.qz = q.z();
    pos.request.initial_yaw = initial_yaw;
    
    if (set_position_client.call(pos))  // ← 发送请求
    {
        return true;
    }
}
```

#### 5.1.4 数据流

```
1. main 节点调用 SetPoint(0, 0, 3.5)
   ↓
2. control.cpp 创建 position::Request
   - mode = 0 (位置模式)
   - x, y, z = 目标坐标
   - qw, qx, qy, qz = 四元数
   - initial_yaw = 初始偏航角
   ↓
3. 通过 ROS Service 发送到 SetPoint 节点
   ↓
4. SetPoint.cpp 的 SetPosition() 回调接收
   ↓
5. 解析数据，更新全局变量 pose 和 vel_msg
   ↓
6. SetPoint 节点通过 Publisher 发布到 MAVROS
   - 位置模式：发布到 /mavros/setpoint_position/local
   - 速度模式：发布到 /mavros/setpoint_raw/local
   ↓
7. MAVROS 接收并发送给 PX4 飞控
```

#### 5.1.5 数据来源

- **来自**：`main.cpp` 中的函数调用（`SetPoint()`、`SetVel()`）
- **数据内容**：
  - `mode`：0=位置模式, 1=速度模式
  - `x, y, z`：目标位置或高度
  - `vx, vy`：目标速度（速度模式）
  - `qw, qx, qy, qz`：姿态四元数
  - `initial_yaw`：初始偏航角

### 5.2 ThrowCmd 服务（投放控制）

#### 5.2.1 通信关系

```
main 节点（客户端） → throw 节点（服务端）
```

#### 5.2.2 服务端（发布者/服务器）

**文件**：`src/px4_controller/src/throw.cpp`

**节点名**：`throw`

**服务注册位置**（第 82 行）：
```cpp
ros::ServiceServer server = nh.advertiseService("ThrowCmd", ThrowBottle);
```

**回调函数**：`ThrowBottle()`（第 140-147 行）
```cpp
bool ThrowBottle(px4_controller::throwcmd::Request& req,
                 px4_controller::throwcmd::Response& resp)
{
    int cmd = req.cmd;
    // TODO: 实际控制舵机投放
    resp.success = true;
    return true;
}
```

#### 5.2.3 客户端（订阅者/调用者）

**文件**：`src/px4_controller/src/main.cpp`

**节点名**：`main`

**创建客户端**（第 47 行）：
```cpp
throw_client = nh.serviceClient<px4_controller::throwcmd>("ThrowCmd");
```

**调用位置**：`control.cpp` 中的 `ThrowBottle()` 函数（第 198-212 行）
```cpp
bool ThrowBottle(int cmd)
{
    px4_controller::throwcmd command;
    command.request.cmd = cmd;
    if (throw_client.call(command))
    {
        ROS_INFO("[Throw] Throw Successfully!");
        return true;
    }
}
```

#### 5.2.4 数据流

```
1. main 节点调用 ThrowBottle(1)
   ↓
2. control.cpp 创建 throwcmd::Request
   - cmd = 1 (投放命令)
   ↓
3. 通过 ROS Service 发送到 throw 节点
   ↓
4. throw.cpp 的 ThrowBottle() 回调接收
   ↓
5. 解析 cmd 参数
   ↓
6. 控制舵机（目前代码未完全实现，只有占位符）
   - Servo1: 物理引脚 15
   - Servo2: 物理引脚 33
   ↓
7. 返回 success = true
```

#### 5.2.5 数据来源

- **来自**：`main.cpp` 中的函数调用（`ThrowBottle(cmd)`）
- **数据内容**：
  - `cmd`：投放命令（整数，具体含义未定义）

### 5.3 IR 话题（视觉识别结果）

#### 5.3.1 通信关系

```
cvisual 节点（发布者） → main 节点（订阅者）
```

#### 5.3.2 发布者

**文件**：`src/px4_controller/src/cvisual.cpp`

**节点名**：`visual_node`

**发布位置**（第 594 行）：
```cpp
ros::Publisher pub = nh.advertise<px4_controller::tbag>("IR", 1);
```

**发布位置**（第 754 行，主循环中）：
```cpp
pub.publish(make_visual_msg(tracked));
```

**数据生成过程**：
```
摄像头采集（第 710 行）
  ↓
capture_thread 线程取帧（第 366-380 行）
  ↓
inference_thread 线程推理（第 499-546 行）
  - CUDA 预处理
  - TensorRT 推理（YOLOv8）
  - D2H 拷贝
  ↓
主线程 CPU 后处理（第 745-750 行）
  - parse_yolov8()：解码输出
  - apply_nms()：非极大值抑制
  - tracker.update()：SORT 跟踪
  ↓
make_visual_msg() 打包数据（第 556-587 行）
  ↓
发布到 IR 话题
```

#### 5.3.3 订阅者

**文件**：`src/px4_controller/src/main.cpp`

**节点名**：`main`

**订阅位置**（第 49 行）：
```cpp
visual_sub = nh.subscribe("IR", 1, visual_cb);
```

**回调函数**：`communication.cpp` 中的 `visual_cb()`（第 73-107 行）
```cpp
void visual_cb(const px4_controller::tbag::ConstPtr& msg)
{
    VisualData.Target1_Exist = msg->Target1_Exist;
    VisualData.Target1_PR = msg->Target1_PR;
    VisualData.Target1_LU_x = msg->Target1_LU_x;
    // ... 复制所有目标数据
}
```

#### 5.3.4 数据流

```
1. cvisual 节点：
   - 摄像头采集图像（1920x1200, 120fps）
   - YOLOv8 推理检测桶目标
   - SORT 跟踪器跟踪目标
   - make_visual_msg() 打包最多3个目标
   - 发布到 IR 话题（30Hz）
   ↓
2. main 节点：
   - visual_cb() 回调接收
   - 解析 tbag 消息
   - 存储到 VisualData 结构体
   ↓
3. 其他模块使用 VisualData：
   - aim.cpp：瞄准计算
   - control.cpp：投放决策
```

#### 5.3.5 数据来源

- **来自**：摄像头图像（`/dev/video0`，GStreamer 采集）
- **数据处理**：
  - YOLOv8 目标检测（TensorRT 推理）
  - SORT 多目标跟踪（Kalman 滤波 + IOU 匹配）
  - 提取目标的像素坐标和置信度
- **数据内容**（`tbag.msg`）：
  ```
  目标1：
    Target1_Exist: 是否存在 (0/1)
    Target1_PR: 置信度 (0~1)
    Target1_LU_x, Target1_LU_y: 左上角像素坐标
    Target1_RU_x, Target1_RU_y: 右上角像素坐标
    Target1_RD_x, Target1_RD_y: 右下角像素坐标
    Target1_LD_x, Target1_LD_y: 左下角像素坐标
  
  目标2、目标3：同上
  ```

---

## 六、代码架构与数据流

### 6.1 项目编译结构

**CMakeLists.txt 第 155 行**：
```cmake
add_executable(Main 
    src/main.cpp 
    src/WebServer.cpp 
    src/TelemetryBroadcaster.cpp 
    src/communication.cpp 
    src/control.cpp      // ← 这里！
    src/aim.cpp 
    src/Drone.cpp 
    src/calibration.cpp)
```

**关键理解**：
- `main.cpp`、`control.cpp`、`Drone.cpp` 等**所有文件被编译成一个可执行文件**
- 这个可执行文件叫 `Main`
- 运行时，**只有一个 ROS 节点**，节点名是 `main`（在 main.cpp 第 23 行定义）

### 6.2 文件之间的关系

```
Main（可执行文件）
├── main.cpp（主函数入口，ROS 节点初始化）
├── control.cpp（控制函数：SetPoint, TakeOff, Land...）
├── Drone.cpp（状态机：管理飞行状态）
├── communication.cpp（ROS 通信：订阅者、回调函数）
├── aim.cpp（瞄准逻辑）
├── WebServer.cpp（HTTP 服务器）
├── TelemetryBroadcaster.cpp（UDP 遥测广播）
└── ...
```

**类比**：
- 就像一个大项目有多个 `.cpp` 文件
- `main.cpp` 是主函数
- `control.cpp` 是一些工具函数
- 它们**在同一个程序中**，可以互相调用

### 6.3 调用关系示例

```cpp
// main.cpp
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main");  // 创建 ROS 节点
    Drone drone;                     // 创建状态机对象
    // ...
    while (ros::ok())
    {
        drone.HandleState();  // 调用 Drone 的状态机
        ros::spinOnce();
    }
}

// Drone.cpp
void Drone::ExecuteTakeOff()
{
    TakeOff(5);  // ← 调用 control.cpp 中的 TakeOff() 函数
}

// control.cpp
void TakeOff(double waittime)
{
    SetPoint(0, 0, InitialHeight);  // ← 调用 control.cpp 中的 SetPoint() 函数
}
```

### 6.4 整体架构图

```
┌─────────────────────────────────────────────────────────────┐
│                      Main（可执行文件）                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  main.cpp（主函数）                                    │  │
│  │  - ROS 节点初始化                                      │  │
│  │  - 创建 Drone 对象                                    │  │
│  │  - 主循环：drone.HandleState()                        │  │
│  └──────────────────────────────────────────────────────┘  │
│                            ↓                                │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Drone.cpp（状态机）                                   │  │
│  │  - 管理飞行状态（TAKEOFF, GOAL, RETURN...）           │  │
│  │  - 调用 control.cpp 的函数                            │  │
│  └──────────────────────────────────────────────────────┘  │
│                            ↓                                │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  control.cpp（控制函数）                               │  │
│  │  - SetPoint()：设置目标位置                           │  │
│  │  - TakeOff()：起飞流程                                │  │
│  │  - Land()：降落流程                                   │  │
│  │  - 创建 Service Request 并发送                        │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                          │
                          │ ROS Service 通信
                          ↓
┌─────────────────────────────────────────────────────────────┐
│  SetPoint 节点（独立可执行文件）                              │
│  - 服务端：接收 SendPosition 请求                           │
│  - 发布者：持续发布到 MAVROS Topic                          │
└─────────────────────────────────────────────────────────────┘
                          │
                          │ ROS Topic 发布
                          ↓
┌─────────────────────────────────────────────────────────────┐
│  MAVROS（订阅者）                                            │
│  - 订阅 /mavros/setpoint_position/local                     │
│  - 转发给 PX4 飞控                                          │
└─────────────────────────────────────────────────────────────┘
```

---

## 七、完整调用链示例

### 7.1 场景：起飞流程

```
时间线：
─────────────────────────────────────────────────────────────

T1: main.cpp（主循环）
    while (ros::ok())
    {
        drone.HandleState();  // 调用状态机
    }
    ↓
    Drone::HandleState()
    ↓
    current_state_ = DroneState_TAKEOFF
    ↓
    ExecuteTakeOff()

T2: Drone.cpp - ExecuteTakeOff()
    {
        TakeOff(5);  // 调用 control.cpp
    }
    ↓
    参数：waittime = 5（显示位置5秒）

T3: control.cpp - TakeOff() 函数
    {
        SetPoint(0, 0, InitialHeight);
        // 参数来源：
        //   x = 0（硬编码）
        //   y = 0（硬编码）
        //   z = InitialHeight = 3.5（宏定义）
    }
    ↓
    调用 SetPoint(0, 0, 3.5)

T4: control.cpp - SetPoint() 函数
    {
        // 1. 创建 Request 对象
        px4_controller::position pos;
        
        // 2. 填充数据
        pos.request.mode = 0;  // 位置模式
        pos.request.x = ENU_x; // 转换后的X坐标
        pos.request.y = ENU_y; // 转换后的Y坐标
        pos.request.z = z;     // 高度 = 3.5
        pos.request.qw = q.w(); // 四元数
        pos.request.qx = q.x();
        pos.request.qy = q.y();
        pos.request.qz = q.z();
        pos.request.initial_yaw = initial_yaw;
        
        // 3. 发送请求
        set_position_client.call(pos);
    }
    ↓
    发送 ROS Service 请求到 SetPoint 节点

T5: SetPoint.cpp - SetPosition() 回调函数
    {
        // 1. 接收 Request
        pub_mode = req.mode;  // 0
        pose.pose.position.x = req.x;
        pose.pose.position.y = req.y;
        pose.pose.position.z = req.z;  // 3.5
        
        // 2. 更新全局变量
        initial_yaw = req.initial_yaw;
        
        // 3. 返回响应
        resp.success = true;
    }
    ↓
    返回 Response

T6: SetPoint.cpp - main() 主循环
    while(ros::ok())
    {
        if(pub_mode == 0)  // 位置模式
        {
            local_pose_pub.publish(pose);
            // 发布到 /mavros/setpoint_position/local
        }
        
        ros::spinOnce();  // 处理 Service 请求
        loop_rate.sleep();
    }
    ↓
    持续发布目标位置 (0, 0, 3.5)

T7: MAVROS（订阅者）
    收到 /mavros/setpoint_position/local 消息
    ↓
    转发给 PX4 飞控（通过 MAVLink 协议）

T8: PX4 飞控
    接收目标位置 (0, 0, 3.5)
    ↓
    控制电机，飞向目标点

─────────────────────────────────────────────────────────────
```

### 7.2 场景：飞往目标点流程（Web 远程控制）

```
时间线：
─────────────────────────────────────────────────────────────

T1: 用户发送 HTTP 请求
    http://<ip>:8880/setgoal?mode=0&px=10&py=20&pz=3
    ↓
    参数：
    - mode = 0（位置模式）
    - px = 10（X坐标）
    - py = 20（Y坐标）
    - pz = 3（高度）

T2: WebServer.cpp - 路由处理
    svr.Get("/setgoal", [this](const httplib::Request& req, httplib::Response& res) {
        drone_.UpdateGoal(req, res);  // 调用 Drone 的方法
    });
    ↓
    转发到 Drone::UpdateGoal()

T3: Drone.cpp - UpdateGoal() 函数
    {
        // 1. 验证当前状态
        if(current_state_ != DroneState_GOAL)
        {
            res.set_content("Denied: Switch to GOAL Mode First!");
            return;
        }
        
        // 2. 解析参数
        int new_mode = std::stoi(req.get_param_value("mode"));  // 0
        double new_px = std::stod(req.get_param_value("px"));   // 10
        double new_py = std::stod(req.get_param_value("py"));   // 20
        double new_pz = std::stod(req.get_param_value("pz"));   // 3
        
        // 3. 存储到 Goal 结构体
        Goal.mode = new_mode;
        Goal.px = new_px;
        Goal.py = new_py;
        Goal.pz = new_pz;
    }
    ↓
    存储目标点：(10, 20, 3)

T4: main.cpp（主循环）
    while (ros::ok())
    {
        drone.HandleState();
    }
    ↓
    Drone::HandleState()
    ↓
    current_state_ = DroneState_GOAL
    ↓
    ExecuteGoal()

T5: Drone.cpp - ExecuteGoal()
    {
        if(Goal.mode == 0)
        {
            SetPoint(Goal.px, Goal.py, Goal.pz);
            // 参数：x=10, y=20, z=3
        }
    }
    ↓
    调用 control.cpp 的 SetPoint()

T6: control.cpp - SetPoint() 函数
    {
        px4_controller::position pos;
        pos.request.mode = 0;
        pos.request.x = ENU_x;  // 转换后的坐标
        pos.request.y = ENU_y;
        pos.request.z = 3;
        // ... 填充四元数
        set_position_client.call(pos);
    }
    ↓
    发送 Service 请求

T7: SetPoint.cpp - SetPosition() 回调
    {
        pub_mode = req.mode;
        pose.pose.position.x = req.x;  // 10
        pose.pose.position.y = req.y;  // 20
        pose.pose.position.z = req.z;  // 3
        resp.success = true;
    }
    ↓
    更新全局变量

T8: SetPoint.cpp - main() 主循环
    while(ros::ok())
    {
        if(pub_mode == 0)
        {
            local_pose_pub.publish(pose);
            // 持续发布 (10, 20, 3)
        }
    }
    ↓
    发布到 MAVROS

T9: MAVROS → PX4 飞控
    控制无人机飞向 (10, 20, 3)

─────────────────────────────────────────────────────────────
```

### 7.3 场景：视觉识别流程

```
时间线：
─────────────────────────────────────────────────────────────

T1: cvisual.cpp - 摄像头采集
    cv::VideoCapture cap;
    cap >> frame;  // 采集一帧图像
    ↓
    图像尺寸：1920x1200
    帧率：120fps

T2: cvisual.cpp - capture_thread 线程
    void capture_thread(cv::VideoCapture& cap)
    {
        while (!stop_flag.load())
        {
            cap >> tmp;
            if (tmp.empty()) continue;
            std::lock_guard<std::mutex> lock(frame_mutex);
            tmp.copyTo(latest_frame);
        }
    }
    ↓
    存储到 latest_frame（线程安全）

T3: cvisual.cpp - inference_thread 线程
    void inference_thread_func(...)
    {
        while (!stop_flag.load())
        {
            // 1. 获取最新帧
            cv::Mat frame;
            latest_frame.copyTo(frame);
            
            // 2. CUDA 预处理
            preprocess_cuda(frame, (float*)buffers[input_idx], stream);
            
            // 3. TensorRT 推理
            context->enqueueV2(buffers, stream, nullptr);
            
            // 4. D2H 拷贝
            cudaMemcpyAsync(result.output_data.data(), buffers[output_idx],
                            output_size, cudaMemcpyDeviceToHost, stream);
            
            // 5. 推入结果队列
            result_queue.push(std::move(result));
        }
    }
    ↓
    YOLOv8 推理输出：8400 个锚点，每个锚点 84 个值（4个坐标 + 80个类别）

T4: cvisual.cpp - 主线程 CPU 后处理
    while (ros::ok())
    {
        // 1. 从队列取出结果
        InferenceResult result = result_queue.try_pop(result);
        
        // 2. 解码 YOLOv8 输出
        auto detections = parse_yolov8(result.output_data.data(),
                                       result.img_width, result.img_height);
        // 输出：多个 Detection 结构体（x1, y1, x2, y2, conf）
        
        // 3. NMS 去重
        apply_nms(raw_dets, IOU_THRESH);
        
        // 4. SORT 跟踪
        auto tracked = tracker.update(detections, result.img_width, result.img_height);
        // 输出：跟踪后的目标列表
        
        // 5. 打包消息
        px4_controller::tbag msg = make_visual_msg(tracked);
        
        // 6. 发布
        pub.publish(msg);
    }
    ↓
    发布到 IR 话题（30Hz）

T5: main.cpp - visual_cb() 回调
    void visual_cb(const px4_controller::tbag::ConstPtr& msg)
    {
        VisualData.Target1_Exist = msg->Target1_Exist;
        VisualData.Target1_PR = msg->Target1_PR;
        VisualData.Target1_LU_x = msg->Target1_LU_x;
        // ... 复制所有目标数据
    }
    ↓
    存储到 VisualData 结构体

T6: aim.cpp - 使用 VisualData
    // 读取 VisualData
    if (VisualData.Target1_Exist)
    {
        // 计算瞄准点
        // PID 控制
        // 发送投放指令
    }

─────────────────────────────────────────────────────────────
```

---

## 八、常见问题解答

### 8.1 main 节点不是 main.cpp 吗，怎么跟 control.cpp 扯上关系了？

**答案**：

- `main.cpp` 是**主函数入口**，创建 ROS 节点
- `control.cpp` 是**工具函数集合**，包含 SetPoint()、TakeOff() 等
- 它们被**编译成同一个可执行文件**
- 就像你的左手和右手，属于同一个身体（Main 可执行文件）

**CMakeLists.txt 第 155 行**：
```cmake
add_executable(Main 
    src/main.cpp 
    src/control.cpp      // ← 一起编译！
    src/Drone.cpp 
    ...)
```

**调用关系**：
```cpp
// main.cpp
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main");  // 创建 ROS 节点
    Drone drone;                     // 创建状态机对象
    
    while (ros::ok())
    {
        drone.HandleState();  // 调用 Drone 的状态机
    }
}

// Drone.cpp
void Drone::ExecuteTakeOff()
{
    TakeOff(5);  // ← 调用 control.cpp 中的 TakeOff() 函数
}

// control.cpp
void TakeOff(double waittime)
{
    SetPoint(0, 0, InitialHeight);  // ← 调用 control.cpp 中的 SetPoint() 函数
}
```

### 8.2 回调函数到底是发布者发布时候使用，还是订阅者订阅之后使用？

**答案**：

**回调函数是订阅者使用的！**

**Service 的回调函数**：
- 在**服务端**（服务器）定义
- 当**客户端发送请求**时，服务端自动调用回调函数
- 就像餐厅的服务员（回调函数）在餐厅里（服务端）等待顾客（客户端）点餐

**Topic 的回调函数**：
- 在**订阅者**定义
- 当**发布者发布消息**时，订阅者自动调用回调函数
- 就像收音机（订阅者）收到电台广播（发布者）时自动播放

### 8.3 为什么回调函数放在了发布者的文件里？

**答案**：

你混淆了**Service 的回调函数**和**Topic 的发布者**。

#### **Service 的回调函数**

在 `SetPoint.cpp` 中的 `SetPosition()` 函数：
```cpp
bool SetPosition(px4_controller::position::Request& req,
                 px4_controller::position::Response& resp)
{
    // 这是服务端的回调函数
    // 当收到客户端的请求时调用
}
```

**为什么在 SetPoint.cpp？**
- 因为 SetPoint.cpp 是**服务端**（服务器）
- 服务端需要**接收和处理请求**
- 就像餐厅的服务员在餐厅里等待顾客

#### **Topic 的发布者**

在 `SetPoint.cpp` 中的 `local_pose_pub.publish(pose)`：
```cpp
while(ros::ok())
{
    if(pub_mode == 0)
    {
        local_pose_pub.publish(pose);  // ← 发布者
    }
    ros::spinOnce();  // ← 处理 Service 请求（调用回调函数）
}
```

**SetPoint 节点同时做两件事**：
1. **服务端**：接收 main 节点的 Service 请求（通过回调函数）
2. **发布者**：持续发布到 MAVROS 的 Topic

### 8.4 发布者不应该是这个服务端吗？应该是 SetPoint 吗？

**答案**：

是的，**SetPoint 节点既是服务端，又是发布者**！

```
SetPoint 节点
├── 服务端角色
│   └── 接收 main 节点的 SendPosition 请求
│       └── SetPosition() 回调函数
│
└── 发布者角色
    └── 持续发布到 MAVROS Topic
        ├── /mavros/setpoint_position/local（位置模式）
        └── /mavros/setpoint_raw/local（速度模式）
```

**为什么这样设计？**

因为需要**坐标系转换**和**数据封装**：

1. **main 节点使用机体坐标系**（BODY Frame）：
   - 前飞是 X+
   - 右飞是 Y+
   - 向上是 Z+

2. **MAVROS 使用 ENU 坐标系**（East-North-Up）：
   - 东是 X+
   - 北是 Y+
   - 上是 Z+

3. **SetPoint 节点的作用**：
   - 接收 main 节点的请求（已经转换好的 ENU 坐标）
   - 自动填充四元数（根据初始偏航角）
   - 持续发布到 MAVROS

**简单说**：SetPoint 节点是一个**"翻译官"**，把 main 节点的指令"翻译"成 MAVROS 能理解的格式。

### 8.5 跟 control 那个文件又有什么关系？为什么又在 control 那里创建 position::request？

**答案**：

**control.cpp 是客户端**，需要**创建请求数据**并发送给服务端。

```cpp
// control.cpp - SetPoint() 函数
bool SetPoint(double x, double y, double z)
{
    // 1. 创建 Request 对象（这就是"创建"）
    px4_controller::position pos;
    
    // 2. 填充数据
    pos.request.mode = 0;  // 位置模式
    pos.request.x = ENU_x; // 转换后的X坐标
    pos.request.y = ENU_y; // 转换后的Y坐标
    pos.request.z = z;     // 高度
    pos.request.qw = q.w(); // 四元数
    pos.request.qx = q.x();
    pos.request.qy = q.y();
    pos.request.qz = q.z();
    pos.request.initial_yaw = initial_yaw;
    
    // 3. 发送请求
    set_position_client.call(pos);  // ← 发送给 SetPoint 节点
}
```

**类比**：
- 顾客（control.cpp）创建订单（Request）
- 把订单交给服务员（SetPoint.cpp）
- 服务员接收订单（SetPosition() 回调）

### 8.6 怎么又发布到 mavros 的 topic 上了？又跟 mavros 话题扯上什么关系了

**答案**：

**SetPoint 节点同时扮演两个角色**：

1. **服务端**：接收 main 节点的 Service 请求
2. **发布者**：持续发布到 MAVROS 的 Topic

**完整流程**：

```
main 节点（客户端）
  ↓
  调用 SetPoint(0, 0, 3.5)
  ↓
  创建 position::Request
  ↓
  发送 Service 请求到 SetPoint 节点
  ↓
SetPoint 节点（服务端 + 发布者）
  ↓
  SetPosition() 回调接收请求
  ↓
  解析数据，更新全局变量 pose
  ↓
  主循环持续发布：
  local_pose_pub.publish(pose)
  ↓
  发布到 /mavros/setpoint_position/local
  ↓
MAVROS（订阅者）
  ↓
  收到 /mavros/setpoint_position/local 消息
  ↓
  转发给 PX4 飞控
```

**为什么这样设计？**

因为：
1. **main 节点不直接与 MAVROS 交互**，而是通过 SetPoint 节点"翻译"和"转发"
2. **SetPoint 节点负责坐标系转换**（机体坐标系 → ENU 坐标系）
3. **SetPoint 节点负责自动填充四元数**（根据初始偏航角）
4. **SetPoint 节点持续发布**，确保飞控持续收到目标位置

### 8.7 main 节点创建请求时的传参的数据又是哪来的？

**答案**：

数据来自**四个地方**：

#### **1. 硬编码的常量（宏定义）**

在 `main.h` 中定义的宏：
```cpp
#define InitialHeight 3.5      // 初始起飞高度
#define DetectHeight 3.5       // 侦察高度
#define TakeofftoThrow 32.5    // 起飞点到投放中心的距离
#define TakeofftoDetect 57.5   // 起飞点到侦察中心的距离
```

**使用示例**：
```cpp
// control.cpp - TakeOff()
SetPoint(0, 0, InitialHeight);  // 使用宏定义的高度

// control.cpp - Detect()
SetPoint(0, TakeofftoThrow, DetectHeight);  // 使用宏定义的距离和高度
```

#### **2. 全局变量（实时更新的传感器数据）**

在 `main.h` 中声明，在 `communication.cpp` 中更新：
```cpp
extern struct Position PX4_Position;  // 当前位置（来自 MAVROS）
extern struct Velocity PX4_Velocity;  // 当前速度（来自 MAVROS）
extern double initial_yaw;            // 初始偏航角（来自 IMU）
```

**使用示例**：
```cpp
// Drone.cpp - OnEnterState()
Goal.px = PX4_Position.x;  // 使用当前位置作为目标点
Goal.py = PX4_Position.y;
Goal.pz = PX4_Position.z;
```

#### **3. Web 服务器（HTTP 远程控制）**

通过 HTTP 请求设置：
```
http://<ip>:8880/setgoal?mode=0&px=10&py=20&pz=3
```

**处理流程**：
```cpp
// WebServer.cpp - 路由处理
↓
// Drone.cpp - UpdateGoal() 函数
↓
// 解析 HTTP 参数
Goal.mode = 0;
Goal.px = 10;
Goal.py = 20;
Goal.pz = 3;
↓
// 存储到 Goal 结构体
```

#### **4. 视觉数据（摄像头识别结果）**

通过 ROS Topic `IR` 接收：
```cpp
// cvisual.cpp 发布
pub.publish(make_visual_msg(tracked));
↓
// main.cpp 订阅
visual_sub = nh.subscribe("IR", 1, visual_cb);
↓
// communication.cpp 回调
void visual_cb(const px4_controller::tbag::ConstPtr& msg)
{
    VisualData.Target1_Exist = msg->Target1_Exist;
    // ...
}
↓
// aim.cpp 使用 VisualData 进行瞄准
```

---

## 九、通信关系总图

### 9.1 整体架构图

```
┌──────────────────────────────────────────────────────────┐
│  main 节点（可执行文件：Main）                             │
│  ┌────────────────────────────────────────────────────┐  │
│  │ main.cpp                                           │  │
│  │ - ROS 节点初始化                                    │  │
│  │ - 创建 Drone 对象                                  │  │
│  │ - 主循环：drone.HandleState()                      │  │
│  └────────────────────────────────────────────────────┘  │
│                            ↓                             │
│  ┌────────────────────────────────────────────────────┐  │
│  │ Drone.cpp（状态机）                                 │  │
│  │ - 管理飞行状态                                      │  │
│  │ - 调用 control.cpp 的函数                          │  │
│  └────────────────────────────────────────────────────┘  │
│                            ↓                             │
│  ┌────────────────────────────────────────────────────┐  │
│  │ control.cpp（控制函数）                             │  │
│  │ - SetPoint()：创建 Request，发送 Service 请求      │  │
│  │ - TakeOff()、Land() 等流程函数                     │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
                          │
                          │ ROS Service（SendPosition）
                          ↓
┌──────────────────────────────────────────────────────────┐
│  SetPoint 节点（可执行文件：SetPoint）                     │
│  ┌────────────────────────────────────────────────────┐  │
│  │ SetPosition() 回调函数                              │  │
│  │ - 接收 Request                                     │  │
│  │ - 解析数据，更新全局变量                            │  │
│  │ - 返回 Response                                    │  │
│  └────────────────────────────────────────────────────┘  │
│                            ↓                             │
│  ┌────────────────────────────────────────────────────┐  │
│  │ main() 主循环                                       │  │
│  │ - 持续发布到 /mavros/setpoint_position/local       │  │
│  │ - 或发布到 /mavros/setpoint_raw/local              │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
                          │
                          │ ROS Topic
                          ↓
┌──────────────────────────────────────────────────────────┐
│  MAVROS（订阅者）                                         │
│  - 订阅 /mavros/setpoint_position/local                  │
│  - 转发给 PX4 飞控                                       │
└──────────────────────────────────────────────────────────┘
                          │
                          │ MAVLink 协议
                          ↓
┌──────────────────────────────────────────────────────────┐
│  PX4 飞控                                                │
│  - 接收目标位置                                          │
│  - 控制电机                                              │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│  cvisual 节点（可执行文件：cvisual）                       │
│  - 摄像头采集图像                                        │
│  - YOLOv8 推理检测                                      │
│  - SORT 跟踪                                            │
│  - 发布到 IR 话题（30Hz）                               │
└──────────────────────────────────────────────────────────┘
                          │
                          │ ROS Topic（IR）
                          ↓
┌──────────────────────────────────────────────────────────┐
│  main 节点（订阅者）                                      │
│  - visual_cb() 回调接收                                 │
│  - 存储到 VisualData 结构体                              │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│  throw 节点（可执行文件：throw）                          │
│  - ThrowBottle() 回调函数                               │
│  - 控制舵机                                              │
└──────────────────────────────────────────────────────────┘
                          │
                          │ ROS Service（ThrowCmd）
                          ↓
┌──────────────────────────────────────────────────────────┐
│  main 节点（客户端）                                     │
│  - ThrowBottle() 函数创建 Request                       │
│  - 发送 Service 请求                                    │
└──────────────────────────────────────────────────────────┘
```

### 9.2 数据流向总结

#### **控制指令（main → 飞控）**
```
SetPoint() 
  → set_position_client.call() 
  → SendPosition 服务（自定义）
  → SetPoint 节点
  → /mavros/setpoint_position/local 话题
  → MAVROS 
  → PX4 飞控
```

#### **状态数据（飞控 → main）**
```
PX4 飞控 
  → MAVROS 
  → /mavros/local_position/pose 话题
  → pos_cb() 回调函数
  → PX4_Position 全局变量
```

#### **视觉数据（cvisual → main）**
```
cvisual 节点 
  → IR 话题（自定义消息 tbag）
  → visual_cb() 回调函数
  → VisualData 结构体
```

#### **投放指令（main → throw）**
```
ThrowBottle() 
  → throw_client.call() 
  → ThrowCmd 服务（自定义）
  → throw 节点
  → 舵机控制
```

### 9.3 通信分类总结

| 通信类型 | 名称 | 发布者/服务端 | 订阅者/客户端 | 数据来源 | 数据内容 |
|---------|------|-------------|-------------|---------|---------|
| **Service** | SendPosition | SetPoint.cpp (SetPoint节点) | main.cpp (main节点) | 函数调用 SetPoint() | 目标位置/速度、四元数 |
| **Service** | ThrowCmd | throw.cpp (throw节点) | main.cpp (main节点) | 函数调用 ThrowBottle() | 投放命令 cmd |
| **Topic** | IR | cvisual.cpp (visual_node) | main.cpp (main节点) | 摄像头 + YOLOv8推理 | 目标像素坐标、置信度 |
| **Service** | /mavros/cmd/arming | MAVROS | main.cpp | 函数调用 Arm() | 解锁命令 |
| **Service** | /mavros/set_mode | MAVROS | main.cpp | 函数调用 SetMode() | 模式设置命令 |
| **Topic** | mavros/state | MAVROS | main.cpp | PX4 飞控 | 连接状态、 armed、 mode |
| **Topic** | /mavros/local_position/pose | MAVROS | main.cpp | PX4 飞控 | 位置 x,y,z |
| **Topic** | /mavros/local_position/velocity_local | MAVROS | main.cpp | PX4 飞控 | 速度 vx,vy,vz |
| **Topic** | /mavros/imu/data | MAVROS | main.cpp | PX4 飞控 | IMU 数据 |
| **Topic** | /mavros/distance_sensor/hrlv_ez4_pub | MAVROS | main.cpp | 激光雷达 | 测距数据 |

---

## 十、核心概念总结

### 10.1 ROS 节点组织

- **一个可执行文件 = 一个 ROS 节点**
- `main.cpp`、`control.cpp`、`Drone.cpp` 等被编译成同一个可执行文件 `Main`
- 运行时只有一个 ROS 节点，节点名是 `main`
- 不同文件中的函数可以互相调用，因为它们在同一个程序中

### 10.2 Service 通信模式

- **客户端**：创建 Request，调用服务
- **服务端**：接收 Request，处理并返回 Response
- **回调函数**：在服务端定义，用于接收和处理请求
- **你的代码**：
  - 客户端：`control.cpp` 中的 `SetPoint()`、`ThrowBottle()`
  - 服务端：`SetPoint.cpp` 中的 `SetPosition()`、`throw.cpp` 中的 `ThrowBottle()`

### 10.3 Topic 通信模式

- **发布者**：持续发布消息
- **订阅者**：接收消息，调用回调函数
- **你的代码**：
  - 发布者：`cvisual.cpp` 发布 IR 话题
  - 订阅者：`main.cpp` 订阅 IR 话题

### 10.4 数据来源

1. **硬编码常量**：宏定义（如 `InitialHeight = 3.5`）
2. **全局变量**：传感器数据（如 `PX4_Position`、`VisualData`）
3. **Web 服务器**：HTTP 请求参数
4. **视觉数据**：摄像头识别结果

### 10.5 架构设计原因

**为什么需要 SetPoint 节点？**

1. **坐标系转换**：机体坐标系 ↔ ENU 坐标系
2. **数据封装**：自动填充四元数
3. **持续发布**：确保飞控持续收到目标位置
4. **解耦设计**：main 节点不直接与 MAVROS 交互

---

## 十一、附录

### 11.1 文件清单

| 文件 | 作用 | 所属可执行文件 |
|------|------|--------------|
| `main.cpp` | 主函数入口，ROS 节点初始化 | Main |
| `control.cpp` | 控制函数（SetPoint, TakeOff, Land...） | Main |
| `Drone.cpp` | 状态机管理 | Main |
| `communication.cpp` | ROS 通信（订阅者、回调函数） | Main |
| `aim.cpp` | 瞄准逻辑 | Main |
| `WebServer.cpp` | HTTP 服务器 | Main |
| `SetPoint.cpp` | SendPosition 服务端 | SetPoint |
| `throw.cpp` | ThrowCmd 服务端 | throw |
| `cvisual.cpp` | 视觉识别节点 | cvisual |

### 11.2 可执行文件清单

| 可执行文件 | 源文件 | 节点名 | 作用 |
|-----------|--------|--------|------|
| `Main` | main.cpp, control.cpp, Drone.cpp, ... | `main` | 主节点，状态机，控制逻辑 |
| `SetPoint` | SetPoint.cpp | `SetPoint` | SendPosition 服务端，发布到 MAVROS |
| `throw` | throw.cpp | `throw` | ThrowCmd 服务端，控制舵机 |
| `cvisual` | cvisual.cpp | `visual_node` | 视觉识别，发布 IR 话题 |

### 11.3 自定义消息/服务清单

| 名称 | 类型 | 文件 | 用途 |
|------|------|------|------|
| `position` | Service | `srv/position.srv` | 位置/速度设置 |
| `throwcmd` | Service | `srv/throwcmd.srv` | 投放控制 |
| `tbag` | Message | `msg/tbag.msg` | 视觉识别结果 |

---

**文档生成时间**：2026-07-07  
**版本**：v1.0  
**作者**：AI Assistant  
**说明**：本文档基于项目代码分析生成，详细解释了项目中的 ROS 通信机制、代码架构和数据流。