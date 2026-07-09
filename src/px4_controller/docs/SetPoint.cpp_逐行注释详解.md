# SetPoint.cpp 逐行注释详解

---

## 文件概述

`SetPoint.cpp` 是一个**独立的 ROS 节点**，作为主节点（main）和 PX4 飞控之间的"桥梁"。它负责：

1. **接收服务请求**：通过 `SendPosition` 服务接收主节点发来的位置/速度指令
2. **发布到 MAVROS**：将指令转换为 ROS 话题发布给 MAVROS，最终发给 PX4 飞控
3. **双模式支持**：位置控制模式（发布到 `/mavros/setpoint_position/local`）和速度控制模式（发布到 `/mavros/setpoint_raw/local`）

**为什么需要单独的节点**：
- PX4 的 OFFBOARD 模式要求以 >2Hz 的频率持续接收指令
- 如果主节点在阻塞操作（如 `TakeOff(5)`）时停止发送指令，飞控会退出 OFFBOARD 模式
- 独立的 `SetPoint` 节点可以持续以 30Hz 发布最新指令，不受主节点阻塞影响

---

## 第1-5行：包含头文件

```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_controller/position.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>
```

| 头文件 | 用途 |
|--------|------|
| `<ros/ros.h>` | ROS 核心库 |
| `<geometry_msgs/PoseStamped.h>` | 位置消息类型（用于位置控制模式） |
| `<px4_controller/position.h>` | 自定义服务类型（接收主节点指令） |
| `<geometry_msgs/Twist.h>` | 速度消息类型（未使用，注释掉了） |
| `<mavros_msgs/PositionTarget.h>` | MAVROS 速度+位置消息类型（用于速度控制模式） |

---

## 第7-11行：全局变量

```cpp
ros::Publisher local_pose_pub;
ros::Publisher vel_pub;
geometry_msgs::PoseStamped pose; //ENU
//geometry_msgs::Twist vel_msg;//全局速度
mavros_msgs::PositionTarget vel_msg;
```

- `local_pose_pub`：发布位置指令到 `/mavros/setpoint_position/local`
- `vel_pub`：发布速度指令到 `/mavros/setpoint_raw/local`
- `pose`：存储最新的位置指令（ENU 坐标系）
- `vel_msg`：存储最新的速度指令（使用 `mavros_msgs::PositionTarget` 类型）

**为什么速度用 `PositionTarget` 而不是 `Twist`**：
- `PositionTarget` 可以同时指定位置和速度
- 速度模式下只使用速度分量，位置分量被 type_mask 忽略
- 但保留位置 `z` 用于控制高度

---

## 第13-14行：模式标志和偏航角

```cpp
int pub_mode=0;//0为发布坐标点，1为发布速度
double initial_yaw;
```

- `pub_mode`：0=位置模式，1=速度模式。由服务请求中的 `mode` 字段设置
- `initial_yaw`：初始偏航角，由主节点传入

---

## 第15-34行：服务回调函数

```cpp
bool SetPosition(px4_controller::position::Request& req,
                 px4_controller::position::Response& resp)
{
    pub_mode=req.mode;
    pose.pose.position.x = req.x;
    pose.pose.position.y = req.y;
    pose.pose.position.z = req.z;
    pose.pose.orientation.w=req.qw;
    pose.pose.orientation.x=req.qx;
    pose.pose.orientation.y=req.qy;
    pose.pose.orientation.z=req.qz;
    vel_msg.velocity.x = req.vx;
    vel_msg.velocity.y = req.vy;
    vel_msg.velocity.z = 0;
    vel_msg.position.z = req.z;
    initial_yaw=req.initial_yaw;
    ROS_INFO("发布ENU-Position: %f %f %f,Vel %f %f ",req.x,req.y,req.z,req.vx,req.vy);
    resp.success = true;
    return true;
}
```

### 工作流程

```
主节点调用 set_position_client.call(req)
  → SetPoint 节点收到服务请求
    → SetPosition() 被调用
      → 更新全局变量 pose 和 vel_msg
      → 设置 resp.success = true
      → 返回 true
```

### 位置模式数据处理

```cpp
pose.pose.position.x = req.x;   // ENU 坐标 X
pose.pose.position.y = req.y;   // ENU 坐标 Y
pose.pose.position.z = req.z;   // 高度 Z
```

- 注意：这些坐标已经是 ENU 坐标系了（`control.cpp` 的 `SetPoint()`/`SetVel()` 中已经调用了 `bodyToENU` 转换）
- `pose` 是 `geometry_msgs::PoseStamped` 类型

### 速度模式数据处理

```cpp
vel_msg.velocity.x = req.vx;   // ENU 速度 X
vel_msg.velocity.y = req.vy;   // ENU 速度 Y
vel_msg.velocity.z = 0;        // Z 方向速度为 0
vel_msg.position.z = req.z;    // 但位置 Z（高度）用于控制高度
```

- 速度模式下，`vel_msg.velocity.z = 0`：垂直速度由位置 Z 控制（通过 `position.z` 字段）
- `vel_msg.position.z = req.z`：设置目标高度

---

## 第37-80行：主函数

```cpp
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"SetPoint");
    setlocale(LC_ALL,"");

    ros::Time::init();
    ros::NodeHandle nh;
    local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local",10);
    vel_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local",10);
    ros::ServiceServer server = nh.advertiseService("SendPosition",SetPosition);
    ROS_INFO_STREAM("\033[32m" << "[SetPoint] 位置速度发布服务成功启动!" << "\033[0m");

    ros::Rate loop_rate(30);
    // ...
    while(ros::ok())
    {
        if(pub_mode==0)//pos
        {
            local_pose_pub.publish(pose);
        }
        else//mode==1 vel
        {
            vel_msg.header.stamp = ros::Time::now();
            vel_pub.publish(vel_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
```

### 第44-47行：创建发布者

```cpp
local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local",10);
vel_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local",10);
```

- 位置模式发布到 `/mavros/setpoint_position/local`（MAVROS 的位置控制话题）
- 速度模式发布到 `/mavros/setpoint_raw/local`（MAVROS 的原始控制话题）

### 第48行：创建服务端

```cpp
ros::ServiceServer server = nh.advertiseService("SendPosition",SetPosition);
```

- 服务名 `SendPosition`
- 回调函数 `SetPosition`
- 主节点通过 `set_position_client` 调用这个服务

### 第52-62行：速度消息配置

```cpp
vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;  //Attention!!! NED  not ENU
vel_msg.type_mask = 
    mavros_msgs::PositionTarget::IGNORE_PX | 
    mavros_msgs::PositionTarget::IGNORE_PY | //Attention!!! Do not IGNORE VZ
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
```

**`FRAME_LOCAL_NED` 说明**：
- 注释强调 **NED not ENU**
- NED = North-East-Down（北-东-地）
- 这是 PX4 内部使用的坐标系
- 虽然代码中其他地方使用 ENU，但这里必须用 NED

**`type_mask` 位掩码的作用**：
- `IGNORE_PX`、`IGNORE_PY`：忽略位置 X 和 Y（使用速度控制）
- **不忽略** `IGNORE_PZ`：所以 `position.z` 用于控制高度
- `IGNORE_AFX/AFY/AFZ`：忽略加速度
- `IGNORE_YAW/YAW_RATE`：忽略偏航角和偏航角速度

### 第63-77行：主循环（30Hz）

```cpp
while(ros::ok())
{
    if(pub_mode==0)//pos
    {
        local_pose_pub.publish(pose);
    }
    else//mode==1 vel
    {
        vel_msg.header.stamp = ros::Time::now();
        vel_pub.publish(vel_msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
}
```

**30Hz 持续发布**：
- 无论主节点是否发送新的服务请求，这个循环都以 30Hz 持续发布
- 位置模式：持续发布 `pose`（除非主节点更新，否则保持上次的值）
- 速度模式：持续发布 `vel_msg`（需要更新 `header.stamp` 时间戳）

**为什么需要 `header.stamp`**：
- 速度模式下，每次发布前更新 `vel_msg.header.stamp = ros::Time::now()`
- MAVROS 根据时间戳判断指令是否过时
- 如果没有更新时间戳，飞控可能认为指令已过期

---

## 总结：SetPoint 节点的工作模式

```
主节点 main.cpp                          SetPoint 节点
     │                                       │
     │  set_position_client.call(pos)         │
     │ ──────────────────────────────────→    │
     │                                       ├─ SetPosition() 更新 pose/vel_msg
     │  ←────────────────────────────────── │
     │                                       │
     │                                       30Hz 循环
     │                                       │
     │  位置模式:                             │
     │  local_pose_pub.publish(pose)          │
     │  ─────────────────────────────────→   MAVROS → PX4
     │                                       │
     │  速度模式:                             │
     │  vel_pub.publish(vel_msg)              │
     │  ─────────────────────────────────→   MAVROS → PX4
```

**为什么这个设计很重要**：
- 如果主节点阻塞在某个操作中（如 `TakeOff(5)` 的 5 秒休眠），SetPoint 节点仍然在 30Hz 发布指令
- 这确保了 PX4 的 OFFBOARD 模式不会因为指令中断而退出