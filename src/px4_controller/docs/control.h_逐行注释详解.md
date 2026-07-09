# control.h 逐行注释详解

---

## 文件概述

`control.h` 声明了**飞控控制指令**的所有函数接口。它是整个系统中与 PX4 飞控交互的"驱动器"层，负责：

1. **飞控连接与管理**：连接 PX4、切换模式、解锁
2. **运动控制**：位置控制、速度控制、位置检查
3. **任务流程**：起飞、侦察、返航、降落、缓慢移动
4. **投放控制**：发送投放指令

---

## 第1-2行：头文件保护

```cpp
#ifndef __CONTROL_H
#define __CONTROL_H
```

---

## 第4-5行：包含头文件

```cpp
#include "main.h"
#include "communication.h"
```

**为什么包含这两个头文件**：
- `main.h`：提供 `Position`、`Velocity` 等结构体，`#define` 常量（`InitialHeight`、`DetectHeight` 等），`bodyToENU()`/`ENUToBody()` 工具函数
- `communication.h`：提供 `set_position_client`、`throw_client`、`set_mode_client`、`arming_client` 等 ROS 服务客户端，以及 `current_state` 等全局变量

---

## 第7行：时间戳全局变量

```cpp
extern ros::Time last_request;
```

**作用**：记录上次发送请求的时间戳，用于控制请求频率。

**为什么需要它**：
- 在 `SetMode()` 和 `Arm()` 中，防止以过高频率重复发送请求
- 每次发送请求前检查 `now - last_request > 1.0s`
- 确保每个请求间隔至少 1 秒

---

## 第10行：连接飞控

```cpp
void ConnectPX4();
```

**作用**：阻塞等待飞控连接，然后切换到 OFFBOARD 模式。

**流程**：等待连接 → 设置初始位置 → 切 POSCTL → 切 OFFBOARD

---

## 第11行：模式设置（带重试）

```cpp
bool SetMode(std::string md);
```

**作用**：循环发送模式切换请求，直到飞控成功切换到目标模式。

**与 `PX4_SetMode` 的区别**：`SetMode` 是高层封装，会不断重试直到成功；`PX4_SetMode` 是低层一次性调用。

---

## 第12行：模式设置（一次性）

```cpp
bool PX4_SetMode(std::string md);
```

**作用**：直接调用 MAVROS 服务一次，不重试。

---

## 第13行：解锁

```cpp
bool Arm();
```

**作用**：解锁无人机电机。

**流程**：循环发送解锁指令直到 `current_state.armed == true`。

---

## 第14行：发送位置指令

```cpp
bool SetPoint(double x, double y, double z);
```

**作用**：通过 ROS 服务向 `SetPoint` 节点发送位置控制指令。

**参数**：机体坐标系下的 `(x, y, z)` 坐标，内部转换为 ENU 坐标系。

---

## 第15行：发送速度指令

```cpp
bool SetVel(double vx, double vy, double pz);
```

**作用**：通过 ROS 服务向 `SetPoint` 节点发送速度控制指令。

**参数**：机体坐标系下的 `(vx, vy)` 速度和目标高度 `pz`。

---

## 第16行：检查位置

```cpp
bool CheckPosition(float x, float y, float z);
```

**作用**：阻塞等待，直到飞机到达指定位置 ±0.1m 的范围内并稳定至少 100 帧。

---

## 第17行：投放

```cpp
bool ThrowBottle(int cmd);
```

**作用**：通过 ROS 服务向 `throw` 节点发送投放指令。

---

## 第19行：显示位置

```cpp
void ShowPosition(int delay);
```

**作用**：每隔 1 秒打印一次当前位置，持续 `delay` 秒。

---

## 第20行：起飞

```cpp
void TakeOff(double waittime);
```

**作用**：执行完整起飞流程：检查偏差 → 解锁 → 飞到初始高度 → 等待。

---

## 第21行：侦察

```cpp
void Detect();
```

**作用**：执行侦察航线飞行，S 形扫描目标区域。

---

## 第22行：返航

```cpp
void GoHome();
```

**作用**：返回到起飞点。

---

## 第23行：降落

```cpp
void Land();
```

**作用**：切换到 AUTO.LAND 模式，自动降落。

---

## 第24行：缓慢下降

```cpp
void SlowDescend(double px, double py, double Lh, double Nh);
```

**作用**：从高度 `Lh` 逐级下降到 `Nh`，每次降 0.1m 间隔 0.5 秒，保持位置 `(px, py)`。

---

## 第25行：缓慢前移

```cpp
void SlowMoveForward(double px, double pz, double Ly, double Ny);
```

**作用**：从 Y 坐标 `Ly` 逐级前移到 `Ny`，每次移动 3m 间隔 0.3 秒，保持 X 位置 `px` 和高度 `pz`。

> **注意**：当前代码中 `SlowMoveForward` 没有被使用（在 `Locating()` 中被注释掉了）。