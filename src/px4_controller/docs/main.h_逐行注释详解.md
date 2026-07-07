# main.h 逐行注释详解

---

## 文件头：头文件保护宏

```cpp
#ifndef __main_H
#define __main_H
```

**作用**：头文件保护（Include Guard），防止同一个头文件被多次 `#include` 导致重复定义错误。

**语法**：
- `#ifndef` = "if not defined"——如果 `__main_H` 这个宏**没有**被定义过
- `#define __main_H`——立即定义它，这样下次再 `#include` 时，`#ifndef` 检查会失败，整个文件内容被跳过

**为什么这样写**：C/C++ 编译单元（.cpp 文件）在预处理阶段会逐字展开所有 `#include`。如果没有保护，`main.h` 被 A.h 和 B.h 同时包含，而 main.cpp 又同时包含 A.h 和 B.h，就会导致 `struct Position` 等被定义两次，编译报错"redefinition"。

---

## 第4行：ROS 核心头文件

```cpp
#include <ros/ros.h>
```

**作用**：包含 ROS (Robot Operating System) 的核心库。这是**所有 ROS 节点都必须包含**的头文件。

**提供了什么**：
- `ros::init()` —— 初始化 ROS 节点
- `ros::NodeHandle` —— 与 ROS 系统交互的句柄
- `ros::Publisher` / `ros::Subscriber` —— 发布/订阅机制
- `ros::ServiceServer` / `ros::ServiceClient` —— 服务通信机制
- `ros::Rate` —— 循环频率控制
- `ros::Time` / `ros::Duration` —— 时间相关类
- `ROS_INFO()` / `ROS_ERROR()` / `ROS_WARN()` 等日志宏

**为什么用尖括号**：`<ros/ros.h>` 是系统/环境路径下的头文件，由 ROS 的 catkin 构建系统提供。如果是项目内部的头文件（如 `"main.h"`），则用双引号。

---

## 第5行：ROS 包路径工具

```cpp
#include <ros/package.h>
```

**作用**：提供 `ros::package::getPath("包名")` 函数，用于获取某个 ROS 功能包在文件系统中的绝对路径。

**为什么需要它**：在 `main.cpp` 中，我们需要读取 `config/calibration_data.txt` 标定文件。由于 ROS 的工作空间路径不固定（不同电脑可能在不同位置），不能写死路径。通过 `ros::package::getPath("px4_controller")` 可以动态获取当前功能包的安装/编译路径，再拼接上 `/config/calibration_data.txt` 即可。

---

## 第6行：MAVROS 模式设置服务

```cpp
#include <mavros_msgs/SetMode.h>
```

**作用**：包含 MAVROS 的 `SetMode` 服务类型定义。MAVROS 是 PX4 飞控与 ROS 之间的桥梁。

**提供了什么**：`mavros_msgs::SetMode` 是一个 ROS 服务（Service），包含：
- `request.custom_mode`（string 类型）—— 要切换到的飞行模式，如 `"OFFBOARD"`、`"AUTO.LAND"`、`"POSCTL"` 等
- `response.mode_sent`（bool 类型）—— 飞控是否成功接受了模式切换指令

**为什么需要它**：无人机飞行需要先切换到 OFFBOARD 模式才能接受外部位置/速度指令，降落时需要切换到 AUTO.LAND 模式。

---

## 第7行：MAVROS 解锁服务

```cpp
#include <mavros_msgs/CommandBool.h>
```

**作用**：包含 MAVROS 的 `CommandBool` 服务类型定义，用于**解锁（Arm）** 无人机。

**提供了什么**：`mavros_msgs::CommandBool` 服务：
- `request.value`（bool）—— `true` 表示解锁，`false` 表示上锁
- `response.success`（bool）—— 飞控是否成功执行

**为什么需要它**：PX4 飞控出于安全考虑，默认是上锁（Disarmed）状态，电机不会转动。必须发送解锁指令后电机才能启动。

---

## 第8行：C++ 标准库 string

```cpp
#include <string>
```

**作用**：C++ 标准库中的 `std::string` 类，用于处理文本字符串。

**为什么需要它**：代码中大量使用字符串，例如：
- `SetMode(std::string md)` 函数参数传递模式名称
- `GetStateName()` 返回状态名称字符串
- ROS 日志输出等

---

## 第9行：C 标准库

```cpp
#include <cstdlib>
```

**作用**：C 标准库的 C++ 版本，提供 `std::abs()`（绝对值）、`std::atoi()`/`std::atof()`（字符串转数字）等函数。

**为什么需要它**：在 `control.cpp` 的 `TakeOff()` 中使用了 `abs(PX4_Position.x)` 来判断位置偏差。

---

## 第10行：自定义服务——位置设定

```cpp
#include <px4_controller/position.h>
```

**作用**：包含本功能包自定义的 ROS 服务类型 `position`。这个服务定义在 `srv/position.srv` 文件中。

**提供了什么**：`px4_controller::position` 服务：
- `request.mode` —— 0=位置控制，1=速度控制
- `request.x/y/z` —— 目标位置（ENU 坐标系）
- `request.vx/vy` —— 目标速度
- `request.qw/qx/qy/qz` —— 姿态四元数
- `request.initial_yaw` —— 初始偏航角

**为什么需要它**：主节点（main）通过这个服务将位置/速度指令发送给 `SetPoint.cpp` 节点，后者再通过 MAVROS 发布给飞控。这是一种**跨节点的进程间通信**方式。

---

## 第11行：自定义服务——投放指令

```cpp
#include <px4_controller/throwcmd.h>
```

**作用**：包含自定义的 `throwcmd` 服务类型，定义在 `srv/throwcmd.srv` 中。

**提供了什么**：`px4_controller::throwcmd` 服务：
- `request.cmd`（int）—— 投放指令编号（1-6）
- `response.success`（bool）—— 执行结果

**为什么需要它**：主节点通过这个服务将投放指令发送给 `throw.cpp` 节点，后者控制 Jetson Nano 的 GPIO 引脚驱动舵机。

---

## 第12行：MAVROS 状态消息

```cpp
#include <mavros_msgs/State.h>
```

**作用**：包含 MAVROS 的 `State` 消息类型，用于订阅飞控的实时状态。

**提供了什么**：`mavros_msgs::State` 包含：
- `connected`（bool）—— 是否连接到飞控硬件
- `armed`（bool）—— 是否已解锁
- `mode`（string）—— 当前飞行模式（如 "OFFBOARD"、"STABILIZED" 等）

**为什么需要它**：主循环需要知道飞控的连接状态、解锁状态和当前模式，才能决定下一步操作。例如 `ConnectPX4()` 中要等待 `connected == true`，`Arm()` 中要等待 `armed == true`。

---

## 第13行：标准消息类型

```cpp
#include <std_msgs/String.h>
```

**作用**：ROS 标准消息中的字符串类型。

**为什么需要它**：虽然本文件中没有直接使用，但其他文件（如 `communication.h`）可能需要。放在这里是为了让所有包含 `main.h` 的文件都能访问到这个类型。

---

## 第14行：自定义话题消息——视觉数据

```cpp
#include <px4_controller/tbag.h>
```

**作用**：包含自定义的 ROS 消息类型 `tbag`，定义在 `msg/tbag.msg` 中。

**提供了什么**：`px4_controller::tbag` 消息包含三个目标的检测结果：
- `Target1/2/3_Exist` —— 目标是否存在
- `Target1/2/3_PR` —— 置信度（Probability）
- `Target1/2/3_LU_x/y`、`RU_x/y`、`RD_x/y`、`LD_x/y` —— 目标框的四个角像素坐标

**为什么需要它**：视觉检测节点（`cvisual.cpp`）通过话题 `"IR"` 发布检测结果，主节点订阅后更新 `VisualData` 结构体，供瞄准模块使用。

---

## 第15行：字符串流

```cpp
#include <sstream>
```

**作用**：C++ 标准库中的 `std::stringstream` 类，用于字符串的格式化输入输出。

**为什么需要它**：在 `calibration.cpp` 中用于解析标定文件的每一行数据，类似于 `sscanf` 但更安全。

---

## 第16行：位姿消息

```cpp
#include <geometry_msgs/PoseStamped.h>
```

**作用**：ROS 标准几何消息类型，包含带时间戳的三维位姿（位置 + 姿态四元数）。

**提供了什么**：`geometry_msgs::PoseStamped`：
- `header.stamp` —— 时间戳
- `pose.position.x/y/z` —— 三维位置
- `pose.orientation.x/y/z/w` —— 姿态四元数

**为什么需要它**：MAVROS 通过 `/mavros/local_position/pose` 话题发布飞控的当前位置信息，回调函数 `pos_cb` 接收的就是这个消息类型。

---

## 第17行：IMU 消息

```cpp
#include <sensor_msgs/Imu.h>
```

**作用**：ROS 标准传感器消息类型，包含 IMU（惯性测量单元）数据。

**提供了什么**：`sensor_msgs::Imu`：
- `orientation.x/y/z/w` —— 姿态四元数
- `angular_velocity.x/y/z` —— 角速度
- `linear_acceleration.x/y/z` —— 线加速度

**为什么需要它**：在 `imu_cb` 回调中，我们从 IMU 的四元数中提取初始偏航角 `initial_yaw`，这个角度用于坐标系转换（机体坐标系 ↔ ENU 坐标系）。

---

## 第18行：测距传感器消息

```cpp
#include <sensor_msgs/Range.h>
```

**作用**：ROS 标准传感器消息类型，表示距离传感器的测量值。

**提供了什么**：`sensor_msgs::Range`：
- `range`（float）—— 测量的距离值（米）
- `min_range` / `max_range` —— 传感器量程

**为什么需要它**：激光雷达（LiDAR）通过 `/mavros/distance_sensor/hrlv_ez4_pub` 话题发布高度数据，`lidar_cb` 回调接收后用于标定查询。

---

## 第19行：速度消息

```cpp
#include <geometry_msgs/TwistStamped.h>
```

**作用**：ROS 标准几何消息类型，包含带时间戳的三维线速度和角速度。

**提供了什么**：`geometry_msgs::TwistStamped`：
- `twist.linear.x/y/z` —— 线速度
- `twist.angular.x/y/z` —— 角速度

**为什么需要它**：MAVROS 通过 `/mavros/local_position/velocity_local` 话题发布飞控的当前速度，`vel_cb` 回调接收后更新 `PX4_Velocity` 全局变量。

---

## 第20-21行：TF2 四元数/矩阵

```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
```

**作用**：ROS 的 TF2 库中的四元数和 3x3 矩阵类，用于姿态表示和转换。

**提供了什么**：
- `tf2::Quaternion` —— 四元数类，支持 `setRPY(roll, pitch, yaw)` 从欧拉角创建
- `tf2::Matrix3x3` —— 3x3 旋转矩阵，支持 `getRPY(roll, pitch, yaw)` 从四元数提取欧拉角

**为什么需要它**：
1. 在 `imu_cb` 中：从 IMU 的四元数提取偏航角 `initial_yaw`
2. 在 `SetPoint()` 和 `SetVel()` 中：创建姿态四元数用于位置/速度指令

---

## 第22行：随机数

```cpp
#include <random>
```

**作用**：C++11 标准库中的随机数生成工具。

**提供了什么**：
- `std::random_device` —— 真随机数种子
- `std::mt19937` —— 梅森旋转算法（Mersenne Twister）伪随机数生成器
- `std::uniform_int_distribution` —— 均匀整数分布

**为什么需要它**：在 `aim.cpp` 的 `robustKMeans()` 函数中，需要随机选择初始聚类中心点。

---

## 第23行：线程

```cpp
#include <thread>
```

**作用**：C++11 标准库中的多线程支持。

**提供了什么**：`std::thread` 类，用于创建和管理线程。

**为什么需要它**：
- `WebServer` 在独立线程中运行 HTTP 服务器
- `TelemetryBroadcaster` 在独立线程中广播 UDP 遥测数据
- `cvisual.cpp` 中创建取帧线程和推理线程

---

## 第24行：HTTP 库

```cpp
#include <httplib.h>
```

**作用**：一个轻量级的 C++ HTTP 库（单头文件），用于创建 HTTP 服务器和客户端。

**提供了什么**：
- `httplib::Server` —— HTTP 服务器，支持 GET/POST 路由
- `httplib::Client` —— HTTP 客户端
- `httplib::Request` / `httplib::Response` —— 请求/响应对象

**为什么需要它**：`WebServer` 使用这个库在端口 8880 上启动 HTTP 服务器，提供 `/setpid`、`/setstat`、`/setgoal`、`/throw` 等 Web API，供上位机远程控制。

---

## 第27-31行：电子围栏宏

```cpp
#define Electronic_Fence_ENABLE false
#define Electronic_Fence_X_MAX 5.0
#define Electronic_Fence_X_Min -5.0
#define Electronic_Fence_Y_MAX 70.0
#define Electronic_Fence_Y_Min -1.0
```

**作用**：定义电子围栏（Geofence）的开关和边界参数。

**语法**：`#define` 是预处理指令，在编译前进行文本替换。`Electronic_Fence_ENABLE` 会被替换为 `false`。

**为什么这样写**：
- 使用宏而不是变量：宏在编译前就替换好了，没有运行时开销，适合常量定义
- `Electronic_Fence_ENABLE = false`：默认关闭电子围栏。如果开启，在 `pos_cb` 回调中会检查飞机位置是否超出边界，超出则强制悬停并切换到 WAITING 状态
- X 方向范围 -5.0 ~ 5.0 米（左右），Y 方向范围 -1.0 ~ 70.0 米（前后）

**为什么用 `#define` 而不是 `const`**：宏可以用于条件编译（`#ifdef`），但这里只是作为常量使用。用 `constexpr` 或 `const` 也可以，但 ROS 项目中传统上习惯用 `#define` 定义这类参数。

---

## 第34-43行：飞行路径参数宏

```cpp
#define TakeofftoThrow 32.5   // 起飞点到投放中心的距离 (32.5m)
#define TakeofftoDetect 57.5  // 起飞点到侦察中心的距离 (57.5m)
#define Throw1_X 660          // 1号舵机能投放进的像素x
#define Throw1_Y 288
#define Throw2_X 660
#define Throw2_Y 515
#define DetectHeight 3.5      // 侦察高度
#define InitialHeight 3.5     // 初始起飞高度
#define ThrowHeight 0.8       // 投放时高度
#define ThrowMidHeight 2.1    // 投放二级瞄准高度
```

**逐行解释**：

| 宏名 | 值 | 单位 | 用途 |
|------|-----|------|------|
| `TakeofftoThrow` | 32.5 | 米 | 起飞点沿 Y 轴（前方）到投放区域中心的距离 |
| `TakeofftoDetect` | 57.5 | 米 | 起飞点沿 Y 轴到侦察区域中心的距离 |
| `Throw1_X` | 660 | 像素 | 桶1在图像中的瞄准中心 X 坐标 |
| `Throw1_Y` | 288 | 像素 | 桶1在图像中的瞄准中心 Y 坐标 |
| `Throw2_X` | 660 | 像素 | 桶2在图像中的瞄准中心 X 坐标 |
| `Throw2_Y` | 515 | 像素 | 桶2在图像中的瞄准中心 Y 坐标 |
| `DetectHeight` | 3.5 | 米 | 侦察飞行时的高度 |
| `InitialHeight` | 3.5 | 米 | 起飞后的初始悬停高度 |
| `ThrowHeight` | 0.8 | 米 | 最终投放时的高度（低空） |
| `ThrowMidHeight` | 2.1 | 米 | 投放前的中间过渡高度（中空） |

**为什么这样设计**：
- 32.5m 和 57.5m：根据实际场地测量得到，投放区在 32.5m 处，侦察区在 57.5m 处
- 660 像素：图像宽度 1280 的一半是 640，660 略偏右，说明桶的投放口在图像中心偏右位置
- 3.5m → 2.1m → 0.8m：分级下降策略，每级都做 PID 对准，避免一次性下降导致偏离目标

---

## 第46-54行：PID 参数宏

```cpp
#define Kp_H 0.01
#define Ki_H 0
#define Kd_H 0
#define Kp_L 0.001
#define Ki_L 0
#define Kd_L 0
#define MaxVel_H 0.15
#define MaxVel_L 0.05
#define DeadZone 0.01
```

**逐行解释**：

| 宏名 | 值 | 含义 |
|------|-----|------|
| `Kp_H` | 0.01 | 高空（High）比例增益 |
| `Ki_H` | 0 | 高空积分增益（未使用） |
| `Kd_H` | 0 | 高空微分增益（未使用） |
| `Kp_L` | 0.001 | 低空（Low）比例增益 |
| `Ki_L` | 0 | 低空积分增益（未使用） |
| `Kd_L` | 0 | 低空微分增益（未使用） |
| `MaxVel_H` | 0.15 | 高空最大速度（m/s） |
| `MaxVel_L` | 0.05 | 低空最大速度（m/s） |
| `DeadZone` | 0.01 | 速度死区阈值 |

**为什么 Ki=Kd=0（纯 P 控制）**：
- 积分项（Ki）用于消除稳态误差，但视觉伺服中目标一直在移动，积分容易导致超调
- 微分项（Kd）用于预测误差变化趋势，但视觉数据噪声大，微分会放大噪声
- 纯 P 控制简单可靠，对于这个应用场景已经足够

**为什么高空 Kp 比低空大 10 倍**：
- 高空（3.5m）时，飞机离桶远，同样的像素误差对应的实际偏移更大，需要更大的增益快速接近
- 低空（0.8m）时，飞机离桶近，需要更精细的调整，用小增益避免震荡

**为什么高空速度限制 0.15m/s 而低空 0.05m/s**：
- 高空可以快速移动，低空需要缓慢精确调整

**DeadZone 的作用**：当 PID 输出小于 0.01 时，直接置为 0。这是为了防止微小的误差导致飞机不断抖动。

---

## 第57-59行：摄像头参数宏

```cpp
#define CamX 1280
#define CamY 800
#define MinPx 15
```

**解释**：
- `CamX=1280`、`CamY=800`：摄像头分辨率，图像宽度 1280 像素，高度 800 像素
- `MinPx=15`：投放判定阈值（像素）。当桶中心到瞄准中心的距离 ≤ 15 像素时，认为已经对准，可以投放

**为什么是 15 像素**：这是一个经验值。在 0.8m 高度，15 像素对应的实际物理偏移大约在 2-3cm 左右，在桶的开口范围内。

---

## 第62行：时间限制宏

```cpp
#define MaxThrowDuration 120
```

**作用**：最大投放持续时间（秒）。如果瞄准时间超过 120 秒，应该强制投放或放弃。

**注意**：这个宏在当前的 `aim.cpp` 中**并没有被使用**，是一个预留参数。

---

## 第65-70行：PID 控制器结构体

```cpp
struct PIDController
{
    double kp, ki, kd, limit, deadzone;
    double ans_error;
    double previous_error;
};
```

**逐字段解释**：

| 字段 | 类型 | 含义 |
|------|------|------|
| `kp` | double | 比例增益 |
| `ki` | double | 积分增益 |
| `kd` | double | 微分增益 |
| `limit` | double | 输出限幅（当前未使用） |
| `deadzone` | double | 死区（当前未使用） |
| `ans_error` | double | 误差累积（积分项） |
| `previous_error` | double | 上一次误差（微分项） |

**为什么用 `struct` 而不是 `class`**：`struct` 的成员默认是 `public` 的，适合简单的数据容器。这里 PID 控制器只是一个数据集合，没有复杂的封装需求。

**为什么用 `double` 而不是 `float`**：`double` 精度更高（15-16 位有效数字 vs float 的 7 位），在 PID 积分累积过程中可以避免精度损失。

---

## 第72-77行：位置结构体

```cpp
struct Position
{
    double x;
    double y;
    double z;
};
```

**作用**：表示三维空间中的一个点。

**为什么需要它**：`PX4_Position` 全局变量使用这个类型存储飞机的当前位置。注意这里的坐标系是**机体坐标系**（经过 `ENUToBody` 转换后的结果），X=右，Y=前，Z=上。

---

## 第79-87行：目标位速结构体

```cpp
struct GoalPosVel
{
    int  mode;
    double px;
    double py;
    double pz;
    double vx;
    double vy;
};
```

**作用**：存储远程控制（Web）设定的目标位置或速度。

**字段说明**：
- `mode=0`：位置控制模式，使用 `px/py/pz`
- `mode=1`：速度控制模式，使用 `vx/vy/pz`（高度用 pz）

**为什么 mode 是 int 而不是 bool**：虽然只有两种模式，但用 int 可以方便未来扩展更多模式。

---

## 第89-94行：速度结构体

```cpp
struct Velocity
{
    double vx;
    double vy;
    double vz;
};
```

**作用**：表示三维速度向量。

**为什么需要它**：`PX4_Velocity` 全局变量使用这个类型存储飞机的当前速度。注意这里的坐标系是 **ENU 坐标系**（直接从 MAVROS 获取，未转换）。

---

## 第97-100行：全局变量 extern 声明

```cpp
extern struct Position PX4_Position;
extern struct Velocity PX4_Velocity;
extern PIDController pos_pid_xy;
extern double initial_yaw;
```

**语法**：`extern` 关键字告诉编译器"这个变量在其他地方（某个 .cpp 文件）定义了，这里只是声明它的存在"。

**为什么需要 extern**：
- 如果直接在头文件中定义变量（如 `struct Position PX4_Position;`），那么每个包含这个头文件的 .cpp 文件都会定义一次，链接时会报"multiple definition"错误
- 正确的做法是：在**一个** .cpp 文件（`main.cpp`）中定义，在头文件中用 `extern` 声明，其他 .cpp 文件通过 `#include` 头文件来访问

**各变量定义位置**：
- `PX4_Position`、`PX4_Velocity`、`pos_pid_xy`、`initial_yaw` —— 在 `main.cpp` 中定义
- `pos_pid_xy` 是瞄准用的 PID 控制器，在 `aim.cpp` 中使用

---

## 第103-108行：控制函数声明

```cpp
bool PX4_SetMode(std::string);
bool SetMode(std::string);
bool Arm();
bool SetPoint(double, double, double);
bool SetVel(double, double, double);
bool CheckPosition(float x, float y, float z);
```

**逐函数说明**：

| 函数 | 作用 | 定义位置 |
|------|------|---------|
| `PX4_SetMode` | 直接调用 MAVROS 服务切换模式（一次尝试） | `control.cpp` |
| `SetMode` | 循环等待直到模式切换成功 | `control.cpp` |
| `Arm` | 解锁无人机 | `control.cpp` |
| `SetPoint` | 发送位置目标（BODY 坐标系） | `control.cpp` |
| `SetVel` | 发送速度目标（BODY 坐标系） | `control.cpp` |
| `CheckPosition` | 等待飞机到达指定位置（精度 ±0.1m） | `control.cpp` |

**为什么 `SetMode` 和 `PX4_SetMode` 分开**：
- `PX4_SetMode` 是底层的一次性调用
- `SetMode` 是高层封装，会循环重试直到成功，避免因通信延迟导致模式切换失败

---

## 第110行：投放函数

```cpp
bool ThrowBottle(int cmd);
```

**作用**：通过 ROS 服务调用投放指令。

**定义位置**：`control.cpp`

**参数**：`cmd` 取值范围 1-6，对应不同的舵机动作（投桶1、投桶2、开舱1、关舱1、开舱2、关舱2）。

---

## 第112-121行：流程控制函数

```cpp
void ShowPosition(int delay);
bool Positioning(double pZ, int tim, int CenterX, int CenterY, double MaxVel, int BottleLabel);
void Locating();
void Detect();
void GoHome();
void Land();
void TakeOff(double);
void SlowDescend(double, double, double, double);
void SlowMoveForward(double, double, double, double);
```

**逐函数说明**：

| 函数 | 作用 | 定义位置 |
|------|------|---------|
| `ShowPosition` | 每隔1秒打印一次当前位置，持续 `delay` 秒 | `control.cpp` |
| `Positioning` | **核心瞄准函数**：PID 视觉伺服闭环 | `aim.cpp` |
| `Locating` | **瞄准主流程**：飞往投放区→识别→逐桶瞄准投放 | `aim.cpp` |
| `Detect` | 侦察流程：按航线飞行扫描目标区域 | `control.cpp` |
| `GoHome` | 返航：先向后飞再回到起飞点 | `control.cpp` |
| `Land` | 降落：切换到 AUTO.LAND 模式 | `control.cpp` |
| `TakeOff` | 起飞：检查偏差→解锁→飞到初始高度 | `control.cpp` |
| `SlowDescend` | 缓慢下降：从 Lh 逐级下降到 Nh | `control.cpp` |
| `SlowMoveForward` | 缓慢前移：从 Ly 逐级前移到 Ny | `control.cpp` |

---

## 第123行：Web 调参函数

```cpp
void Start_PID_WebTune();
```

**作用**：启动 PID 网络调参服务。

**注意**：这个函数在 `WebServer` 类重构后已经不再使用，但声明保留在这里避免编译错误。实际使用的是 `WebServer::start()`。

---

## 第127-132行：机体→ENU 坐标转换

```cpp
inline void bodyToENU(double x_body, double y_body,
                      double& x_enu, double& y_enu)
{
    x_enu = x_body * sin(initial_yaw) + y_body * cos(initial_yaw);
    y_enu = -x_body * cos(initial_yaw) + y_body * sin(initial_yaw);
}
```

**数学原理**：
这是一个二维旋转矩阵变换。机体坐标系（BODY）到 ENU 坐标系的转换公式：

```
[ x_enu ]   [  sin(ψ)   cos(ψ) ] [ x_body ]
[ y_enu ] = [ -cos(ψ)   sin(ψ) ] [ y_body ]
```

其中 ψ（psi）是偏航角 `initial_yaw`。

**为什么需要这个转换**：
- 飞控使用 ENU 坐标系（东-北-上）
- 人的直觉是机体坐标系（右-前-上）
- 当飞机偏航角不为 0 时，机头方向与 ENU 的北方向有夹角，必须转换

**为什么用 `inline`**：
- `inline` 建议编译器在调用处直接展开函数体，避免函数调用开销
- 这个函数非常小且频繁调用（每帧都可能调用多次），适合内联

**为什么参数用引用 `double&`**：
- 输出参数 `x_enu`、`y_enu` 使用引用传递，函数内部修改它们会直接反映到外部变量
- 避免按值传递的拷贝开销

---

## 第135-140行：ENU→机体坐标转换

```cpp
inline void ENUToBody(double x_enu, double y_enu,
                      double& x_body, double& y_body)
{
    x_body = x_enu * sin(initial_yaw) - y_enu * cos(initial_yaw);
    y_body = x_enu * cos(initial_yaw) + y_enu * sin(initial_yaw);
}
```

**数学原理**：
这是 `bodyToENU` 的逆变换，旋转矩阵的逆等于其转置：

```
[ x_body ]   [  sin(ψ)  -cos(ψ) ] [ x_enu ]
[ y_body ] = [  cos(ψ)   sin(ψ) ] [ y_enu ]
```

**使用场景**：在 `pos_cb` 回调中，从 MAVROS 获取的 ENU 坐标需要转换为机体坐标存储在 `PX4_Position` 中。

---

## 第142行：头文件保护结束

```cpp
#endif // __main_H
```

**作用**：与第1行的 `#ifndef` 配对，结束条件编译块。

---

## 总结：main.h 在整个项目中的角色

`main.h` 是**整个项目的核心头文件**，它扮演了以下角色：

1. **类型定义中心**：定义了 `Position`、`Velocity`、`GoalPosVel`、`PIDController` 等核心数据结构
2. **全局变量声明中心**：通过 `extern` 声明了所有跨文件共享的全局变量
3. **函数声明中心**：声明了所有核心控制函数，供其他文件调用
4. **参数配置中心**：通过 `#define` 集中管理所有可调参数（高度、速度、PID 系数等）
5. **工具函数中心**：提供了坐标系转换的内联函数

**被哪些文件包含**：
- `main.cpp` —— 主程序
- `control.h` —— 控制模块
- `aim.h` —— 瞄准模块
- `Drone.h` —— 状态机
- `communication.h` —— 通信模块
- `WebServer.h` —— Web 服务器
- `calibration.h` —— 标定模块
- `pid.h` —— PID 控制器