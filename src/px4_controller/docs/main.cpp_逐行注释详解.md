# main.cpp 逐行注释详解

---

## 文件概述

`main.cpp` 是**整个无人机控制系统的入口点**（Entry Point）。它负责：

1. **初始化** ROS 节点、全局变量、通信服务
2. **加载** 外部参数（比赛模式）和标定数据
3. **创建** 关键对象：状态机（`Drone`）、Web服务器（`WebServer`）、遥测广播（`TelemetryBroadcaster`）
4. **建立** 所有 ROS 通信渠道（服务客户端 + 话题订阅）
5. **启动** 后台服务线程（Web + 遥测）
6. **连接** 飞控并进入主循环

---

## 第1行：包含核心头文件

```cpp
#include "main.h"                   //大部分全局变量
```

**语法**：`#include "main.h"` 使用双引号，表示从**当前文件所在目录**开始查找头文件。

**作用**：引入 `main.h` 中定义的所有内容：
- 类型定义（`Position`、`Velocity`、`PIDController`、`GoalPosVel`）
- `#define` 常量（高度、速度、PID参数等）
- `extern` 全局变量声明
- 函数声明
- 内联工具函数（`bodyToENU`、`ENUToBody`）

**为什么这样写**：`main.cpp` 是主控文件，几乎所有的其他模块都要通过 `main.h` 来访问共享类型和变量。如果 `main.h` 不放在第一行，后面的头文件可能因为找不到依赖的类型定义而编译失败。

---

## 第2行：Web 服务器

```cpp
#include "WebServer.h"              //PID网络调参函数
```

**作用**：引入 `WebServer` 类的定义。`WebServer` 封装了一个 HTTP 服务器，提供 Web API 供上位机远程控制无人机。

**提供了什么**：`WebServer` 类，支持：
- `/setpid` —— 在线修改 PID 参数
- `/setstat` —— 切换无人机状态
- `/setgoal` —— 设置目标点
- `/throw` —— 投放指令

**与 main.cpp 的关系**：`main.cpp` 中创建了 `WebServer webserver(drone)` 对象并调用 `webserver.start()`。

---

## 第3行：UDP 遥测广播

```cpp
#include "TelemetryBroadcaster.h"   //UDP遥测广播
```

**作用**：引入 `TelemetryBroadcaster` 类的定义。这个类负责通过 UDP 协议向地面站广播无人机的实时遥测数据（位置、速度、状态等）。

**为什么需要它**：地面站需要实时显示无人机的飞行数据，而 ROS 通信依赖于同一台机器上的 roscore。UDP 广播可以跨网络传输，适合无线通信场景。

---

## 第4行：通信模块

```cpp
#include "communication.h"          //通信相关函数
```

**作用**：引入 `communication.h`，它包含了所有 ROS 通信对象的 `extern` 声明以及回调函数的声明。

**包含了什么**：
- `arming_client`、`set_mode_client`、`set_position_client`、`throw_client` —— ROS 服务客户端
- `state_sub`、`visual_sub`、`pos_sub`、`imu_sub`、`vel_sub`、`lidar_sub` —— ROS 话题订阅者
- `current_state`、`imu_msg`、`vel_msg` —— 全局状态变量
- `state_cb`、`pos_cb`、`visual_cb`、`imu_cb`、`vel_cb`、`lidar_cb` —— 回调函数声明

---

## 第5行：瞄准模块

```cpp
#include "aim.h"                    //瞄准相关函数
```

**作用**：引入瞄准模块的函数声明。

**包含了什么**：
- `Positioning()` —— 核心 PID 视觉伺服瞄准函数
- `Locating()` —— 瞄准主流程（识别 + 逐桶投放）
- `BarrelPosition` 结构体
- `robustKMeans()` 聚类函数

---

## 第6行：控制模块

```cpp
#include "control.h"                //飞控控制指令相关函数
```

**作用**：引入控制模块的函数声明。

**包含了什么**：
- `ConnectPX4()` —— 连接飞控
- `TakeOff()` —— 起飞
- `SetPoint()` / `SetVel()` —— 位置/速度控制
- `ThrowBottle()` —— 投放
- `Detect()` —— 侦察
- `GoHome()` —— 返航
- `Land()` —— 降落
- `SlowDescend()` / `SlowMoveForward()` —— 缓慢移动

---

## 第7行：状态机

```cpp
#include "Drone.h"                  //无人机状态机相关函数
```

**作用**：引入 `Drone` 类的定义。

**包含了什么**：
- `Drone` 类（状态机）
- `DroneState` 枚举（NONE、WAITING、TAKEOFF、GOAL、RETURN、LAND、ZHENCHA、MIAOZHUN）
- 状态转移表

---

## 第8行：参数标定器

```cpp
#include "calibration.h"            //参数标定器
```

**作用**：引入 `HeightCalibration` 类的定义和 `loadCalibrationFromFile` 函数。

**用途**：读取 `calibration_data.txt` 中的高度→像素坐标映射数据，用于在低空瞄准时将 LiDAR 高度转换为对应的瞄准中心像素坐标。

---

## 第9-13行：全局变量定义

```cpp
// ── 全局变量定义 (main.h 中 extern 声明) ──────
struct Position PX4_Position;              //当前飞机坐标，由回调函数更新
struct Velocity PX4_Velocity;              //当前飞机速度，由回调函数更新
PIDController pos_pid_xy;
double initial_yaw;
```

**为什么这些变量在这里定义而不是在头文件中**：

这是 C/C++ 中全局变量管理的标准做法：
1. 在 **头文件**（`main.h`）中用 `extern` 声明："存在这么一个变量，类型是 XXX，在其他地方定义了"
2. 在 **一个 .cpp 文件**（`main.cpp`）中实际定义："这个变量就在这里，给它分配内存"

如果直接在头文件中定义（去掉 `extern`），那么每个包含这个头文件的 .cpp 文件都会生成一个独立的变量定义，链接时就会报"multiple definition"（重复定义）错误。

**PX4_Position 的更新流程**：
```
MAVROS 飞控 → /mavros/local_position/pose 话题
  → pos_cb() 回调函数 (communication.cpp)
    → ENUToBody 转换坐标系
      → PX4_Position.x/y/z 被更新
```

**PX4_Velocity 的更新流程**：
```
MAVROS 飞控 → /mavros/local_position/velocity_local 话题
  → vel_cb() 回调函数 (communication.cpp)
    → PX4_Velocity.vx/vy/vz 被更新
```

**pos_pid_xy 的用途**：这是瞄准 PID 控制器的实例，在 `aim.cpp` 的 `Positioning()` 函数中使用。它可以通过 WebServer 的 `/setpid` 路由在线调整。

**initial_yaw 的初始化**：在 `imu_cb` 回调中，从第一次接收到的 IMU 四元数中提取偏航角并赋值。一旦赋值，`yaw_initialized` 变为 `true`，后续 IMU 数据不再更新它。

---

## 第15行：状态机对象

```cpp
Drone drone;                                   // 定义无人机状态机对象
```

**作用**：创建 `Drone` 类的全局实例。这是整个程序的核心对象。

**构造函数做了什么**（详见 `Drone.cpp:28-34`）：
1. 初始化 `current_state_` 为 `DroneState_NONE`（-1）
2. 初始化 `previous_state_` 为 `DroneState_NONE`
3. 调用 `BuildTransitionTable()` 构建状态转移规则表
4. 打印绿色日志表示初始化成功

**为什么是全局对象**：
- 多个模块需要访问它：`WebServer` 需要引用它来处理 Web 命令，`communication.cpp` 中的 `pos_cb` 在触发电子围栏时需要调用它的 `RequestTransition`
- 它的生命周期必须覆盖整个程序运行期间

---

## 第16行：Web 服务器对象

```cpp
WebServer webserver(drone);                    // 定义网络服务器对象
```

**作用**：创建 `WebServer` 类的实例，并将上面创建的 `drone` 对象作为参数传入。

**语法分析**：
- `WebServer` 的构造函数接受一个 `Drone&` 引用（详见 `WebServer.h:20`）
- `webserver(drone)` 表示调用构造函数，将 `drone` 的引用存储起来
- 因为 `WebServer` 删除（delete）了拷贝构造函数和赋值运算符（`WebServer.h:23-24`），所以不能复制 `WebServer` 对象

**为什么传递 drone 引用**：
- `WebServer` 需要在 HTTP 回调中调用 `drone.UpdateState()`、`drone.UpdateGoal()`、`drone.Throw()` 等方法
- 传递引用而不是指针，语义上表示"这个 WebServer 关联到一个必须存在的 Drone"

---

## 第17行：遥测广播对象

```cpp
TelemetryBroadcaster telemetry_broadcaster;    // UDP 遥测广播对象
```

**作用**：创建 `TelemetryBroadcaster` 类的实例，负责将无人机状态通过 UDP 协议广播到地面站。

**与 WebServer 对比**：
- `WebServer` 是**请求-响应**模式（HTTP）：地面站主动请求，服务器响应
- `TelemetryBroadcaster` 是**发布-订阅**模式（UDP）：无人机主动广播，地面站被动接收

---

## 第19行：标定对象

```cpp
HeightCalibration calib;
```

**作用**：创建 `HeightCalibration` 类的实例，用于存储和查询高度→像素坐标的映射数据。

**这个对象会被谁使用**：
- `main.cpp`：调用 `loadCalibrationFromFile()` 加载数据到 `calib`
- `aim.cpp`：`extern HeightCalibration calib;` 声明后，在 `Locating()` 中通过 `calib.query()` 获取对应高度的瞄准中心
- `cvisual.cpp`：有一个独立的 `HeightCalibration calib`（同名但不同作用域），在显示窗口中绘制标定点

---

## 第21行：main 函数入口

```cpp
int main(int argc,char *argv[])
```

**语法**：
- `main` 是 C/C++ 程序的入口函数
- `argc`（argument count）：命令行参数的数量，至少为 1（程序名本身）
- `argv`（argument vector）：字符串数组，存储每个命令行参数

**返回值 `int`**：程序退出码，`0` 表示正常退出，非零表示异常退出。

---

## 第23行：ROS 节点初始化

```cpp
ros::init(argc,argv,"main");
```

**作用**：初始化 ROS 节点。这是使用 ROS 的第一步，必须在任何其他 ROS 调用之前执行。

**参数分析**：
- `argc`、`argv`：传递命令行参数，ROS 会解析其中的 `__name:=xxx`、`__ns:=xxx` 等特殊参数
- `"main"`：节点名称。在 ROS 网络中，节点名必须唯一，如果已经有一个名为 "main" 的节点在运行，当前节点会自动在名称后添加数字（如 "main_1"）

**背后发生了什么**：
1. ROS 注册该节点到 roscore
2. 设置节点的命名空间
3. 初始化线程相关资源

**为什么是 "main"**：这是主控制节点的名称，视觉节点命名为 "visual_node"（在 `cvisual.cpp` 中），投放节点命名为 "throw"（在 `throw.cpp` 中），位置发布节点命名为 "SetPoint"（在 `SetPoint.cpp` 中）。每个节点都是独立的进程。

---

## 第24行：设置中文 locale

```cpp
setlocale(LC_ALL,"");
```

**作用**：设置程序的本地化环境，使程序能够正确显示中文字符。

**参数分析**：
- `LC_ALL`：表示设置所有类别（包括字符编码、数字格式、时间格式等）
- `""`：空字符串表示使用系统默认的 locale 设置

**为什么需要它**：ROS 的日志输出可能包含中文（如 `"[Drone] 非法转移"`），如果不设置 locale，控制台可能显示乱码。

---

## 第25行：ROS 时间初始化

```cpp
ros::Time::init();
```

**作用**：初始化 ROS 时间系统。

**为什么需要显式调用**：
- `ros::Time::now()` 需要在 ROS 节点初始化后才能正常工作
- 在没有 `ros::NodeHandle` 的情况下，`ros::Time::init()` 提供了必要的时间初始化
- 在较新版本的 ROS 中，`ros::init()` 已经包含了时间初始化，但显式调用更安全

---

## 第26行：创建节点句柄

```cpp
ros::NodeHandle nh;
```

**作用**：创建与 ROS 系统交互的句柄（Handle）。这是使用 ROS 功能的"门户"。

**可以通过 `nh` 做什么**：
- `nh.subscribe(...)` —— 订阅话题
- `nh.advertise(...)` —— 发布话题
- `nh.serviceClient(...)` —— 创建服务客户端
- `nh.advertiseService(...)` —— 创建服务端（本例中未使用）
- `nh.param(...)` —— 读取 ROS 参数服务器中的参数

**生命周期**：当 `nh` 对象被析构（超出作用域）时，ROS 会自动清理该节点创建的所有发布者、订阅者和服务。这里 `nh` 在 `main()` 函数中，所以它的生命周期覆盖整个程序。

---

## 第29-31行：读取比赛模式参数

```cpp
/*>>>外部参数<<<*/
bool InGame;
nh.param<bool>("InGame", InGame, true);   // 比赛模式，默认启动
drone.setgamemode(InGame);
```

**nh.param 语法分析**：
```cpp
nh.param<bool>("参数名", 存储变量, 默认值);
```
- 从 ROS 参数服务器读取名为 `"InGame"` 的参数
- 类型为 `bool`
- 如果参数不存在，使用默认值 `true`
- 将读取的值存入 `InGame` 变量

**参数设置方式**：这个参数可以在 launch 文件中设置：
```xml
<param name="InGame" value="false"/>  <!-- 练习模式 -->
```

**setgamemode 的作用**（详见 `Drone.cpp:373-378`）：
- `InGame = true`（比赛模式）：状态机自动跳转，无人机自动执行 NONE → TAKEOFF → MIAOZHUN → ZHENCHA → RETURN → LAND 的完整流程
- `InGame = false`（练习模式）：每个状态执行完毕后进入 WAITING 状态，等待 Web 远程控制

---

## 第33-42行：加载标定数据

```cpp
// 加载标定参数
std::string package_path = ros::package::getPath("px4_controller");
std::string file_path = package_path + "/config/calibration_data.txt";
if (loadCalibrationFromFile(file_path, calib)) 
{
    ROS_INFO_STREAM("\033[32m" << "[Calibration] 标定参数读取成功!" << "\033[0m");
}
else
{
    ROS_ERROR("[Calibration] 标定参数读取错误!!!");
}
```

**第33行**：获取功能包路径
- `ros::package::getPath("px4_controller")` 返回该包在文件系统中的绝对路径
- 在编译后的 devel 空间中，可能是 `~/catkin_ws/src/px4_controller`
- 在安装后的环境中，可能是 `~/catkin_ws/install/share/px4_controller`

**第34行**：拼接标定文件路径
- 使用 `+` 运算符拼接字符串：包路径 + "/config/calibration_data.txt"
- 最终结果例如：`/home/user/catkin_ws/src/px4_controller/config/calibration_data.txt`

**第35行**：加载标定数据
- `loadCalibrationFromFile` 函数（定义在 `calibration.cpp:88-139`）：
  1. 打开文件
  2. 逐行读取，跳过注释行（以 `#` 开头）和空行
  3. 按空格或逗号分隔解析每一行的 `高度 x1 y1 x2 y2`
  4. 调用 `calib.addCalibrationData()` 添加数据
  5. 调用 `calib.sortAndValidate()` 按高度排序
  6. 返回 `true`（成功）或 `false`（失败）

**第37行**：成功日志（绿色）
- `"\033[32m"` 是 ANSI 转义码，设置终端文本颜色为绿色
- `"\033[0m"` 重置颜色
- `ROS_INFO_STREAM` 支持 `<<` 运算符流式输出

**第41行**：失败日志
- `ROS_ERROR` 以红色显示错误信息

**为什么标定文件加载失败不会导致程序退出**：
- 标定数据主要用于低空瞄准的精确落点计算
- 如果加载失败，低空瞄准会使用硬编码的 `Throw1_X/Y` 和 `Throw2_X/Y`（在 `main.h` 中定义）
- 程序仍然可以运行，只是低空瞄准精度会下降

---

## 第44-47行：创建 ROS 服务客户端

```cpp
/*>>>全局变量赋值<<<*/
arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
set_position_client = nh.serviceClient<px4_controller::position>("SendPosition");
throw_client = nh.serviceClient<px4_controller::throwcmd>("ThrowCmd");
```

**语法分析**：
- `nh.serviceClient<服务类型>("服务名称")` 创建一个服务客户端
- 模板参数 `<mavros_msgs::CommandBool>` 指定服务类型
- 参数 `"/mavros/cmd/arming"` 是服务在 ROS 网络中的名称

**各服务客户端用途**：

| 客户端 | 服务名 | 服务类型 | 作用 |
|--------|--------|---------|------|
| `arming_client` | `/mavros/cmd/arming` | `mavros_msgs::CommandBool` | 解锁/上锁飞控 |
| `set_mode_client` | `/mavros/set_mode` | `mavros_msgs::SetMode` | 切换飞行模式 |
| `set_position_client` | `SendPosition` | `px4_controller::position` | 发送位置/速度指令到 SetPoint 节点 |
| `throw_client` | `ThrowCmd` | `px4_controller::throwcmd` | 发送投放指令到 throw 节点 |

**为什么 `arming_client` 和 `set_mode_client` 的服务名以 `/` 开头**：
- `/mavros/cmd/arming` 是**全局名称**（以 `/` 开头），不受节点命名空间影响
- `SendPosition` 是**相对名称**，会根据节点的命名空间解析
- MAVROS 的服务通常使用全局名称，因为 mavros 节点有自己的命名空间

---

## 第48-53行：创建话题订阅

```cpp
state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
visual_sub=nh.subscribe("IR",1,visual_cb);////队列是一
pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,pos_cb);
imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,imu_cb);
vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,vel_cb);
lidar_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub",10,lidar_cb);
```

**语法分析**：
```cpp
nh.subscribe<消息类型>("话题名称", 队列大小, 回调函数);
```
- 模板参数：指定消息的数据类型
- 话题名称：订阅的话题在 ROS 网络中的名称
- 队列大小：消息队列的缓冲区大小。如果处理速度跟不上接收速度，超过队列大小的旧消息会被丢弃
- 回调函数：每当有新消息到达时，ROS 会调用这个函数

**各订阅者详解**：

| 订阅者 | 话题名 | 消息类型 | 队列 | 回调 | 发布频率 | 作用 |
|--------|--------|---------|------|------|---------|------|
| `state_sub` | `mavros/state` | `mavros_msgs::State` | 10 | `state_cb` | ~10Hz | 获取飞控连接/解锁/模式状态 |
| `visual_sub` | `IR` | `px4_controller::tbag` | **1** | `visual_cb` | ~30Hz | 获取视觉检测结果 |
| `pos_sub` | `/mavros/local_position/pose` | `geometry_msgs::PoseStamped` | 10 | `pos_cb` | ~30Hz | 获取当前位置（ENU） |
| `imu_sub` | `/mavros/imu/data` | `sensor_msgs::Imu` | 10 | `imu_cb` | ~100Hz | 获取 IMU 数据（用于提取偏航角） |
| `vel_sub` | `/mavros/local_position/velocity_local` | `geometry_msgs::TwistStamped` | 10 | `vel_cb` | ~30Hz | 获取当前速度 |
| `lidar_sub` | `/mavros/distance_sensor/hrlv_ez4_pub` | `sensor_msgs::Range` | 10 | `lidar_cb` | ~20Hz | 获取激光雷达高度数据 |

**为什么 `visual_sub` 的队列大小是 1**：
```cpp
visual_sub=nh.subscribe("IR",1,visual_cb);////队列是一
```
- 注释特意强调了"队列是一"
- 视觉数据更新频率很高（30Hz），而且是控制闭环的关键输入
- 使用队列大小 1 意味着只保留最新的检测结果，旧的未处理数据直接丢弃
- 这样可以确保瞄准模块总是使用最新的视觉数据，而不是处理过时的数据

**为什么其他订阅者队列是 10**：
- 位置、速度、状态等数据变化相对平缓
- 队列大小 10 可以缓冲短暂的处理延迟，不会丢失关键数据

---

## 第54行：初始化时间戳

```cpp
last_request=ros::Time::now();
```

**作用**：初始化 `last_request` 全局变量（定义在 `control.cpp:4`）。

**为什么需要它**：
- `last_request` 用于控制模式切换和解锁等操作的频率
- 在 `SetMode()` 和 `Arm()` 中，每次发送请求前检查 `ros::Time::now() - last_request > ros::Duration(1.0)`，确保每隔至少 1 秒才发送一次请求
- 这避免了在通信延迟或飞控未响应时，以毫秒级频率疯狂发送请求导致飞控过载

**为什么在订阅者之后初始化**：
- 确保所有回调函数已经注册
- 这样在后续的操作中，回调函数可以立即开始接收数据更新全局变量

---

## 第55行：设置循环频率

```cpp
ros::Rate rate(40.0);
```

**作用**：创建一个 `ros::Rate` 对象，用于控制主循环的频率为 40Hz。

**语法**：`ros::Rate rate(频率)` 参数单位是赫兹（Hz）。

**为什么是 40Hz**：
- 40Hz 意味着主循环每 25 毫秒执行一次
- 这个频率与 `SetPoint.cpp` 节点发布位置/速度指令的频率（30Hz）接近
- 不需要太快，因为飞控的位置控制本身有内部 PID 环，外部指令频率 20-50Hz 已经足够
- 也不可以太慢，否则状态机响应不及时

**如何使用**：在循环末尾调用 `rate.sleep()`，它会根据自上次 `sleep()` 以来的时间自动调整休眠时长，确保循环以稳定的 40Hz 运行。

---

## 第57-58行：启动后台服务

```cpp
webserver.start();
telemetry_broadcaster.start();
```

**webserver.start() 内部发生了什么**（详见 `WebServer.cpp:12-31`）：
1. 检查是否已经启动（防止重复启动）
2. 创建新线程执行 `run_server()` 方法
3. 在新线程中：
   - 创建 `httplib::Server` 实例
   - 注册路由：`/setpid`、`/setstat`、`/setgoal`、`/throw`
   - 设置 `server_running_ = true` 通知主线程
   - 调用 `svr.listen("0.0.0.0", 8880)` 开始监听
4. 主线程等待 `server_running_` 变为 `true`
5. 分离线程（`detach()`），让其独立运行

**telemetry_broadcaster.start() 内部发生了什么**：
- 类似地创建线程，定期读取遥测快照并通过 UDP 广播

**为什么在 `ConnectPX4()` 之前启动**：
- Web 服务器和遥测广播与飞控连接是独立的
- 即使飞控还没连接，地面站也需要能访问 Web 界面
- 提前启动可以确保在连接飞控的过程中，地面站就能看到状态更新

---

## 第60行：连接飞控

```cpp
ConnectPX4();
```

**ConnectPX4() 内部执行流程**（详见 `control.cpp:134-149`）：

```
1. 循环等待飞控连接 (current_state.connected == true)
   - 每1秒打印一次 "Attempt To Connect PX4"
   
2. 连接成功后，发送初始悬停指令
   SetPoint(0, 0, InitialHeight)  → 飞到 (0, 0, 3.5m)

3. 切换到 POSCTL 模式（位置控制模式）
   - POSCTL 是 PX4 的基础模式，操作手可以通过遥控器接管
   
4. 切换到 OFFBOARD 模式（外部控制模式）
   - 只有 OFFBOARD 模式下才能接受 ROS 发布的位置/速度指令
```

**为什么需要先 POSCTL 再 OFFBOARD**：
- PX4 的安全设计不允许直接从某些模式（如 MANUAL）切换到 OFFBOARD
- POSCTL 是一个中间模式，从 MANUAL → POSCTL → OFFBOARD 的转换路径是安全的
- 经验表明，直接切 OFFBOARD 有时会失败，两步切换更可靠

**ConnectPX4 是阻塞的**：
- 如果飞控没有启动或没有连接，程序会卡在这个循环里
- 这是故意的：无人机没有飞控连接就无法飞行，应该等待

---

## 第62-67行：状态机主循环

```cpp
// ── 状态机主循环 ──────────────────────────
while (ros::ok())
{
    drone.HandleState();    // 每帧分发到当前状态的 Execute*()
    ros::spinOnce();
    rate.sleep();
}
```

**while (ros::ok()) 的作用**：
- `ros::ok()` 返回 `false` 的情况：
  1. 收到 SIGINT 信号（用户按 Ctrl+C）
  2. ROS 网络关闭（roscore 停止）
  3. `ros::shutdown()` 被调用
- 当 `ros::ok()` 返回 `false` 时，循环退出，程序结束

**drone.HandleState() 内部**（详见 `Drone.cpp:290-303`）：
```cpp
void Drone::HandleState()
{
    switch (current_state_)
    {    
    case DroneState_NONE:                         break;
    case DroneState_WAITING:  ExecuteWaiting();   break;
    case DroneState_TAKEOFF:  ExecuteTakeOff();   break;
    case DroneState_GOAL:     ExecuteGoal();      break;
    case DroneState_RETURN:   ExecuteReturn();    break;
    case DroneState_LAND:     ExecuteLand();      break;
    case DroneState_ZHENCHA:  ExecuteZhencha();   break;
    case DroneState_MIAOZHUN: ExecuteMiaozhun();  break;
    }
}
```
- 根据当前状态调用对应的 `Execute*()` 方法
- 每个 `Execute*()` 方法内部可能会调用 `RequestTransition()` 来切换状态
- 主循环以 40Hz 的频率不断调用 `HandleState()`，状态机在其中不断推进

**ros::spinOnce() 的作用**：
- 处理所有等待中的 ROS 回调函数（执行 `state_cb`、`pos_cb`、`visual_cb` 等）
- **关键**：如果不调用 `spinOnce()`，回调函数永远不会被执行，`PX4_Position` 等全局变量永远不会被更新
- 与 `ros::spin()` 的区别：`spin()` 是阻塞的，会一直处理回调不返回；`spinOnce()` 处理一次就返回，适合自定义循环

**rate.sleep() 的作用**：
- 根据设定的 40Hz 频率控制循环节奏
- 计算自上次 `sleep()` 以来的时间，休眠剩余时间
- 如果处理时间超过了 25ms（40Hz 的周期），则不休眠直接进入下一帧

**主循环的执行顺序**：
```
1. drone.HandleState()   → 执行当前状态的逻辑（可能触发状态转移）
2. ros::spinOnce()       → 处理所有回调，更新全局变量
3. rate.sleep()          → 等待到下一个 40Hz 周期
```

**为什么先 HandleState 再 spinOnce**：
- 先执行状态逻辑，确保状态机向前推进
- 再处理回调，接收最新的飞控数据
- 这样下一帧的 HandleState 使用的是最新的数据

---

## 第68行：程序结束

```cpp
}
```

**作用**：`main` 函数结束，程序退出。

**退出时自动发生的事**：
- `drone` 对象析构 —— 但 `Drone` 没有定义析构函数，所以没有特殊清理
- `webserver` 对象析构 —— 但 `WebServer::stop()` 是空的，HTTP 线程是 `detach()` 的，会继续运行
- `telemetry_broadcaster` 对象析构 —— 类似情况
- `nh` 节点句柄析构 —— ROS 自动清理所有客户端和订阅者
- `ros::shutdown()` 被自动调用

**潜在问题**：
- HTTP 服务线程和 UDP 广播线程是 `detach()` 的，主线程退出后它们可能还在运行
- 在 ROS 节点中，`ros::ok()` 返回 `false` 后主循环退出，`main()` 结束，进程终止，所有线程自然终止
- 但严格来说，应该先 `stop()` 这些后台线程再进行清理

---

## 总结：main.cpp 的完整执行流

```
程序启动
  │
  ├─ ros::init()         → 注册为 "main" 节点
  ├─ setlocale()         → 支持中文
  ├─ ros::Time::init()   → 时间系统初始化
  ├─ ros::NodeHandle     → 创建通信句柄
  │
  ├─ 读取 InGame 参数    → 设置比赛/练习模式
  ├─ 加载标定数据        → 读取 calibration_data.txt
  │
  ├─ 创建服务客户端      → arming / set_mode / position / throw
  ├─ 创建话题订阅        → state / visual / pos / imu / vel / lidar
  ├─ 初始化时间戳        → last_request = now
  │
  ├─ webserver.start()          → 启动 HTTP 服务器（新线程）
  ├─ telemetry_broadcaster.start() → 启动 UDP 广播（新线程）
  │
  ├─ ConnectPX4()        → 阻塞等待飞控连接并切换到 OFFBOARD
  │
  └─ while(ros.ok())     → 40Hz 主循环
       ├─ drone.HandleState()   → 执行状态机逻辑
       ├─ ros::spinOnce()       → 处理 ROS 回调
       └─ rate.sleep()          → 等待下一帧
```

---

## 与其他文件的联系

| 全局变量/对象 | 在哪些文件中被使用 |
|--------------|------------------|
| `drone` | `main.cpp`（主循环）、`WebServer.cpp`（路由回调）、`communication.cpp`（电子围栏） |
| `webserver` | 仅在 `main.cpp` 中创建和启动 |
| `telemetry_broadcaster` | 仅在 `main.cpp` 中创建和启动 |
| `calib` | `main.cpp`（加载数据）、`aim.cpp`（extern 声明后使用） |
| `PX4_Position` | `main.h`（extern）、`communication.cpp`（pos_cb 更新）、`control.cpp`（起飞检查）、`aim.cpp`（位置日志） |
| `PX4_Velocity` | `main.h`（extern）、`communication.cpp`（vel_cb 更新）、`aim.cpp`（速度日志） |
| `pos_pid_xy` | `main.h`（extern）、`aim.cpp`（PID 瞄准）、`WebServer.cpp`（在线调参） |
| `initial_yaw` | `main.h`（extern）、`communication.cpp`（imu_cb 初始化）、`control.cpp`（坐标转换） |

---

## 关键设计决策总结

1. **全局变量模式**：使用全局变量在模块间共享数据（位置、速度、PID 参数），而不是通过 ROS 话题传递。优点是延迟低、代码简单；缺点是模块耦合度高。

2. **多进程架构**：`main`、`SetPoint`、`throw`、`cvisual` 是四个独立的 ROS 节点，通过 ROS 服务/话题通信。这样某个节点崩溃不会影响其他节点。

3. **后台线程**：Web 服务器和 UDP 广播在独立线程中运行，不阻塞主控制循环。

4. **阻塞式飞控连接**：`ConnectPX4()` 是阻塞的，确保飞控就绪后才进入主循环。

5. **40Hz 主循环**：平衡了响应速度和 CPU 负载，与飞控的 OFFBOARD 控制频率匹配。