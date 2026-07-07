# Drone.h 逐行注释详解

---

## 文件概述

`Drone.h` 定义了**无人机状态机**的核心结构和接口。它是整个控制系统的"大脑"，负责：

1. **定义状态枚举**：无人机的所有可能状态（NONE、WAITING、TAKEOFF、GOAL、RETURN、LAND、ZHENCHA、MIAOZHUN）
2. **定义状态机类**：`Drone` 类封装了状态转移规则、状态执行逻辑、Web 远程控制接口
3. **提供虚函数接口**：允许子类重写各状态的执行逻辑（虽然当前没有子类，但为未来扩展预留）

---

## 第1-2行：头文件保护

```cpp
#ifndef DRONE_H
#define DRONE_H
```

**作用**：防止 `Drone.h` 被多次包含导致重复定义。

**命名规范**：`DRONE_H` 使用大写字母和下划线，这是 C++ 头文件保护的常见命名风格。与 `main.h` 的 `__main_H` 风格略有不同，但功能相同。

---

## 第4行：HTTP 库

```cpp
#include <httplib.h>
```

**作用**：引入 `httplib` 库，这是一个轻量级的 C++ HTTP 库（单头文件）。

**为什么 Drone.h 需要它**：
- `Drone` 类的 `UpdateState()`、`UpdateGoal()`、`Throw()` 三个方法的参数类型是 `const httplib::Request&` 和 `httplib::Response&`
- 这些方法会被 `WebServer` 作为 HTTP 路由回调调用
- 所以 `Drone.h` 必须包含 `httplib.h` 才能编译

**设计思路**：`Drone` 类直接处理 HTTP 请求和响应，而不是通过中间层。这样 `WebServer` 只需要简单地将 HTTP 路由转发给 `Drone` 的方法，职责清晰。

---

## 第5行：字符串

```cpp
#include <string>
```

**作用**：C++ 标准库的 `std::string` 类。

**用途**：
- `GetStateName()` 返回 `std::string` 类型的状态名称
- `UpdateState()` 中解析 HTTP 参数时使用字符串操作

---

## 第6行：映射容器

```cpp
#include <map>
```

**作用**：C++ 标准库的 `std::map` 容器，一种基于红黑树的有序键值对集合。

**用途**：`transition_table_` 成员变量的类型是 `std::map<DroneState, std::set<DroneState>>`，用于存储状态转移规则表。

**为什么用 `map` 而不是其他容器**：
- 需要根据当前状态（key）快速查找允许跳转的目标状态集合（value）
- `map` 的查找时间复杂度为 O(log n)，适合这种查找密集型的场景
- 状态数量很少（8个），用 `map` 或 `unordered_map` 差别不大

---

## 第7行：集合容器

```cpp
#include <set>
```

**作用**：C++ 标准库的 `std::set` 容器，一种基于红黑树的有序不重复元素集合。

**用途**：`transition_table_` 的 value 类型是 `std::set<DroneState>`，表示从某个状态可以跳转到的所有目标状态集合。

**为什么用 `set`**：
- 集合中的元素是唯一的，不会出现重复的状态
- `set` 的 `count()` 方法可以快速判断某个状态是否在集合中（`IsTransitionValid` 中使用）
- 自动排序，便于调试查看

---

## 第8行：核心头文件

```cpp
#include "main.h"
```

**作用**：引入 `main.h` 中定义的所有内容。

**为什么 Drone.h 需要 main.h**：
- 使用 `GoalPosVel` 结构体（`Goal` 成员变量）
- 使用 `Position` 结构体（`extern Position PX4_Position` 在 .cpp 中使用）
- 使用 `#define` 常量（如 `REQ_MAX_POS_X` 等在 `UpdateGoal` 中使用）

---

## 第11-21行：无人机状态枚举

```cpp
enum DroneState
{
    DroneState_NONE     = -1,  // 初始状态
    DroneState_WAITING  = 0,   // 空状态
    DroneState_TAKEOFF  = 1,   // 起飞
    DroneState_GOAL     = 2,   // 飞往目标点
    DroneState_RETURN   = 3,   // 返航
    DroneState_LAND     = 4,   // 降落
    DroneState_ZHENCHA  = 5,   // 侦察
    DroneState_MIAOZHUN = 6    // 瞄准投放
};
```

**语法**：`enum` 是 C/C++ 中的枚举类型，定义一组命名的整型常量。

**为什么 NONE = -1 而 WAITING = 0**：
- `NONE = -1`：表示"未初始化"或"无效"状态，与正常状态（从0开始）区分开
- `WAITING = 0`：表示"空闲等待"状态，是正常的初始状态
- 这样设计的好处是：如果忘记初始化状态变量，默认值 0 会被解释为 WAITING 而不是 NONE，行为更安全

**各状态含义**：

| 状态 | 值 | 含义 | 典型行为 |
|------|-----|------|---------|
| `NONE` | -1 | 初始状态，未做任何操作 | 什么都不做，等待切换到 TAKEOFF |
| `WAITING` | 0 | 空闲等待，悬停待命 | 保持当前位置，等待远程指令 |
| `TAKEOFF` | 1 | 起飞过程 | 解锁→起飞到初始高度 |
| `GOAL` | 2 | 飞往目标点 | 按预设或远程设定的目标飞行 |
| `RETURN` | 3 | 返航 | 飞回起飞点 |
| `LAND` | 4 | 降落 | 切换到 LAND 模式落地 |
| `ZHENCHA` | 5 | 侦察 | 按航线飞行扫描目标区域 |
| `MIAOZHUN` | 6 | 瞄准投放 | 视觉识别→PID瞄准→投放 |

**为什么用 `enum` 而不是 `const int` 或 `enum class`**：
- 传统的 C-style `enum` 是全局可见的，可以直接用 `DroneState_TAKEOFF` 而不需要作用域限定
- C++11 的 `enum class` 更安全（不会隐式转换为 int），但需要 `DroneState::TAKEOFF` 的写法
- 这里使用传统 `enum` 可能是因为代码历史原因，或者为了与旧代码兼容

---

## 第24行：状态机类

```cpp
class Drone
```

**作用**：定义 `Drone` 类，封装无人机状态机的所有逻辑。

**为什么用 `class` 而不是 `struct`**：
- `class` 的成员默认是 `private` 的，适合封装
- `struct` 的成员默认是 `public` 的，适合简单的数据容器
- `Drone` 类有复杂的内部状态和逻辑，需要隐藏实现细节，所以用 `class`

---

## 第27行：构造函数

```cpp
Drone();
```

**作用**：构造函数，在创建 `Drone` 对象时自动调用。

**初始化工作**（在 `Drone.cpp:28-34` 中实现）：
1. 初始化 `current_state_` 为 `DroneState_NONE`
2. 初始化 `previous_state_` 为 `DroneState_NONE`
3. 调用 `BuildTransitionTable()` 构建状态转移规则表
4. 打印初始化日志

**为什么构造函数不接收参数**：
- `Drone` 对象在 `main.cpp` 中以全局变量形式创建（`Drone drone;`）
- 无参构造函数使得创建方式最简单
- 所有配置通过后续的 `setgamemode()` 等方法完成

---

## 第31行：核心状态分发器

```cpp
void HandleState();
```

**作用**：状态机的主入口函数，在 ROS 主循环中每帧调用。

**调用频率**：40Hz（由 `main.cpp` 的 `ros::Rate rate(40.0)` 控制）

**内部逻辑**（在 `Drone.cpp:290-303` 中实现）：
```cpp
switch (current_state_)
{
    case DroneState_NONE:                         break;
    case DroneState_WAITING:  ExecuteWaiting();   break;
    case DroneState_TAKEOFF:  ExecuteTakeOff();   break;
    // ... 其他状态
}
```

**设计模式**：这是一种**状态模式**（State Pattern）的简化实现。每个状态对应一个 `Execute*()` 方法，状态切换通过 `RequestTransition()` 完成。

---

## 第34-36行：Web 远程命令接口

```cpp
void UpdateState(const httplib::Request& req, httplib::Response& res);
void UpdateGoal(const httplib::Request& req, httplib::Response& res);
void Throw(const httplib::Request& req, httplib::Response& res);
```

**作用**：这三个方法作为 HTTP 路由的回调处理函数，接收 Web 请求并返回响应。

**参数说明**：
- `req`：HTTP 请求对象，包含 URL 参数（如 `?newstate=2`）
- `res`：HTTP 响应对象，用于设置返回内容和状态码

**各方法功能**：

| 方法 | HTTP 路由 | 功能 |
|------|----------|------|
| `UpdateState` | `/setstat?newstate=N` | 切换无人机到指定状态 |
| `UpdateGoal` | `/setgoal?mode=0&px=...` | 设置目标位置或速度 |
| `Throw` | `/throw?id=N` | 执行投放指令 |

**为什么参数类型是 httplib 的 Request/Response**：
- 这样 `WebServer` 可以直接将这些方法注册为 HTTP 路由回调
- 不需要额外的适配层或数据转换

---

## 第40-42行：状态查询方法

```cpp
DroneState GetState() const { return current_state_; }
std::string GetStateName() const;
bool IsState(DroneState s) const { return current_state_ == s; }
```

**`const` 关键字的作用**：
- 这些方法被声明为 `const`，表示它们不会修改对象的状态
- 编译器会保证：在 `const` 方法中不能修改任何成员变量
- 这样可以在 `const` 对象上调用这些方法

**各方法用途**：

| 方法 | 返回值 | 用途 |
|------|--------|------|
| `GetState()` | `DroneState` 枚举值 | 获取当前状态的数值，用于程序逻辑判断 |
| `GetStateName()` | `std::string` | 获取当前状态的人类可读名称，用于日志输出 |
| `IsState(s)` | `bool` | 判断当前是否处于指定状态 |

**为什么 `GetState()` 和 `IsState()` 是内联的**：
- 函数体非常简单（只有一行 return 语句）
- 在头文件中定义，编译器可以自动内联，消除函数调用开销
- 这些方法在循环中可能被频繁调用，内联可以提高性能

---

## 第46行：状态转移请求

```cpp
bool RequestTransition(DroneState new_state);
```

**作用**：请求将状态机切换到新状态。这是**唯一**合法的状态切换方式。

**内部流程**（在 `Drone.cpp:59-74` 中实现）：
```
1. 检查转移是否合法（IsTransitionValid）
2. 如果不合法 → 打印错误日志，返回 false
3. 如果合法：
   a. 保存当前状态到 previous_state_
   b. 调用 OnExitState() 退出当前状态
   c. 更新 current_state_ 为新状态
   d. 调用 OnEnterState() 进入新状态
   e. 打印转移日志，返回 true
```

**为什么返回 bool**：
- 调用者需要知道转移是否成功
- 如果转移非法，调用者可以采取相应的错误处理

---

## 第49行：设置游戏模式

```cpp
void setgamemode(bool InGame);
```

**作用**：设置无人机的工作模式，决定状态机是否自动跳转。

**参数**：
- `InGame = true`：比赛模式，状态机自动执行完整流程
- `InGame = false`：练习模式，每个状态执行完后进入 WAITING 等待远程控制

**为什么方法名是 `setgamemode` 而不是 `setGameMode`**：
- 驼峰命名不规范，可能是历史遗留或笔误
- 但已经这样写了，修改会导致所有调用处都需要修改

---

## 第51-52行：雷达数据接口

```cpp
void update_Lidar(int lidar_data);   //更新雷达数据
int get_Lidar();                     //获取当前雷达数据
```

**作用**：提供激光雷达高度数据的存取接口。

**为什么需要这两个方法**：
- `update_Lidar()` 在 `lidar_cb` 回调中被调用，存储最新的雷达高度
- `get_Lidar()` 在瞄准模块中被调用，获取当前高度用于标定查询

**单位**：厘米（cm），在 `update_Lidar` 中从米转换为厘米（`*100`）。

---

## 第53行：protected 访问权限

```cpp
protected:
```

**作用**：`protected` 访问权限意味着这些成员可以被：
1. 本类的方法访问
2. 子类（派生类）的方法访问
3. **不能**被外部代码访问

**为什么用 `protected` 而不是 `private`**：
- 这些是虚函数（`virtual`），设计意图是允许子类重写
- 如果设为 `private`，子类就无法访问或重写它们
- 设为 `protected` 既隐藏了实现细节，又保留了扩展性

---

## 第59行：进入状态钩子

```cpp
virtual void OnEnterState(DroneState state);
```

**`virtual` 关键字的作用**：
- 声明这是一个**虚函数**，允许子类重写（override）
- 当通过基类指针或引用调用时，会执行实际对象的版本（多态）

**用途**：在进入某个状态时执行一次性的初始化操作，如：
- 打印日志
- 初始化状态相关的变量
- 发送初始指令

**当前实现**（`Drone.cpp:306-317`）：
- 打印绿色日志 `">>> 进入: 状态名"`
- 如果是 GOAL 状态，将目标点设置为当前位置（防止乱飘）

---

## 第62行：退出状态钩子

```cpp
virtual void OnExitState(DroneState state);
```

**作用**：在离开某个状态时执行一次性的清理操作。

**当前实现**（`Drone.cpp:319-322`）：
- 打印绿色日志 `"<<< 退出: 状态名"`
- 没有其他清理逻辑

---

## 第65-81行：各状态执行函数

```cpp
virtual void ExecuteTakeOff();
virtual void ExecuteGoal();
virtual void ExecuteReturn();
virtual void ExecuteLand();
virtual void ExecuteWaiting();
virtual void ExecuteZhencha();
virtual void ExecuteMiaozhun();
```

**作用**：每个状态对应的执行函数，在 `HandleState()` 中被调用。

**为什么都是 `virtual`**：
- 允许子类重写特定状态的执行逻辑
- 例如，可以创建一个 `CompetitionDrone` 子类，重写 `ExecuteMiaozhun()` 使用不同的瞄准算法
- 这是**模板方法模式**（Template Method Pattern）的体现

**各函数行为简述**：

| 函数 | 行为 |
|------|------|
| `ExecuteTakeOff()` | 调用 `TakeOff(5)` → 比赛模式转 MIAOZHUN，练习模式转 WAITING |
| `ExecuteGoal()` | 根据 mode 调用 `SetPoint()` 或 `SetVel()` |
| `ExecuteReturn()` | 调用 `GoHome()` → 转 LAND |
| `ExecuteLand()` | 调用 `Land()` → 转 NONE |
| `ExecuteWaiting()` | 什么都不做（空函数） |
| `ExecuteZhencha()` | 调用 `Detect()` → 比赛模式转 RETURN，练习模式转 WAITING |
| `ExecuteMiaozhun()` | 调用 `Locating()` → 比赛模式转 ZHENCHA，练习模式转 WAITING |

---

## 第85-86行：状态变量

```cpp
DroneState current_state_;
DroneState previous_state_;
```

**作用**：
- `current_state_`：存储当前状态
- `previous_state_`：存储上一个状态，用于日志和回退

**为什么需要 `previous_state_`**：
- 在 `RequestTransition()` 中，切换状态前将当前状态保存到 `previous_state_`
- 这样在日志中可以打印 "从 A 转移到 B" 的完整信息
- 未来可以用于实现"返回上一个状态"的功能

---

## 第88行：目标航点

```cpp
GoalPosVel Goal;      // 目标航点
```

**作用**：存储通过 Web 远程控制设置的目标位置或速度。

**类型**：`GoalPosVel` 结构体（定义在 `main.h:79-87`），包含：
- `mode`：0=位置模式，1=速度模式
- `px/py/pz`：目标位置坐标
- `vx/vy`：目标速度

**使用场景**：
- `UpdateGoal()` 中从 HTTP 参数解析并存储
- `ExecuteGoal()` 中读取并发送给飞控

---

## 第89行：比赛模式标志

```cpp
bool ingame;          // 比赛模式标志，true=比赛模式，false=练习模式
```

**作用**：控制状态机的自动跳转行为。

**影响**：
- `true`：`ExecuteTakeOff()` → 自动转 MIAOZHUN → 自动转 ZHENCHA → 自动转 RETURN → 自动转 LAND
- `false`：每个 `Execute*()` 执行完后都转 WAITING，等待 Web 远程控制

---

## 第90行：雷达高度

```cpp
int lidar_height;     //雷达返回的高度(已转换成cm)
```

**作用**：存储激光雷达测量的高度数据，单位是厘米。

**为什么是 int 而不是 double**：
- 标定数据文件中的高度是整数（如 59、64、71...）
- 厘米精度对于高度测量已经足够
- `int` 比较和索引操作更高效

---

## 第92行：状态转移表

```cpp
std::map<DroneState, std::set<DroneState>> transition_table_;
```

**类型分析**：
- `std::map<Key, Value>`：键值对映射
- Key 是 `DroneState`（当前状态）
- Value 是 `std::set<DroneState>`（允许跳转到的目标状态集合）

**数据结构示例**：
```
{
    NONE     → {TAKEOFF},
    WAITING  → {GOAL, MIAOZHUN, ZHENCHA, RETURN, LAND},
    TAKEOFF  → {WAITING, ZHENCHA, RETURN, LAND},
    GOAL     → {MIAOZHUN, ZHENCHA, RETURN, LAND},
    MIAOZHUN → {ZHENCHA, RETURN, LAND},
    ZHENCHA  → {RETURN, LAND},
    RETURN   → {LAND},
    LAND     → {NONE}
}
```

**为什么需要转移表**：
- 防止非法状态转移（如从 NONE 直接跳到 LAND）
- 集中管理所有转移规则，便于维护和修改
- 在 `RequestTransition()` 中统一校验，避免每个 `Execute*()` 都做重复检查

---

## 第95行：构建转移表

```cpp
void BuildTransitionTable();
```

**作用**：在构造函数中调用，初始化 `transition_table_`。

**为什么不在构造函数中直接初始化**：
- 将初始化逻辑抽取为单独的函数，代码更清晰
- 如果未来需要重新初始化转移表，可以直接调用此函数

---

## 第98行：判断转移合法性

```cpp
bool IsTransitionValid(DroneState from, DroneState to) const;
```

**作用**：检查从 `from` 状态转移到 `to` 状态是否合法。

**实现逻辑**（`Drone.cpp:51-56`）：
```cpp
if (from == to) return true;  // 相同状态总是合法
auto it = transition_table_.find(from);
return it != transition_table_.end() && it->second.count(to);
```

**为什么相同状态总是合法**：
- 在某些情况下，可能需要在同一状态下重复执行
- 允许"自环"转移可以简化某些逻辑

---

## 第101行：头文件保护结束

```cpp
#endif // DRONE_H
```

**作用**：与第1行的 `#ifndef` 配对，结束条件编译块。

---

## 总结：Drone.h 的设计模式

### 状态模式（State Pattern）

`Drone` 类实现了简化的状态模式：
- **上下文**（Context）：`Drone` 类本身
- **状态**（State）：`DroneState` 枚举
- **状态行为**：`Execute*()` 方法
- **状态转移**：`RequestTransition()` + `transition_table_`

### 模板方法模式（Template Method Pattern）

- `HandleState()` 是模板方法，定义了算法的骨架
- `Execute*()` 是具体步骤，子类可以重写
- `OnEnterState()` / `OnExitState()` 是钩子方法

### 与 WebServer 的协作

```
Web 请求 → WebServer (HTTP) → Drone::UpdateState/UpdateGoal/Throw
                                  ↓
                              RequestTransition()
                                  ↓
                              HandleState() 循环调用
                                  ↓
                              Execute*() 执行具体逻辑
```

### 与 main.cpp 的关系

```
main.cpp 创建 Drone 对象
  → 设置比赛模式 (setgamemode)
  → 40Hz 循环调用 HandleState()
    → 状态机自动推进
    → 调用 control.cpp 的函数控制飞控
    → 调用 aim.cpp 的函数执行瞄准