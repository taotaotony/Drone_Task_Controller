# Drone.cpp 逐行注释详解

---

## 文件概述

`Drone.cpp` 是 `Drone` 类的实现文件，实现了 `Drone.h` 中声明的所有方法。它是**无人机状态机的核心逻辑**，负责：

1. **状态转移管理**：构建转移表、验证转移合法性、执行状态切换
2. **Web 远程控制**：处理 HTTP 请求，切换状态、设置目标、执行投放
3. **各状态执行逻辑**：起飞、目标飞行、返航、降落、侦察、瞄准的具体行为
4. **比赛/练习模式控制**：根据模式决定状态机是否自动跳转

---

## 第1行：文件注释

```cpp
// Drone.cpp — 无人机状态机实现
```

**作用**：文件级别的注释，说明本文件的内容。

**为什么这样写**：良好的代码习惯，方便其他开发者快速了解文件的作用。

---

## 第2行：包含头文件

```cpp
#include "Drone.h"
```

**作用**：包含 `Drone` 类的声明。这是实现文件必须包含的头文件。

**为什么用双引号**：`Drone.h` 是项目内部的头文件，与 `Drone.cpp` 在同一目录下，使用双引号从当前目录开始查找。

---

## 第3行：WebServer 头文件

```cpp
#include "WebServer.h"
```

**作用**：包含 `WebServer` 类的声明。

**为什么 Drone.cpp 需要它**：
- 虽然 `Drone` 类本身不直接使用 `WebServer`，但 `WebServer` 中定义的宏（如 `REQ_MAX_POS_X`、`REQ_MAX_VEL_X` 等）在 `UpdateGoal()` 方法中被使用
- 这些宏定义在 `WebServer.h` 中（第11-15行）

**依赖关系说明**：
```
Drone.cpp 需要 WebServer.h 中的 REQ_MAX_* 宏
WebServer.h 需要 Drone.h 中的 Drone 类定义
```
这是合理的依赖关系：`Drone.cpp` 需要 `WebServer.h` 中的宏定义来验证目标坐标的合法性。

**定义的位置**：`WebServer.h` 中定义了以下边界限制宏：
- `REQ_MAX_VEL_X = 3` —— X方向最大速度限制 3m/s
- `REQ_MAX_VEL_Y = 3` —— Y方向最大速度限制 3m/s
- `REQ_MAX_POS_X = 10` —— X方向最大坐标限制 10m
- `REQ_MAX_POS_Y = 100` —— Y方向最大坐标限制 100m
- `REQ_MAX_POS_Z = 5` —— Z方向最大坐标限制 5m

---

## 第4行：控制模块

```cpp
#include "control.h"       // TakeOff, GoHome, Land, Detect, SetPoint...
```

**作用**：包含控制模块的函数声明。

**Drone.cpp 中具体使用的函数**：
| 函数 | 调用位置 | 作用 |
|------|---------|------|
| `TakeOff(5)` | `ExecuteTakeOff()` | 起飞，等待5秒 |
| `GoHome()` | `ExecuteReturn()` | 执行返航流程 |
| `Land()` | `ExecuteLand()` | 执行降落流程 |
| `Detect()` | `ExecuteZhencha()` | 执行侦察流程 |
| `SetPoint()` | `ExecuteGoal()`、`OnEnterState()` | 发布位置控制指令 |
| `SetVel()` | `ExecuteGoal()` | 发布速度控制指令 |
| `ThrowBottle()` | `Throw()` | 发送投放指令 |

---

## 第5行：瞄准模块

```cpp
#include "aim.h"           // Locating, Positioning...
```

**作用**：包含瞄准模块的函数声明。

**使用的函数**：
- `Locating()` —— 在 `ExecuteMiaozhun()` 中调用，执行完整的瞄准投放流程

---

## 第6行：通信模块

```cpp
#include "communication.h" // current_state
```

**作用**：包含通信模块的声明。

**为什么需要它**：
- `communication.h` 中包含了 `extern Drone drone;` 声明（虽然 `Drone.cpp` 自身不需要）
- 包含了 `extern struct VisualData VisualData;` 等变量的声明
- 但这些在 `Drone.cpp` 中没有直接使用，这个包含可能是为了确保所有依赖的全局变量都可访问

---

## 第7行：ROS 核心

```cpp
#include <ros/ros.h>
```

**作用**：ROS 核心库。

**为什么需要它**：
- `ROS_INFO_STREAM`、`ROS_WARN`、`ROS_ERROR` —— 日志输出宏
- `ros::spinOnce()` —— 在 `OnEnterState()` 中调用，确保回调函数被处理
- `ros::Time` —— 虽然本文件未直接使用，但 ROS 日志宏依赖时间系统

---

## 第9行：extern 声明

```cpp
extern Position PX4_Position;
```

**作用**：声明 `PX4_Position` 全局变量是在其他地方定义的。

**为什么需要这个 extern**：
- `PX4_Position` 在 `main.cpp` 中定义
- `Drone.cpp` 的 `OnEnterState()` 中需要读取 `PX4_Position.x/y/z` 来设置初始目标点
- 目的是当进入 GOAL 状态时，将目标点设置为无人机当前位置，防止突然切换到目标模式时飞机乱飘

**为什么 main.h 已经有 extern 了还要再声明一次**：
- `main.h` 中确实已经有 `extern struct Position PX4_Position;`（`main.h:97`）
- 这里重复声明可能是因为 `Drone.cpp` 早期版本没有包含 `main.h`（注意 `Drone.h` 已经包含了 `main.h`，所以 `Drone.cpp` 通过 `Drone.h` 间接包含了 `main.h`）
- 这种重复声明虽然不会导致编译错误，但属于代码冗余

---

## 第10-25行：匿名命名空间中的辅助函数

```cpp
namespace {
std::string StateName(DroneState s)
{
    switch (s) {
    case DroneState_NONE:     return "NONE";
    case DroneState_WAITING:  return "WAITING";
    case DroneState_TAKEOFF:  return "TAKEOFF";
    case DroneState_GOAL:     return "GOAL";
    case DroneState_RETURN:   return "RETURN";
    case DroneState_LAND:     return "LAND";
    case DroneState_ZHENCHA:  return "ZHENCHA";
    case DroneState_MIAOZHUN: return "MIAOZHUN";
    default:                  return "UNKNOWN";
    }
}
}
```

**语法分析**：
- `namespace { ... }` —— **匿名命名空间**（unnamed namespace）
- 内部的 `StateName` 函数具有**内部链接**（internal linkage），只在当前编译单元（`Drone.cpp`）中可见
- 相当于 C 语言中的 `static` 关键字

**为什么用匿名命名空间而不是 `static`**：
- C++ 中建议用匿名命名空间替代 `static` 来限定翻译单元内可见性
- 匿名命名空间的效果：`StateName` 只能在 `Drone.cpp` 中使用，其他 .cpp 文件无法链接到这个函数
- 这样即使其他文件也定义了同名的 `StateName` 函数，也不会产生链接冲突

**StateName 函数的作用**：
- 将 `DroneState` 枚举值转换为人类可读的字符串
- 用于日志输出（ROS_INFO/ROS_ERROR/ROS_WARN）
- 用于 HTTP 响应内容

**为什么用 `switch` 而不是 `map`**：
- `switch` 更简洁直观
- 编译器可以将 `switch` 优化为跳转表（jump table），性能更高
- 枚举值连续（-1到6），适合 `switch` 优化

**为什么 MIAOZHUN 用汉语拼音**：
- ZHENCHA（侦察）和 MIAOZHUN（瞄准）都是汉语拼音
- 可能是因为开发者习惯用中文思考，但代码中只能用 ASCII 字符

---

## 第27-34行：构造函数

```cpp
// ── 构造函数 ──────────────────────────────────
Drone::Drone()
    : current_state_(DroneState_NONE)
    , previous_state_(DroneState_NONE)
{
    BuildTransitionTable();
    ROS_INFO_STREAM("\033[32m" << "[Drone] StateMachine Initialized, Current State: " << StateName(current_state_) << "\033[0m");
}
```

**语法分析**：
```cpp
Drone::Drone()
    : current_state_(DroneState_NONE)    // 初始化列表
    , previous_state_(DroneState_NONE)   // 初始化列表
```
- `Drone::Drone()` —— 使用作用域解析运算符 `::`，表示这是 `Drone` 类的构造函数
- `: current_state_(val), previous_state_(val)` —— **成员初始化列表**（Member Initializer List）
- 初始化列表在构造函数体**之前**执行

**初始化列表 vs 构造函数体内赋值**：
```cpp
// 初始化列表方式（正确）
Drone::Drone() : current_state_(DroneState_NONE) {}

// 构造函数体内赋值方式（也可行，但效率稍低）
Drone::Drone() { current_state_ = DroneState_NONE; }
```
- 对于内置类型（如枚举、int、double），两种方式没有实质区别
- 对于类类型（如 `std::string`、`std::map`），初始化列表可以避免先默认构造再赋值的两次操作

**初始化顺序**：
- `current_state_` 先初始化（因为它在类定义中先声明）
- `previous_state_` 后初始化
- 初始化顺序由**类定义中成员的声明顺序**决定，而不是初始化列表中的顺序

**为什么初始化为 NONE**：
- `NONE = -1` 是一个特殊值，表示"未初始化"
- 与正常的 `WAITING = 0` 区分开
- 这样在 `HandleState()` 中 `case DroneState_NONE: break;` 会先什么都不做，等待第一次状态转移

**构造函数体中的操作**：
1. `BuildTransitionTable()` —— 构建状态转移规则表
2. ROS 日志输出 —— 打印绿色初始化成功信息

**为什么不把初始化列表中的内容放到函数体内**：
- 习惯问题：C++ 中初始化列表是初始化成员变量的标准方式
- 对于 const 成员和引用类型成员，**必须**使用初始化列表

---

## 第36行：GetStateName

```cpp
std::string Drone::GetStateName() const { return StateName(current_state_); }
```

**作用**：返回当前状态的人类可读名称。

**`const` 关键字**：
- 表示这个函数不会修改 `Drone` 对象的任何成员变量
- 可以在 `const Drone` 对象上调用

**实现方式**：
- 直接调用匿名命名空间中的 `StateName()` 辅助函数
- 这是一个简单的委托调用（delegating call）

---

## 第38-49行：构建转移表

```cpp
// ── 转移表 ────────────────────────────────────
void Drone::BuildTransitionTable()
{
    transition_table_[DroneState_NONE]     = { DroneState_TAKEOFF };
    transition_table_[DroneState_WAITING]  = { DroneState_GOAL, DroneState_MIAOZHUN, DroneState_ZHENCHA, DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_TAKEOFF]  = { DroneState_WAITING, DroneState_ZHENCHA, DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_GOAL]     = { DroneState_MIAOZHUN, DroneState_ZHENCHA, DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_MIAOZHUN] = { DroneState_ZHENCHA, DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_ZHENCHA]  = { DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_RETURN]   = { DroneState_LAND };
    transition_table_[DroneState_LAND]     = { DroneState_NONE };
}
```

**`transition_table_` 的类型**：
```cpp
std::map<DroneState, std::set<DroneState>> transition_table_;
```
这是一个映射表：Key 是当前状态，Value 是允许跳转到的目标状态集合。

**语法分析**：
```cpp
transition_table_[DroneState_NONE] = { DroneState_TAKEOFF };
```
- `operator[]` 是 `std::map` 的重载运算符，用于访问或插入元素
- 如果 key 不存在，`operator[]` 会**自动创建一个空值**（`std::set<DroneState>()`），然后赋值
- `{ DroneState_TAKEOFF }` 是**初始化列表语法**（C++11），用于构造 `std::set`

**完整转移规则图解**：
```
NONE ─────→ TAKEOFF ─────→ WAITING ─────→ GOAL ─────→ MIAOZHUN ─────→ ZHENCHA ─────→ RETURN ─────→ LAND
              │               │              │              │              │              │
              ├→ WAITING      ├→ MIAOZHUN     ├→ MIAOZHUN    ├→ ZHENCHA     ├→ RETURN      └→ NONE
              ├→ ZHENCHA      ├→ ZHENCHA      ├→ ZHENCHA     ├→ RETURN      └→ LAND
              ├→ RETURN       ├→ RETURN       ├→ RETURN      └→ LAND
              └→ LAND         └→ LAND         └→ LAND
```

**设计思路分析**：

| 状态 | 可转移到 | 设计原因 |
|------|---------|---------|
| NONE | TAKEOFF | 初始化后只能起飞 |
| WAITING | 5个目标 | 悬停等待时可以切换到任何任务模式 |
| TAKEOFF | 4个目标 | 起飞后可以开始任何任务 |
| GOAL | 4个目标 | 飞行中可中断去瞄准/侦察/返航/降落 |
| MIAOZHUN | 3个目标 | 瞄准后可侦察/返航/降落 |
| ZHENCHA | 2个目标 | 侦察后只能返航或降落 |
| RETURN | 1个目标 | 返航中只能降落 |
| LAND | 1个目标 | 降落后回到初始状态 |

**状态转移的不可逆性**：
- 一旦进入 RETURN，只能 LAND，不能再回到其他任务状态
- 一旦进入 LAND，只能回到 NONE
- 这种单向性保证了任务流程的确定性

**为什么 WAITING 不能回 NONE**：
- NONE 是"未初始化"状态，WAITING 是"空闲"状态
- 从逻辑上讲，空闲的无人机不应该回到未初始化状态
- 如果需要重置，通过 LAND→NONE 的路径

---

## 第51-56行：转移合法性验证

```cpp
bool Drone::IsTransitionValid(DroneState from, DroneState to) const
{
    if (from == to) return true;
    auto it = transition_table_.find(from);
    return it != transition_table_.end() && it->second.count(to);
}
```

**`const` 关键字**：表示这个方法不会修改对象状态，可以安全地在任何上下文中调用。

**逻辑分析**：

1. **相同状态总是合法**：`if (from == to) return true;`
   - 允许"自环"转移
   - 在某些情况下，可能会重复请求同一个状态
   - 不需要在转移表中为每个状态添加自环项

2. **查找当前状态**：`auto it = transition_table_.find(from);`
   - `auto` 自动推导为 `std::map<DroneState, std::set<DroneState>>::iterator`
   - `find()` 的时间复杂度为 O(log n)

3. **检查目标状态是否在集合中**：`it->second.count(to)`
   - `it->second` 是 `std::set<DroneState>`，即允许转移的目标状态集合
   - `count()` 返回元素在集合中出现的次数（对于 set，只能是 0 或 1）
   - > 0 表示合法转移

**为什么用 `count()` 而不是 `find()`**：
- `set::count()` 和 `set::find()` 的时间复杂度都是 O(log n)
- `count()` 返回 0 或 1，语义更简洁
- 这里只需要知道"是否存在"，`count()` 更适合

**边界情况**：
- 如果 `from` 不在转移表中（例如新增的状态但忘记更新转移表），`find()` 返回 `end()`，函数返回 `false`
- 这提供了一定程度的容错性

---

## 第58-74行：状态转移主函数

```cpp
// ── 状态转移 ──────────────────────────────────
bool Drone::RequestTransition(DroneState new_state)
{
    if (!IsTransitionValid(current_state_, new_state))
    {
        ROS_ERROR("[Drone] 非法转移: %s -> %s",
                  StateName(current_state_).c_str(), StateName(new_state).c_str());
        return false;
    }
    previous_state_ = current_state_;
    OnExitState(current_state_);
    current_state_ = new_state;
    OnEnterState(current_state_);
    ROS_WARN("[Drone] 转移: %s -> %s",
             StateName(previous_state_).c_str(), StateName(current_state_).c_str());
    return true;
}
```

**执行流程**：

```
1. 检查合法性
   ├─ 非法 → 打印错误日志，返回 false
   └─ 合法 → 继续执行

2. 保存旧状态
   previous_state_ = current_state_  （记住"我从哪里来"）

3. 退出旧状态
   OnExitState(current_state_)       （清理即将离开的状态）

4. 更新当前状态
   current_state_ = new_state        （切换到新状态）

5. 进入新状态
   OnEnterState(current_state_)      （初始化新状态）

6. 打印转移日志
   ROS_WARN("转移: 旧状态 -> 新状态")

7. 返回成功
   return true
```

**为什么用 `ROS_WARN` 而不是 `ROS_INFO` 打印成功转移**：
- `ROS_WARN` 在终端中显示为黄色，更容易从大量日志中辨识
- 状态转移是重要的系统事件，突出显示有助于调试

**`StateName().c_str()` 的作用**：
- `StateName()` 返回 `std::string`
- `.c_str()` 将其转换为 C 风格的 `const char*`
- 因为 `ROS_ERROR()` 和 `ROS_WARN()` 的格式化参数 **不支持** `std::string`，必须是 `const char*`

**为什么不把 `OnExitState` 放在状态切换之后**：
- 必须在状态切换前调用 `OnExitState`，因为我们需要知道**正在退出的是哪个状态**
- 如果在切换后再调用，传递的参数就是新状态了

---

## 第76-112行：UpdateState Web 命令

```cpp
// ── Web 远程命令 ──────────────────────────────
// 更新状态
void Drone::UpdateState(const httplib::Request& req, httplib::Response& res)
{
    if (!req.has_param("newstate"))
    {
        res.set_content("Param ERROR! Missing 'newstate'", "text/plain");
        res.status = 400; return;
    }
    try
    {
        int raw = std::stoi(req.get_param_value("newstate"));
        if (raw < 0 || raw > 5)
        {
            res.set_content("Param ERROR! newstate must be 0-5", "text/plain");
            res.status = 400; return;
        }
        DroneState target = static_cast<DroneState>(raw);
        if (RequestTransition(target))
        {
            res.set_content("OK -> " + GetStateName(), "text/plain");
            res.status = 200;
        }
        else
        {
            res.set_content("Denied: " + StateName(current_state_) +
                            " -> " + StateName(target) + " not allowed",
                            "text/plain");
            res.status = 403;
        }
    }
    catch (...)
    {
        res.set_content("Param ERROR!", "text/plain");
        res.status = 400;
    }
}
```

**参数类型**：
- `const httplib::Request& req` —— HTTP 请求，包含 URL 查询参数
- `httplib::Response& res` —— HTTP 响应，用于返回结果

**HTTP 状态码使用**：

| 状态码 | 含义 | 使用场景 |
|--------|------|---------|
| 200 | OK | 状态切换成功 |
| 400 | Bad Request | 缺少参数、参数格式错误、参数值越界 |
| 403 | Forbidden | 状态转移被拒绝（非法转移） |

**为什么 `newstate` 的取值范围是 0-5 而不是 0-6**：
- 从 Web 端不能切换到 NONE（-1）和 MIAOZHUN（6）
- NONE 是初始化状态，不需要手动切换
- MIAOZHUN（6）是自动流程中的状态，不允许 Web 手动切换
- 实际上 Web 可以切换的状态是：WAITING(0), TAKEOFF(1), GOAL(2), RETURN(3), LAND(4), ZHENCHA(5)

**`try-catch (...)` 的作用**：
- `std::stoi()` 如果参数不是有效的数字，会抛出 `std::invalid_argument` 或 `std::out_of_range` 异常
- `catch (...)` 捕获**所有类型**的异常
- 这是一种防御性编程，确保即使出现意外异常也不会导致程序崩溃

**`static_cast<DroneState>(raw)` 的安全性**：
- 从 `int` 到枚举的 `static_cast` 是**不安全的**
- 如果 `raw` 的值不在枚举定义的范围（-1到6）内，转换后的值可能无效
- 但是前面已经检查了 `raw >= 0 && raw <= 5`，所以在这个特定场景下是安全的

**为什么直接返回 StateName 而不是用 JSON**：
- 简单的文本响应对于 Web 调试已经足够
- 降低了对上位机开发的要求

---

## 第114-217行：UpdateGoal Web 命令

```cpp
//更新目标
void Drone::UpdateGoal(const httplib::Request& req, httplib::Response& res)
{
    if(current_state_ != DroneState_GOAL)
    {
        res.set_content("Denied: Switch to GOAL Mode First!", "text/plain");
        res.status = 400; return;
    }
    // ... 后续参数解析
}
```

**先决条件检查**：
- 只有当前状态是 `DroneState_GOAL` 时才能更新目标
- 这样可以避免在起飞或返航过程中意外改变目标

**两种模式**：

**模式 0：位置控制**（第128-168行）
```
参数: mode=0, px=X, py=Y, pz=Z
验证: 0 ≤ pz ≤ 5, |px| ≤ 10, |py| ≤ 100
存储: Goal.mode=0, Goal.px/py/pz
```

**模式 1：速度控制**（第170-216行）
```
参数: mode=1, vx=VX, vy=VY, pz=Z
验证: 0 ≤ pz ≤ 5, |vx| ≤ 3, |vy| ≤ 3
存储: Goal.mode=1, Goal.vx/vy, Goal.pz
```

**边界验证的设计**：
- `pz`（高度）限制在 0-5 米，防止飞太高或钻地
- `px/py`（位置）限制在 ±10/±100 米，防止飞出场地
- `vx/vy`（速度）限制在 ±3 m/s，防止飞控饱和

**发现的问题**：
- 第189行错误提示写的是 `vx must be -` + `REQ_MAX_VEL_X` + `-` + `REQ_MAX_POS_X`
- 最后的 `REQ_MAX_POS_X` 应该是 `REQ_MAX_VEL_X`（3），这是一个 **bug**
- 同样的，第194行错误提示中 `res.get_content` 显示"py must be..."但实际检查的是 `vy`

**一个明显的 bug 在第160行和第202行**：
```cpp
snprintf(buf, sizeof(buf), "OK Set Goal POS %.4f %.4f %.4f", new_vx, new_vy, new_pz);
```
- 速度模式下（mode=1），格式字符串写的是 `POS`（位置）但传入的是 `vx, vy, pz`
- 应该写成 `"OK Set Goal VEL %.4f %.4f %.4f"` 更合理

---

## 第219-287行：Throw Web 命令

```cpp
// Throw
void Drone::Throw(const httplib::Request& req, httplib::Response& res)
{
    if (!req.has_param("id"))
    {
        res.set_content("Param ERROR! Missing 'id'", "text/plain");
        res.status = 400; return;
    }
    try
    {
        int id = std::stoi(req.get_param_value("id"));
        switch(id)
        {
            case 1: ThrowBottle(1); res.set_content("Throw 1 OK", ...); break;
            case 2: ThrowBottle(2); res.set_content("Throw 2 OK", ...); break;
            case 3: ThrowBottle(3); res.set_content("Open 1 OK", ...); break;
            case 4: ThrowBottle(4); res.set_content("Close 1 OK", ...); break;
            case 5: ThrowBottle(5); res.set_content("Open 2 OK", ...); break;
            case 6: ThrowBottle(6); res.set_content("Close 2 OK", ...); break;
            default: // 错误处理
        }
    }
    catch (...) { /* 错误处理 */ }
}
```

**指令映射**：

| id | 调用参数 | 响应内容 | 实际动作 |
|----|---------|---------|---------|
| 1 | `ThrowBottle(1)` | "Throw 1 OK" | 投放桶1 |
| 2 | `ThrowBottle(2)` | "Throw 2 OK" | 投放桶2 |
| 3 | `ThrowBottle(3)` | "Open 1 OK" | 打开舱门1 |
| 4 | `ThrowBottle(4)` | "Close 1 OK" | 关闭舱门1 |
| 5 | `ThrowBottle(5)` | "Open 2 OK" | 打开舱门2 |
| 6 | `ThrowBottle(6)` | "Close 2 OK" | 关闭舱门2 |

**设计意图**：
- 1 和 2 是完整的一次性投放（开舱+投+关舱）
- 3-4 和 5-6 是手动控制舱门开关，用于调试或特殊操作

**每个 case 都有 `return` 和 `break`**：
- `return` 会直接退出函数，后面的 `break` 实际上不会执行
- 这是冗余代码，`return` 和 `break` 只需要一个
- 但这样写也没有什么坏处，属于防御性编程

---

## 第289-303行：核心分发器 HandleState

```cpp
// ── 核心分发器 ────────────────────────────────
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

**调用频率**：40Hz（在 `main.cpp` 的主循环中调用）

**设计模式**：这是**状态模式**的核心——根据当前状态分发给对应的处理函数。

**为什么 NONE 是 `break` 而不是 `ExecuteNone()`**：
- NONE 是"什么都不做"的状态
- 创建一个空的 `ExecuteNone()` 函数只会增加不必要的调用开销
- `break` 直接跳出 switch，什么都不执行

**为什么不检查状态转移的返回值**：
- `ExecuteTakeOff()` 等函数内部会调用 `RequestTransition()` 来自动推进状态
- `HandleState()` 只负责分发，不关心状态转移的结果
- 状态转移的结果在下一次 `HandleState()` 调用时自然体现

**这种设计的局限性**：
- 所有 `Execute*()` 函数必须是**非阻塞**的，否则会阻塞主循环
- 实际上 `TakeOff(5)` 是阻塞的（休眠5秒），这会阻塞整个主循环
- 更好的设计应该是：每个 `Execute*()` 只执行一步操作，用状态机内部的状态来跟踪进度

---

## 第305-322行：Entry / Exit 钩子

```cpp
// ── Entry / Exit 钩子 ─────────────────────────
void Drone::OnEnterState(DroneState state)
{
    ROS_INFO_STREAM("\033[32m" << "[Drone] >>> 进入: " << StateName(state) << "\033[0m");
    if(state == DroneState_GOAL) 
    {
        ros::spinOnce();
        Goal.mode = 0;
        Goal.px = PX4_Position.x;
        Goal.py = PX4_Position.y;
        Goal.pz = PX4_Position.z;
        SetPoint(Goal.px, Goal.py, Goal.pz);
    } // 切入目标模式时将目标点设置为当前的坐标，防止乱飘
}

void Drone::OnExitState(DroneState state)
{
    ROS_INFO_STREAM("\033[32m" << "[Drone] <<< 退出: " << StateName(state) << "\033[0m");
}
```

**OnEnterState 的特殊处理——GOAL 状态**：
- `ros::spinOnce()`：强制处理一次 ROS 回调，获取最新的 `PX4_Position`
- 将目标点设置为当前位置：`Goal.px = PX4_Position.x` 等
- 立即发送位置指令：`SetPoint(Goal.px, Goal.py, Goal.pz)`

**为什么进入 GOAL 时要锁住当前位置**：
- GOAL 是"飞往目标点"模式
- 如果不先锁住当前位置，`Goal` 中的值可能是上次残留的旧数据
- 切换到 GOAL 模式的瞬间，如果目标点突然跳到某个远方，飞机会猛地冲过去
- 先设为当前位置，飞机保持悬停，再由 `UpdateGoal()` 设置真正的目标

**为什么只有 GOAL 有特殊处理**：
- 其他状态不需要预设参数
- TAKEOFF 调用 `TakeOff(5)` 内部已经处理
- MIAOZHUN 调用 `Locating()` 内部已经处理
- ZHENCHA 调用 `Detect()` 内部已经处理

**OnExitState 的实现**：
- 目前只打印日志，没有实际的清理逻辑
- 这为未来扩展预留了接口

---

## 第324-329行：ExecuteTakeOff

```cpp
void Drone::ExecuteTakeOff()
{
    TakeOff(5);
    if(ingame) {RequestTransition(DroneState_MIAOZHUN);}    
    else {RequestTransition(DroneState_WAITING);}           
}
```

**执行流程**：
1. 调用 `TakeOff(5)` —— 起飞并等待5秒稳定
2. 根据模式决定下一个状态：
   - 比赛模式（`ingame=true`）：→ 瞄准投放（MIAOZHUN）
   - 练习模式（`ingame=false`）：→ 等待（WAITING）

**`TakeOff(5)` 内部做了什么**（详见 `control.cpp:152-183`）：
1. 发送位置指令 `SetPoint(0, 0, InitialHeight)` 飞到 3.5m
2. 检查当前位置偏差：
   - 偏差小（±0.2m）：2秒后解锁起飞
   - 偏差中等（±0.8m）：5秒后解锁起飞（警告）
   - 偏差大（>0.8m）：10秒后解锁起飞（严重警告，但不禁止）
3. 调用 `Arm()` 解锁
4. 调用 `ShowPosition(waittime)` 等待并显示位置

**问题**：`ExecuteTakeOff()` 每次被调用都会执行一次完整的起飞流程，但起飞只需要做一次。40Hz 的循环意味着它在40帧内会被调用40次，但 `TakeOff(5)` 内部有 while 循环和长时间休眠，所以实际上只有第一次调用会真正执行，后续调用会被阻塞在 `TakeOff` 的内部循环中。这种设计虽然能工作，但不优雅。

---

## 第331-341行：ExecuteGoal

```cpp
void Drone::ExecuteGoal()
{
    if(Goal.mode == 0)
    {
        SetPoint(Goal.px, Goal.py, Goal.pz);
    }
    else
    {
        SetVel(Goal.vx, Goal.vy, Goal.pz);
    }
}
```

**作用**：每帧（40Hz）向飞控发送目标位置或速度指令。

**为什么每帧都要发送**：
- PX4 的 OFFBOARD 模式要求以 >2Hz 的频率持续接收外部指令
- 如果停止发送指令超过 0.5 秒，飞控会自动退出 OFFBOARD 模式
- 所以每帧都要调用 `SetPoint()` 或 `SetVel()` 保持通信

**模式选择**：
- `Goal.mode == 0`：发送位置指令 `SetPoint(x, y, z)`
- `Goal.mode == 1`：发送速度指令 `SetVel(vx, vy, z)`

**不阻塞**：这个函数只发送指令，不等待到达目标点，所以它可以被 40Hz 循环反复调用。

---

## 第343-346行：ExecuteWaiting

```cpp
void Drone::ExecuteWaiting()
{
    return;
}
```

**作用**：什么都不做，飞机保持当前状态（悬停）。

**为什么是空函数**：
- WAITING 状态表示"等待远程指令"
- 飞机应该保持当前的位置和姿态
- 由于之前最后一次发送的指令是 `SetPoint()` 或 `SetVel()`，飞控会继续执行最后一个指令
- 所以不需要做任何事

---

## 第347-351行：ExecuteReturn

```cpp
void Drone::ExecuteReturn()
{
    GoHome();
    RequestTransition(DroneState_LAND);
}
```

**执行流程**：
1. 调用 `GoHome()` —— 执行返航流程
2. 请求切换到 LAND 状态

**`GoHome()` 内部做了什么**（详见 `control.cpp:239-245`）：
1. `SetVel(0, -5, DetectHeight)` —— 以 5m/s 速度向后飞（Y负方向）
2. `ShowPosition(11)` —— 等待 11 秒（约飞 55 米）
3. `SetPoint(0, 0, DetectHeight)` —— 回到起飞点
4. `ShowPosition(4)` —— 等待 4 秒稳定

**问题**：和 `ExecuteTakeOff()` 一样，`GoHome()` 是阻塞的。40Hz 循环下，只有第一次调用会实际执行，后续调用被阻塞在 `GoHome()` 的内部循环中。

---

## 第353-357行：ExecuteLand

```cpp
void Drone::ExecuteLand()
{
    Land();
    RequestTransition(DroneState_NONE);
}
```

**执行流程**：
1. 调用 `Land()` —— 切换到 AUTO.LAND 模式
2. 请求切换到 NONE 状态

**`Land()` 内部做了什么**（详见 `control.cpp:249-253`）：
1. `SetMode("AUTO.LAND")` —— 切换到 PX4 的自动降落模式
2. `ros::Duration(10).sleep()` —— 等待 10 秒让飞机落地

**降落完成后的状态**：
- 切换到 NONE（初始状态）
- 如果再次起飞，需要重新从 NONE → TAKEOFF → ... 的流程

---

## 第359-364行：ExecuteZhencha

```cpp
void Drone::ExecuteZhencha()
{
    Detect();
    if(ingame) {RequestTransition(DroneState_RETURN);}    
    else {RequestTransition(DroneState_WAITING);}    
}
```

**执行流程**：
1. 调用 `Detect()` —— 执行侦察航线飞行
2. 根据模式决定下一个状态：
   - 比赛模式：→ 返航（RETURN）
   - 练习模式：→ 等待（WAITING）

**`Detect()` 内部做了什么**（详见 `control.cpp:216-235`）：
1. 飞到侦察区起点：`SetPoint(0, TakeofftoThrow, DetectHeight)`
2. 快速前移：`SetVel(0, 4, DetectHeight)` 飞 5.5 秒
3. 到达侦察区：`SetPoint(0, TakeofftoDetect, DetectHeight)`
4. S形航线扫描：
   - 左移（-0.5m/s）6秒
   - 悬停 2 秒
   - 右移（+0.5m/s）12秒
   - 左移（-0.5m/s）6秒
5. 回到侦察区起点

---

## 第366-371行：ExecuteMiaozhun

```cpp
void Drone::ExecuteMiaozhun()
{
    Locating();
    if(ingame) {RequestTransition(DroneState_ZHENCHA);}    
    else {RequestTransition(DroneState_WAITING);}    
}
```

**执行流程**：
1. 调用 `Locating()` —— 执行完整的瞄准投放流程
2. 根据模式决定下一个状态：
   - 比赛模式：→ 侦察（ZHENCHA）
   - 练习模式：→ 等待（WAITING）

**为什么比赛模式是先瞄准再侦察**：
- 比赛流程：起飞 → 瞄准投放（MIAOZHUN）→ 侦察（ZHENCHA）→ 返航（RETURN）→ 降落（LAND）
- 这样设计是因为投放区在 32.5m 处，侦察区在 57.5m 处，先经过投放区

---

## 第373-378行：setgamemode

```cpp
void Drone::setgamemode(bool InGame)
{
    if(InGame) ROS_WARN_STREAM("\033[32m" << "[Drone] 比赛模式启动,状态机将自动跳转,请留意各坐标位置！！！" << "\033[0m");
    else ROS_INFO_STREAM("\033[32m" << "[Drone] 测试模式启动,请使用上位机控制飞机状态..." << "\033[0m");
    ingame = InGame;
}
```

**为什么比赛模式用 `ROS_WARN` 而练习模式用 `ROS_INFO`**：
- 比赛模式是自动运行，不出意外的话整个流程会自动执行
- 用 `WARN`（黄色）强调这是一个重要提示，提醒操作员注意安全
- 练习模式是手动控制，用 `INFO`（白色）正常提示

**命名风格**：
- 方法名 `setgamemode` 使用驼峰但不规范（应该是 `setGameMode`）
- 参数名 `InGame` 首字母大写（应该是 `inGame`）
- 成员变量 `ingame` 全小写
- 代码风格不统一，但不影响功能

---

## 第380-388行：LiDAR 数据接口

```cpp
void Drone::update_Lidar(int lidar_data)
{
    lidar_height = (int)lidar_data*100;  //cm
}

int Drone::get_Lidar()
{
    return lidar_height;
}
```

**update_Lidar 分析**：
- 输入参数是 `int` 类型，单位是**米**（从 `Range.range` 获取）
- `*100` 转换为**厘米**
- 存储到 `lidar_height` 成员变量

**单位转换的原因**：
- 标定数据 `calibration_data.txt` 中的高度单位是 cm
- `HeightCalibration::query()` 接受的高度参数单位是 cm
- 所以需要把米的输入转换为厘米存储

**get_Lidar 分析**：
- 返回 `int` 类型的厘米值
- 可以被 `Locating()` 或其他需要高度的函数调用

**潜在问题**：
- `update_Lidar` 目前**没有被任何回调函数调用**
- `communication.cpp` 中的 `lidar_cb` 只是打印了日志，没有调用 `drone.update_Lidar()`
- 所以 `lidar_height` 始终为默认值 0

---

## 总结：Drone.cpp 的完整调用链

### 比赛模式完整流程

```
NONE ──[自动]──→ TAKEOFF ──[自动]──→ MIAOZHUN ──[自动]──→ ZHENCHA ──[自动]──→ RETURN ──[自动]──→ LAND ──[自动]──→ NONE
  │              │              │              │              │              │              │
  │ 什么都不做    │ TakeOff(5)   │ Locating()   │ Detect()     │ GoHome()     │ Land()       │ 等待下次
  │              │              │              │              │              │              │
  │              │ 解锁+起飞     │ 识别+瞄准+投放 │ 航线扫描     │ 高速返航     │ 自动降落     │
```

### Web 远程控制流程（练习模式）

```
Web 客户端 → HTTP GET → WebServer (8880端口)
  ├─ /setstat?newstate=N  → Drone::UpdateState()  → RequestTransition()
  ├─ /setgoal?mode=0&...  → Drone::UpdateGoal()   → Goal 变量更新
  └─ /throw?id=N          → Drone::Throw()        → ThrowBottle() 服务调用
                              ↓
                        主循环 40Hz → HandleState() → Execute*()
```

### 与外部模块的关系

| 外部模块 | 调用位置 | 作用 |
|---------|---------|------|
| `control.cpp` | `ExecuteTakeOff/Goal/Return/Land/Zhencha` | 飞控控制指令 |
| `aim.cpp` | `ExecuteMiaozhun` | 瞄准投放逻辑 |
| `WebServer.cpp` | 通过 HTTP 路由回调 | 远程控制入口 |
| `communication.cpp` | 间接依赖（全局变量） | 获取飞控数据 |