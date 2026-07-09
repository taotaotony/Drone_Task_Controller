# aim.cpp 逐行注释详解

---

## 文件概述

`aim.cpp` 是**瞄准模块的核心实现**，包含了视觉伺服瞄准的完整逻辑。这是整个无人机控制系统中**最关键、最复杂的模块**，负责：

1. **鲁棒 K-means 聚类**：从视觉检测的多个噪点中精准定位三个桶的位置
2. **PID 视觉伺服闭环**：根据像素误差计算速度指令，驱动无人机对准桶口
3. **分级下降瞄准策略**：高空粗调 → 中空过渡 → 低空精调 → 投放
4. **容错机制**：丢失目标时重试、超时强制投放

---

## 第1行：文件注释

```cpp
// aim.cpp — 瞄准、识别与投放逻辑实现
```

---

## 第2-4行：包含头文件

```cpp
#include "aim.h"
#include "communication.h"
#include "calibration.h"
```

**为什么需要这些头文件**：
- `aim.h`：自身的头文件，包含 BarrelPosition 结构体和函数声明
- `communication.h`：提供 `VisualData` 全局变量（视觉检测数据）、`PX4_Position`、`PX4_Velocity` 等
- `calibration.h`：提供 `HeightCalibration` 标定类

---

## 第5行：时间守卫定义

```cpp
ros::Time time_guard;
```

**作用**：全局变量定义，记录瞄准开始的时间戳。对应于 `aim.h` 中的 `extern ros::Time time_guard;` 声明。

**使用位置**：在 `Locating()` 的第一行 `time_guard=ros::Time::now();` 赋值。

---

## 第7行：标定对象 extern 声明

```cpp
extern HeightCalibration calib;   // 标定参数读取器
```

**作用**：声明 `calib` 对象是在其他地方（`main.cpp`）定义的。

**为什么需要它**：
- `main.cpp` 中创建了 `HeightCalibration calib;` 并加载了标定数据
- `aim.cpp` 中的 `Locating()` 未来可以使用 `calib.query()` 根据当前高度获取瞄准中心
- **注意**：当前代码中**并没有使用** `calib`，这是一个预留的接口

---

## 第9-13行：欧氏距离函数

```cpp
// ── 两点间欧氏距离 ────────────────────────────
double distance(const BarrelPosition& a, const BarrelPosition& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}
```

**`std::hypot` 的作用**：
- 计算 `sqrt(x² + y²)`，即直角三角形的斜边长度
- 与手动 `sqrt(dx*dx + dy*dy)` 相比，`hypot` 在处理超大或超小数值时更精确（防止溢出或下溢）

**为什么用 `const` 引用**：
- 避免拷贝 `BarrelPosition` 对象的开销
- `const` 保证函数不会修改传入的对象

---

## 第15-84行：鲁棒 K-means 聚类

```cpp
// ── 鲁棒 K-means 聚类（单次运行） ─────────────
std::vector<BarrelPosition> robustKMeans(const std::vector<BarrelPosition>& data,
                                          int k, int maxIter)
{
```

### 第19行：空数据保护

```cpp
if (data.empty()) return {};
```

- 如果输入数据为空，直接返回空向量
- `{}` 是 C++11 的初始化列表语法，返回一个空的 `std::vector<BarrelPosition>`

### 第21-23行：随机数生成器

```cpp
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<size_t> dis(0, data.size() - 1);
```

**各组件作用**：

| 组件 | 类型 | 作用 |
|------|------|------|
| `rd` | `std::random_device` | 真随机数种子（基于硬件熵源） |
| `gen` | `std::mt19937` | 梅森旋转算法（Mersenne Twister）伪随机数生成器 |
| `dis` | `std::uniform_int_distribution<size_t>` | 均匀整数分布，范围 [0, data.size()-1] |

**为什么用 `std::mt19937` 而不是 `rand()`**：
- `rand()` 的随机性质量差，周期短（通常只有 2^31）
- `mt19937` 周期长达 2^19937，随机性质量高
- C++11 标准推荐使用 `<random>` 库

### 第25-28行：随机初始化聚类中心

```cpp
std::vector<BarrelPosition> centers;
for (int i = 0; i < k; ++i) {
    centers.push_back(data[dis(gen)]);
}
```

- 从数据点中**随机选择 k 个点**作为初始聚类中心
- 这是标准 K-means 的初始化方法（Forgy 方法）
- 随机初始化的缺点是可能收敛到局部最优解，所以 `Locating()` 中运行 50 次取最优

### 第33行：主迭代循环

```cpp
while (iter++ < maxIter) {
```

- 使用后置自增 `iter++`：先比较再自增，`iter` 从 0 开始
- 最多迭代 `maxIter`（100）次

### 第34-46行：分配步骤（Assignment Step）

```cpp
std::vector<std::vector<BarrelPosition>> clusters(k);
for (const auto& p : data) {
    double minDist = std::numeric_limits<double>::max();
    int clusterIdx = 0;
    for (int i = 0; i < k; ++i) {
        double d = distance(p, centers[i]);
        if (d < minDist) {
            minDist = d;
            clusterIdx = i;
        }
    }
    clusters[clusterIdx].push_back(p);
}
```

**K-means 的 E 步（Expectation）**：
- 对每个数据点，计算它到所有聚类中心的距离
- 将它分配到**最近**的聚类中心所在的簇
- `std::numeric_limits<double>::max()` 返回 double 能表示的最大值，用作初始比较值

### 第48-82行：更新步骤（Update Step）——鲁棒版本

```cpp
bool converged = true;
for (int i = 0; i < k; ++i) {
    if (clusters[i].empty()) continue;

    // 计算簇内所有点到中心的距离
    std::vector<double> dists;
    for (const auto& p : clusters[i]) {
        dists.push_back(distance(p, centers[i]));
    }

    // 排序，取 90% 分位点作为阈值
    std::sort(dists.begin(), dists.end());
    double threshold = dists[static_cast<size_t>(dists.size() * 0.9)];

    // 只保留内点（距离 ≤ 阈值）
    std::vector<BarrelPosition> inliers;
    for (const auto& p : clusters[i]) {
        if (distance(p, centers[i]) <= threshold) {
            inliers.push_back(p);
        }
    }

    // 用内点计算新中心
    double sumX = 0, sumY = 0;
    for (const auto& p : inliers) {
        sumX += p.x;
        sumY += p.y;
    }
    newCenters[i] = BarrelPosition(sumX / inliers.size(), sumY / inliers.size());

    // 检查是否收敛（中心位移 < 1e-5）
    if (distance(newCenters[i], centers[i]) > 1e-5) {
        converged = false;
    }
}

centers = newCenters;
if (converged) break;
```

**鲁棒 K-means 与标准 K-means 的区别**：

| 步骤 | 标准 K-means | 鲁棒 K-means |
|------|-------------|-------------|
| 计算新中心 | 使用簇内**所有点**的均值 | 只使用 **90% 内点**的均值 |
| 抗噪能力 | 差（离群点严重影响中心位置） | 强（自动排除离群点） |

**为什么使用 90% 阈值**：
- 假设视觉检测的误检率约为 10%
- 去除最远的 10% 点可以有效排除误检
- 但又保留了足够多的有效点来计算准确的中心位置

**收敛条件**：所有聚类中心的位移都小于 `1e-5` 像素

---

## 第86-176行：PID 瞄准核心函数

```cpp
// ── 瞄准定位函数 ──────────────────────────────
//瞄准函数
bool Positioning(double pZ,int tim,int CenterX,int CenterY,double MaxVel,int BottleLabel)
{
```

### 第90行：参数日志

```cpp
ROS_INFO("InPositioning Height=%.2f   Time=%d  Bottle=%d",pZ,tim,BottleLabel);
```

- 打印进入瞄准时的参数：高度、最大时间、投放编号
- 使用 `%.2f` 保留两位小数显示高度

### 第91行：设置 PID 频率

```cpp
ros::Rate pid_rate(30);
```

- 瞄准循环以 **30Hz** 运行
- 比主循环的 40Hz 稍慢，因为视觉数据本身就是 30Hz
- 每次循环间隔约 33ms

### 第92-93行：丢失目标计数器

```cpp
int noPos=-1;  //检测瞄准时是否出现瞄歪了导致看不到桶的情况
int PosinnoPos=0;  //检测瞄准时出现瞄歪了导致看不到桶的情况下，又看到桶的情况
```

**设计意图**：
- `noPos`：连续看不到目标的帧数计数器。初始为 -1 表示"还未丢失过目标"
- `PosinnoPos`：重新看到目标后的确认计数器。防止视觉误识别导致错误地重置 `noPos`

**逻辑链**：
```
丢失目标 → noPos 从 0 开始累加
  → 如果看到目标但 noPos != -1（之前丢失过）：
    → PosinnoPos 累加
    → PosinnoPos >= 10（连续10帧都看到）→ 确认目标恢复，重置 noPos = -1
  → 如果 noPos >= 150（连续5秒看不到）→ 返回 false
```

### 第94行：主循环

```cpp
while(tim&&ros::ok())
```

- `tim`：剩余循环次数，每次减 1，到 0 退出
- `ros::ok()`：ROS 正常运行中
- 两者任一为 false 就退出

### 第96行：检查视觉目标是否存在

```cpp
if(VisualData.Target1_Exist||VisualData.Target2_Exist||VisualData.Target3_Exist)
```

- 只要三个桶中任意一个被检测到，就进入瞄准逻辑
- `VisualData` 是由 `visual_cb` 回调函数从 ROS 话题 "IR" 更新的全局变量

### 第107-128行：选择最近的桶

```cpp
float Barrel1X=(VisualData.Target1_LU_x+VisualData.Target1_RD_x)/2;
float Barrel1Y=(VisualData.Target1_LU_y+VisualData.Target1_RD_y)/2;
// ... 类似计算 Barrel2 和 Barrel3 ...

float DcenterB1=(Barrel1X-CenterX)*(Barrel1X-CenterX)+(Barrel1Y-CenterY)*(Barrel1Y-CenterY);
// ... 类似计算 DcenterB2 和 DcenterB3 ...

float BarrelX=Barrel1X;
float BarrelY=Barrel1Y;
if(DcenterB2<DcenterB1 && VisualData.Target2_Exist)
{
    BarrelX=Barrel2X;
    BarrelY=Barrel2Y;
}
if(DcenterB3<DcenterB2 && DcenterB3<DcenterB1 && VisualData.Target3_Exist)
{
    BarrelX=Barrel3X;
    BarrelY=Barrel3Y;
}
```

**逻辑分析**：
1. 计算每个桶的中心像素坐标：`(LU + RD) / 2`
2. 计算每个桶到瞄准中心的**距离平方**（避免开平方运算，提高性能）
3. 选择距离瞄准中心**最近**的桶作为瞄准目标
4. 默认选中桶1，如果桶2更近则选桶2，如果桶3最近则选桶3

**为什么用距离平方而不是距离**：
- 避免 `sqrt()` 计算，提高性能
- 比较距离平方和比较距离的结果完全相同（平方函数是单调递增的）

### 第129-137行：对准判定与提前投放

```cpp
if((abs(BarrelX-CenterX)*abs(BarrelX-CenterX)+abs(BarrelY-CenterY)*abs(BarrelY-CenterY)<=MinPx*MinPx)&&(BottleLabel!=0))
{
    SetVel(0,0,pZ);
    ThrowBottle(BottleLabel);
    ROS_INFO("提前投放 指令%d",BottleLabel);
    ros::spinOnce();
    ros::Duration(1).sleep();
    return true;
}
```

**条件**：
- 桶中心到瞄准中心的距离 ≤ `MinPx`（15像素）
- `BottleLabel != 0`（不是瞄准模式，是投放模式）

**动作**：
1. `SetVel(0,0,pZ)` —— 悬停（速度归零）
2. `ThrowBottle(BottleLabel)` —— 发送投放指令
3. 等待 1 秒确保投放完成
4. 返回 `true`

**为什么叫"提前投放"**：
- 正常的投放是在循环结束后（超时）才执行
- 如果在循环过程中提前对准了，就立即投放，不需要等到超时

### 第138-148行：PID 速度计算

```cpp
float out_vel_x=-PID_Calculate(&pos_pid_xy,CenterX,BarrelX);
ROS_WARN("未限制输出Vx:%.5f",out_vel_x);
if(abs(out_vel_x)<DeadZone) out_vel_x=0;
if(out_vel_x>MaxVel) out_vel_x=MaxVel;
if(out_vel_x<-MaxVel) out_vel_x=-MaxVel;
float out_vel_y=PID_Calculate(&pos_pid_xy,CenterY,BarrelY);
// ... 同样的限幅处理 ...
SetVel(out_vel_x,out_vel_y,pZ);
```

**PID 控制器的输入输出**：
- 输入：`target=CenterX/CenterY`（瞄准中心），`current=BarrelX/BarrelY`（桶中心）
- 输出：速度指令 `out_vel_x/out_vel_y`

**为什么 X 方向取反**：`-PID_Calculate(...)`
- 图像坐标系中 X 向右为正
- 机体坐标系中 X 向右为正
- 但 PID 的误差计算是 `target - current`，如果桶在瞄准中心右侧（BarrelX > CenterX），误差为正，输出正速度
- 正速度意味着飞机向右移动，这会使桶在图像中向左移动
- 所以需要取反：当桶在右边时，飞机向左移，使桶回到中心

**三阶段限幅**：
1. **死区**：输出 < 0.01 时置 0，防止微小的抖动
2. **上限**：输出 > MaxVel 时截断到 MaxVel
3. **下限**：输出 < -MaxVel 时截断到 -MaxVel

### 第154-162行：目标丢失处理

```cpp
else 
{
    noPos++;
    if(noPos>=150)
    {
        ros::spinOnce();
        return false;
    }
}
```

- 连续 150 帧（约 5 秒）看不到任何目标
- 返回 `false` 通知调用者瞄准失败
- 调用者（`Locating()`）会尝试重新瞄准

### 第167-175行：超时强制投放

```cpp
if(BottleLabel!=0)
{
    SetVel(0,0,pZ);
    ThrowBottle(BottleLabel);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    return true;
}
return true;
```

- 循环结束（超时）但还没对准
- 如果 `BottleLabel != 0`（是投放模式），强制投放
- 如果 `BottleLabel == 0`（只是瞄准模式），静默返回

---

## 第177-329行：定位与投放主流程

```cpp
// ── 定位与投放主流程 ──────────────────────────
void Locating()
{
```

### 第180-185行：飞往投放区

```cpp
time_guard=ros::Time::now();
ROS_INFO("飞往投放区");
SetVel(0,1.8,InitialHeight);
ShowPosition(17);
SetPoint(0,TakeofftoThrow,InitialHeight);
ShowPosition(5);
```

**飞行路径**：
1. `SetVel(0, 1.8, 3.5)` —— 以 1.8m/s 的速度向前飞行
2. `ShowPosition(17)` —— 飞行 17 秒（约 30 米）
3. `SetPoint(0, 32.5, 3.5)` —— 精确到达投放区
4. `ShowPosition(5)` —— 悬停 5 秒确认位置

### 第188-207行：收集检测数据

```cpp
int num=2;
ROS_INFO(">>>>>>>开始识别桶<<<<<<<");
// ... 变量声明 ...
std::vector<BarrelPosition> allPositions;
for(int t=100;t>0;t--)
{
    Barrel1X=(VisualData.Target1_LU_x+VisualData.Target1_RD_x)/2;
    // ... 计算三个桶的中心 ...
    
    if(Barrel1X != 0 && Barrel1Y != 0){allPositions.push_back({Barrel1X,Barrel1Y});}
    if(Barrel2X != 0 && Barrel2Y != 0){allPositions.push_back({Barrel2X,Barrel2Y});}
    if(Barrel3X != 0 && Barrel3Y != 0){allPositions.push_back({Barrel3X,Barrel3Y});} 
    ros::Duration(0.1).sleep();
    ros::spinOnce();
}
```

- 循环 100 次，每次间隔 0.1 秒（共 10 秒）
- 每帧收集所有检测到的桶中心坐标
- 只有当 `X != 0 && Y != 0` 时才加入（排除无效检测）

### 第208-238行：空检测结果处理

```cpp
if(allPositions.size()==0)
{
    SetPoint(0,TakeofftoThrow,InitialHeight+1);  // 升高 1 米
    for(int t=100;t>0;t--)
    {
        // 重新收集数据
    }
    if(allPositions.size()==0)
    {
        ROS_ERROR("很遗憾，失败了，放弃任务返航！");
        ThrowBottle(1);
        ThrowBottle(3);
        // ... 返航降落 ...
    }
}
```

**容错机制**：
- 如果第一次没检测到桶，升高 1 米再试一次
- 如果还是检测不到，放弃任务：
  - 投掉桶1的瓶子（`ThrowBottle(1)`）
  - 关闭舱门（`ThrowBottle(3)`）
  - 返航到起飞点
  - 降落
  - `ros::Duration(1000).sleep()` —— 休眠 1000 秒（约 17 分钟），等待救援

### 第240-262行：多次聚类取最优

```cpp
const int RUNS = 50;
std::vector<BarrelPosition> bestCenters;
double bestWCSS = std::numeric_limits<double>::max();

for (int run = 0; run < RUNS; ++run) {
    auto centers = robustKMeans(allPositions, 3, 100);
    
    double wcss = 0;
    for (const auto& p : allPositions) {
        double minDist = std::numeric_limits<double>::max();
        for (const auto& c : centers) {
            minDist = std::min(minDist, distance(p, c));
        }
        wcss += minDist * minDist;
    }
    
    if (wcss < bestWCSS) {
        bestWCSS = wcss;
        bestCenters = centers;
    }
}
```

**为什么运行 50 次**：
- K-means 的初始中心是随机选择的
- 不同的初始中心可能收敛到不同的局部最优解
- 运行多次并选择 WCSS（类内平方和）最小的结果，提高可靠性

**WCSS 计算公式**：
```
WCSS = Σ min_distance(p, centers)²
       p∈data
```

### 第264-277行：像素坐标转世界坐标

```cpp
// 按x坐标排序输出
std::sort(bestCenters.begin(), bestCenters.end(), 
    [](const BarrelPosition& a, const BarrelPosition& b) { return a.x < b.x; });

Barrel1X_Coord_LD = bestCenters[0].x / 430.857 - 2.228;
Barrel1Y_Coord_LD = (1.393 - bestCenters[0].y / 430.857) + TakeofftoThrow;
```

**按 X 坐标排序**：
- 假设三个桶在图像中从左到右排列
- 排序后 `bestCenters[0]` 是最左边的桶，`[1]` 是中间，`[2]` 是最右边

**像素→世界坐标转换公式**（硬编码标定参数）：
```
world_x = pixel_x / 430.857 - 2.228
world_y = (1.393 - pixel_y / 430.857) + TakeofftoThrow
```

**参数含义**：
- `430.857`：像素到米的比例因子（pixels per meter）
- `2.228` 和 `1.393`：相机安装偏移量

**为什么是硬编码**：
- 虽然项目中有 `HeightCalibration` 类，但这里并没有使用
- 这些值是针对特定高度（3.5m）标定得到的固定参数
- 如果高度变化，这些参数需要重新标定

### 第284-328行：逐桶瞄准投放

**对桶1的完整流程**：

```cpp
// 1. 飞到桶1上方 3.5m
SetPoint(Barrel1X_Coord_LD,Barrel1Y_Coord_LD,InitialHeight);
ShowPosition(3);

// 2. 高空瞄准（3.5m，90帧≈3秒，Kp=0.01，瞄准图像中心）
PID_Init(&pos_pid_xy,Kp_H,Ki_H,Kd_H);
Positioning(InitialHeight,90,CamX/2,CamY/2,MaxVel_H,0);

// 3. 缓慢下降到 2.1m
SlowDescend(PX4_Position.x,PX4_Position.y,InitialHeight,ThrowMidHeight);

// 4. 中空瞄准（2.1m，90帧≈3秒，Kp=0.01，瞄准图像中心）
Positioning(ThrowMidHeight,90,CamX/2,CamY/2,MaxVel_H,0);

// 5. 缓慢下降到 0.8m
SlowDescend(PX4_Position.x,PX4_Position.y,ThrowMidHeight,ThrowHeight);

// 6. 低空瞄准（0.8m，300帧≈10秒，Kp=0.001，瞄准桶口位置）
PID_Init(&pos_pid_xy,Kp_L,Ki_L,Kd_L);
if(Positioning(ThrowHeight,300,Throw1_X,Throw1_Y,MaxVel_L,1)==false)
{
    // 7. 失败重试：回到中空重新瞄准
    PID_Init(&pos_pid_xy,Kp_H,Ki_H,Kd_H);
    SetPoint(Barrel1X_Coord_LD,Barrel1Y_Coord_LD,ThrowMidHeight);
    Positioning(ThrowMidHeight,90,CamX/2,CamY/2,MaxVel_H,0);
    SlowDescend(PX4_Position.x,PX4_Position.y,ThrowMidHeight,ThrowHeight);
    PID_Init(&pos_pid_xy,Kp_L,Ki_L,Kd_L);
    if(Positioning(ThrowHeight,300,Throw1_X,Throw1_Y,MaxVel_L,1)==false)
    {
        // 8. 二次失败：强制投放
        ThrowBottle(1);
        ROS_INFO("实在没瞄准，放弃本次投放，进行下一次投放");
    }
}
```

**对桶2的流程**（第308-328行）类似，区别：
- 使用 `Throw2_X/Y`（660, 515）作为瞄准中心
- 投放指令为 3（对应桶2）
- 中空瞄准只有 30 帧（1 秒）

**分级下降策略总结**：
```
3.5m ─[高空瞄准 90帧]─→ 2.1m ─[中空瞄准 90帧]─→ 0.8m ─[低空瞄准 300帧]─→ 投放
  ↑                        ↑                        ↑
Kp=0.01                 Kp=0.01                 Kp=0.001
MaxVel=0.15             MaxVel=0.15             MaxVel=0.05
瞄准图像中心            瞄准图像中心            瞄准桶口位置
```

---

## 总结：瞄准模块的完整数据流

```
Locating() 被 Drone::ExecuteMiaozhun() 调用
  │
  ├─ 阶段1: 飞往投放区 (32.5m)
  │
  ├─ 阶段2: 识别桶位置
  │   ├─ 收集 100 帧检测数据 (10秒)
  │   ├─ 鲁棒 K-means 聚类 (50次取最优)
  │   └─ 像素坐标 → 世界坐标
  │
  └─ 阶段3: 逐桶瞄准投放
      ├─ 桶1: 高空→中空→低空→投放 (带重试)
      ├─ 等待 1 秒
      └─ 桶2: 高空→中空→低空→投放 (带重试)