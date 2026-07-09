# cvisual.cpp 逐行注释详解

---

## 文件概述

`cvisual.cpp` 是一个**独立的视觉检测 ROS 节点**，是整个系统中**最复杂、技术含量最高的模块**。它负责：

1. **实时目标检测**：使用 YOLOv8 模型 + TensorRT 加速，检测图像中的桶目标
2. **CUDA 加速**：GPU 预处理（BGR→RGB、缩放、归一化、通道分离）
3. **多线程流水线**：取帧线程 + 推理线程 + 主线程（CPU 后处理），实现 GPU/CPU 并行
4. **SORT 多目标跟踪**：Kalman 滤波器 + 匈牙利匹配，稳定跟踪检测到的桶
5. **改进 NMS**：针对圆形桶的 IOU + 中心距离联合去重
6. **ROS 通信**：将检测结果发布到话题 "IR"，供主节点瞄准模块使用

---

## 第1-25行：包含头文件

```cpp
#include <ros/ros.h>
#include <ros/package.h>
#include <px4_controller/tbag.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include "calibration.h"
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>
#include <future>
```

**头文件分组说明**：

| 组 | 头文件 | 用途 |
|----|--------|------|
| ROS | `<ros/ros.h>`、`<ros/package.h>` | ROS 节点初始化、包路径获取 |
| 自定义消息 | `<px4_controller/tbag.h>` | 视觉检测结果消息类型 |
| OpenCV | `<opencv2/opencv.hpp>` | 图像处理基础功能 |
| CUDA OpenCV | `<opencv2/cudawarping.hpp>` 等 | GPU 加速的图像缩放、颜色转换、通道分离 |
| 标定 | `"calibration.h"` | 高度标定查询（用于显示标定点） |
| TensorRT | `<NvInfer.h>`、`<NvInferRuntime.h>` | NVIDIA 推理引擎 |
| CUDA | `<cuda_runtime_api.h>` | CUDA 运行时 API（内存管理、流操作） |
| C++ 标准库 | `<thread>`、`<mutex>`、`<atomic>`、`<deque>`、`<future>` 等 | 多线程、同步、异步任务 |

---

## 第28-30行：全局变量

```cpp
// 标定参数读取器
HeightCalibration calib;
std::atomic<int> current_lidar{0};  // [修复] 原子化，避免ROS回调与主线程的数据竞争
ros::Subscriber lidar_sub; // 订阅激光雷达数据
```

**`HeightCalibration calib`**：
- 与 `main.cpp` 中的 `calib` 同名但**不同作用域**（这里是 `cvisual.cpp` 的全局变量）
- 用于在调试窗口中绘制标定落点

**`std::atomic<int> current_lidar{0}`**：
- 原子类型，保证多线程安全
- 存储 LiDAR 高度数据（厘米）
- 注释强调"原子化，避免ROS回调与主线程的数据竞争"
- 因为 `lidar_cb` 在 ROS 回调线程中执行，而主线程在显示循环中读取

**`ros::Subscriber lidar_sub`**：
- 订阅 LiDAR 话题
- 注意：这个节点**也订阅了 LiDAR**，与 `main` 节点重复订阅同一个话题

---

## 第33-38行：TensorRT Logger

```cpp
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            ROS_WARN_STREAM("[TensorRT] " << msg);
    }
} gLogger;
```

**作用**：实现 TensorRT 的日志接口，将 TensorRT 的日志重定向到 ROS 日志系统。

**`Severity::kWARNING`**：只显示 WARNING 及以上级别的日志，忽略 INFO 级别的详细日志。

**`noexcept`**：C++11 关键字，表示这个函数不会抛出异常。

**`gLogger`**：全局 Logger 实例，在创建 TensorRT runtime 时传入。

---

## 第41-51行：模型参数

```cpp
const int INPUT_H = 640;
const int INPUT_W = 640;
const int NUM_ANCHORS = 8400;
const int NUM_CLASSES = 80;
const float CONF_THRESH = 0.4f;
const float IOU_THRESH = 0.45f;
const float CENTER_DIST_RATIO = 2.0f;
const int   NEW_W = 1200;
const int   NEW_H = 750;
```

| 参数 | 值 | 含义 |
|------|-----|------|
| `INPUT_H/W` | 640 | YOLOv8 模型输入尺寸 |
| `NUM_ANCHORS` | 8400 | YOLOv8 输出锚点数量（640/8=80, 80²=6400 + 40²=1600 + 20²=400 = 8400） |
| `NUM_CLASSES` | 80 | COCO 数据集类别数（虽然只检测桶，但模型是在 COCO 上训练的） |
| `CONF_THRESH` | 0.4 | 置信度阈值，低于此值的检测结果被过滤 |
| `IOU_THRESH` | 0.45 | NMS 的 IOU 阈值 |
| `CENTER_DIST_RATIO` | 2.0 | 中心距离合并阈值（相对于较小框宽度） |
| `NEW_W/H` | 1200x750 | 调试窗口的显示尺寸 |

**为什么 NUM_CLASSES=80 但只检测桶**：
- 模型是在 COCO 数据集上预训练的 YOLOv8n
- 虽然能检测 80 类物体，但实际只关注桶（可能属于"bottle"或"cup"类别）
- 或者模型经过了微调（fine-tune），但输出格式仍然是 80 类

---

## 第54-59行：跟踪参数

```cpp
const int   TRACK_CONFIRM_FRAMES = 3;
const int   TRACK_MAX_LOST      = 10;
const float TRACK_IOU_MATCH     = 0.3f;
const float KF_PROCESS_NOISE    = 0.01f;
const float KF_MEASURE_NOISE    = 0.1f;
const float CONF_HYSTERESIS_LOW = 0.25f;
```

| 参数 | 值 | 含义 |
|------|-----|------|
| `TRACK_CONFIRM_FRAMES` | 3 | 新跟踪目标需要连续 3 帧匹配才能确认 |
| `TRACK_MAX_LOST` | 10 | 跟踪目标丢失 10 帧后删除 |
| `TRACK_IOU_MATCH` | 0.3 | 跟踪匹配的 IOU 阈值 |
| `KF_PROCESS_NOISE` | 0.01 | Kalman 滤波器过程噪声协方差 |
| `KF_MEASURE_NOISE` | 0.1 | Kalman 滤波器测量噪声协方差 |
| `CONF_HYSTERESIS_LOW` | 0.25 | 置信度滞后阈值（当前未使用） |

---

## 第62-68行：检测数据结构

```cpp
struct Detection {
    float x1, y1, x2, y2, conf;
    float cx() const { return (x1 + x2) * 0.5f; }
    float cy() const { return (y1 + y2) * 0.5f; }
    float area() const { return (x2 - x1) * (y2 - y1); }
    float width() const { return x2 - x1; }
};
```

**字段**：
- `x1, y1`：检测框左上角像素坐标
- `x2, y2`：检测框右下角像素坐标
- `conf`：检测置信度

**内联方法**：
- `cx()` / `cy()`：计算中心坐标
- `area()`：计算框面积
- `width()`：计算框宽度

**为什么用 `const`**：这些方法不修改对象状态，可以在 const 对象上调用。

---

## 第73-108行：线程安全队列

```cpp
template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_size = 3) : max_size_(max_size) {}

    void push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            queue_.pop_front(); // 丢弃最旧帧，保持低延迟
        }
        queue_.push_back(std::move(item));
    }

    bool try_pop(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    std::deque<T> queue_;
    mutable std::mutex mutex_;
    size_t max_size_;
};
```

**设计模式**：生产者-消费者模式

**为什么需要线程安全队列**：
- 推理线程（生产者）将推理结果推入队列
- 主线程（消费者）从队列取出结果进行后处理
- 两个线程同时访问队列，必须加锁

**`std::lock_guard`**：RAII 风格的互斥锁，构造时加锁，析构时自动解锁。

**`std::move`**：移动语义，避免拷贝，提高性能。

**`mutable std::mutex`**：
- `mutable` 允许在 `const` 方法中修改 `mutex_`
- 因为 `empty()` 是 `const` 方法，但需要加锁

**最大队列大小**：
- 默认 3，实际使用时传入 1
- 超过最大大小时丢弃最旧帧，保持低延迟

---

## 第112-117行：推理结果结构

```cpp
struct InferenceResult {
    std::vector<float> output_data;  // TensorRT 原始输出 (84*8400 floats)
    int img_width;                   // 原始图像宽度
    int img_height;                  // 原始图像高度
    double timestamp;                // 推理完成时间戳
};
```

- `output_data`：TensorRT 推理的原始输出，大小为 `84 * 8400 = 705600` 个 float
  - 每个锚点 84 个值：4（bbox）+ 80（class scores）
  - 共 8400 个锚点
- `img_width/height`：原始图像尺寸，用于坐标缩放
- `timestamp`：推理完成时间，用于 FPS 计算

---

## 第121-153行：同一目标判定函数

```cpp
inline bool is_same_target(const Detection& a, const Detection& b,
                           float iou_thres, float center_dist_ratio) {
    // 1. 计算 IOU
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);
    float iw = std::max(0.0f, xx2 - xx1);
    float ih = std::max(0.0f, yy2 - yy1);
    float inter = iw * ih;
    float area_a = a.area();
    float area_b = b.area();
    float iou = inter / (area_a + area_b - inter + 1e-6f);

    if (iou > iou_thres) return true;

    // 2. 中心包含：如果 b 的中心落在 a 内部
    float bcx = b.cx(), bcy = b.cy();
    if (bcx >= a.x1 && bcx <= a.x2 && bcy >= a.y1 && bcy <= a.y2) return true;

    // 3. 反向：如果 a 的中心落在 b 内部
    float acx = a.cx(), acy = a.cy();
    if (acx >= b.x1 && acx <= b.x2 && acy >= b.y1 && acy <= b.y2) return true;

    // 4. 中心距离：两框中心距离 < 较小框宽度 * ratio
    float dx = acx - bcx;
    float dy = acy - bcy;
    float dist = std::sqrt(dx * dx + dy * dy);
    float min_w = std::min(a.width(), b.width());
    if (dist < min_w * center_dist_ratio) return true;

    return false;
}
```

**为什么需要 4 种判定条件**：

| 条件 | 解决的问题 |
|------|-----------|
| IOU > 阈值 | 标准 NMS，框重叠很大时合并 |
| 中心包含 | 一个框完全在另一个框内部（常见于桶的重复检测） |
| 反向中心包含 | 同上，但方向相反 |
| 中心距离 < 宽度×ratio | 两个框不重叠但很近（桶被光照分割成两个框） |

**`1e-6f` 的作用**：防止除零错误。

**为什么是 `inline`**：这个函数在 NMS 循环中被频繁调用，内联可以消除函数调用开销。

---

## 第156-259行：Kalman 跟踪器

```cpp
class KalmanTracker {
public:
    enum State { kUnconfirmed, kConfirmed, kLost };

    KalmanTracker(const Detection& det, int track_id)
        : id_(track_id), state_(kUnconfirmed), lost_frames_(0),
          confirm_count_(1), total_frames_(1)
    {
        kf_.init(8, 4, 0, CV_32F);
        kf_.transitionMatrix = (cv::Mat_<float>(8, 8) <<
            1,0,0,0,1,0,0,0,
            0,1,0,0,0,1,0,0,
            0,0,1,0,0,0,1,0,
            0,0,0,1,0,0,0,1,
            0,0,0,0,1,0,0,0,
            0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,1);
        kf_.measurementMatrix = (cv::Mat_<float>(4, 8) <<
            1,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,
            0,0,0,1,0,0,0,0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(KF_PROCESS_NOISE));
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(KF_MEASURE_NOISE));
        cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

        kf_.statePost.at<float>(0) = det.cx();
        kf_.statePost.at<float>(1) = det.cy();
        kf_.statePost.at<float>(2) = det.width();
        kf_.statePost.at<float>(3) = det.y2 - det.y1;
        kf_.statePost.at<float>(4) = 0;
        kf_.statePost.at<float>(5) = 0;
        kf_.statePost.at<float>(6) = 0;
        kf_.statePost.at<float>(7) = 0;

        best_conf_ = det.conf;
        update_bbox();
    }
    // ... 方法 ...
};
```

### 状态枚举

```cpp
enum State { kUnconfirmed, kConfirmed, kLost };
```

- `kUnconfirmed`：新创建的跟踪，需要连续匹配 `TRACK_CONFIRM_FRAMES`（3）帧才能确认
- `kConfirmed`：已确认的跟踪，输出到最终结果
- `kLost`：丢失超过 `TRACK_MAX_LOST`（10）帧，将被删除

### Kalman 滤波器配置

**状态向量（8维）**：
```
[x, y, w, h, vx, vy, vw, vh]
```
- `x, y`：框中心坐标
- `w, h`：框宽度和高度
- `vx, vy, vw, vh`：对应的速度

**测量向量（4维）**：
```
[x, y, w, h]
```

**状态转移矩阵**（匀速运动模型）：
```
x' = x + vx
y' = y + vy
w' = w + vw
h' = h + vh
vx' = vx
...
```

**测量矩阵**：
只测量位置和大小，不测量速度。

### 关键方法

| 方法 | 作用 |
|------|------|
| `predict()` | 预测下一帧的位置（调用 `kf_.predict()`） |
| `update(det)` | 用检测结果更新滤波器（调用 `kf_.correct()`） |
| `mark_lost()` | 标记为丢失帧，超过阈值后状态变为 kLost |
| `smoothed_conf()` | 计算平滑置信度（原始置信度 + 帧数奖励） |

### 平滑置信度

```cpp
float smoothed_conf() const {
    float raw = best_conf_;
    float bonus = std::min(0.15f, total_frames_ * 0.002f);
    return std::min(1.0f, raw + bonus);
}
```

- 每跟踪一帧，置信度增加 0.002
- 最大奖励 0.15（75 帧后达到）
- 最终置信度不超过 1.0
- 这样长时间稳定跟踪的目标置信度会逐渐提高

---

## 第262-359行：SORT 多目标跟踪器

```cpp
class SORTTracker {
public:
    SORTTracker() : next_id_(0) {}

    std::vector<Detection> update(const std::vector<Detection>& detections, int img_w, int img_h) {
        // 1. 预测所有已有跟踪的下一帧位置
        for (auto& track : tracks_) {
            track.predict();
        }

        // 2. 构建 IOU 代价矩阵
        std::vector<std::vector<float>> iou_matrix(tracks_.size(),
            std::vector<float>(detections.size(), 0.0f));
        for (size_t t = 0; t < tracks_.size(); ++t) {
            Detection tbox = tracks_[t].get_bbox();
            for (size_t d = 0; d < detections.size(); ++d) {
                iou_matrix[t][d] = 1.0f - compute_iou(tbox, detections[d]);
            }
        }

        // 3. 贪心匹配
        std::vector<int> track_match(tracks_.size(), -1);
        std::vector<bool> det_matched(detections.size(), false);
        greedy_match(iou_matrix, track_match, det_matched, TRACK_IOU_MATCH);

        // 4. 更新匹配的跟踪
        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (track_match[t] >= 0) {
                tracks_[t].update(detections[track_match[t]]);
            } else {
                tracks_[t].mark_lost();
            }
        }

        // 5. 创建新跟踪（未匹配的检测）
        for (size_t d = 0; d < detections.size(); ++d) {
            if (!det_matched[d]) {
                tracks_.emplace_back(detections[d], next_id_++);
            }
        }

        // 6. 删除丢失的跟踪
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                [](const KalmanTracker& t) { return t.is_lost(); }),
            tracks_.end());

        // 7. 输出已确认的跟踪（最多3个）
        std::vector<Detection> output;
        for (auto& track : tracks_) {
            if (track.is_confirmed()) {
                Detection d = track.get_bbox();
                d.conf = track.smoothed_conf();
                output.push_back(d);
            }
        }

        std::sort(output.begin(), output.end(),
            [](const Detection& a, const Detection& b) { return a.conf > b.conf; });
        if (output.size() > 3) output.resize(3);

        return output;
    }
    // ...
};
```

**SORT 算法流程**：

```
输入：当前帧的检测结果
  │
  ├─ 1. 预测：所有已有跟踪预测下一帧位置
  ├─ 2. 匹配：计算 IOU 代价矩阵
  ├─ 3. 关联：贪心算法匹配跟踪和检测
  ├─ 4. 更新：匹配的跟踪用检测结果更新
  ├─ 5. 创建：未匹配的检测创建新跟踪
  ├─ 6. 删除：丢失过久的跟踪
  └─ 7. 输出：已确认的跟踪（最多3个）
```

**为什么最多输出 3 个**：
- 场地中最多有 3 个桶
- 限制输出数量可以减少后续处理的计算量

**`std::remove_if` + `erase` 惯用法**：
- `remove_if` 将满足条件的元素移到容器末尾，返回新的逻辑结尾
- `erase` 删除从新结尾到原结尾的元素
- 这是 C++ 中删除满足条件元素的"标准写法"

---

## 第362-380行：异步取帧

```cpp
cv::Mat latest_frame;
std::mutex frame_mutex;
std::atomic<bool> stop_flag{false};

void capture_thread(cv::VideoCapture& cap) {
    cv::Mat tmp;
    while (!stop_flag.load()) {
        try {
            cap >> tmp;
            if (tmp.empty()) continue;
            if (tmp.type() == CV_8UC3) {
                std::lock_guard<std::mutex> lock(frame_mutex);
                tmp.copyTo(latest_frame);
            }
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("Capture error: " << e.what());
        }
    }
}
```

**为什么需要独立的取帧线程**：
- `cap >> tmp` 是阻塞操作，如果摄像头帧率较慢，会阻塞主线程
- 独立线程可以持续取帧，确保 latest_frame 始终是最新的

**`CV_8UC3` 检查**：
- 8 位无符号整数，3 通道（BGR）
- 确保图像格式正确

**`tmp.copyTo(latest_frame)`**：
- 深拷贝，避免共享数据

---

## 第387-407行：改进版 NMS

```cpp
void apply_nms(std::vector<Detection>& dets, float iou_thres) {
    if (dets.empty()) return;
    // 按置信度降序排列
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) { return a.conf > b.conf; });

    std::vector<Detection> result;
    while (!dets.empty()) {
        // 保留置信度最高的框
        result.push_back(dets[0]);
        std::vector<Detection> remaining;

        for (size_t i = 1; i < dets.size(); ++i) {
            if (!is_same_target(dets[0], dets[i], iou_thres, CENTER_DIST_RATIO)) {
                remaining.push_back(dets[i]);
            }
        }
        dets = remaining;
    }
    dets = result;
}
```

**标准 NMS vs 改进 NMS**：

| 步骤 | 标准 NMS | 改进 NMS |
|------|---------|---------|
| 排序 | 按置信度降序 | 按置信度降序 |
| 判定条件 | IOU > 阈值 | `is_same_target()`（IOU + 中心包含 + 中心距离） |
| 适用场景 | 通用目标检测 | 圆形桶（光照变化导致重复检测） |

---

## 第412-472行：YOLOv8 输出解码（4线程并行）

```cpp
static std::vector<Detection> parse_chunk(float* output, int start, int end,
                                          float scale_x, float scale_y,
                                          int img_width, int img_height) {
    std::vector<Detection> chunk_dets;
    for (int i = start; i < end; ++i) {
        float x_center = output[0 * NUM_ANCHORS + i];
        float y_center = output[1 * NUM_ANCHORS + i];
        float width    = output[2 * NUM_ANCHORS + i];
        float height   = output[3 * NUM_ANCHORS + i];

        float x1 = x_center - width / 2.0f;
        float y1 = y_center - height / 2.0f;
        float x2 = x_center + width / 2.0f;
        float y2 = y_center + height / 2.0f;

        float max_conf = 0.0f;
        for (int c = 0; c < NUM_CLASSES; ++c) {
            float score = output[(4 + c) * NUM_ANCHORS + i];
            if (score > max_conf) max_conf = score;
        }

        if (max_conf >= CONF_THRESH) {
            Detection d;
            d.x1 = std::max(0.0f, std::min(x1 * scale_x, static_cast<float>(img_width - 1)));
            d.y1 = std::max(0.0f, std::min(y1 * scale_y, static_cast<float>(img_height - 1)));
            d.x2 = std::max(0.0f, std::min(x2 * scale_x, static_cast<float>(img_width - 1)));
            d.y2 = std::max(0.0f, std::min(y2 * scale_y, static_cast<float>(img_height - 1)));
            d.conf = max_conf;
            if (d.x2 > d.x1 && d.y2 > d.y1) chunk_dets.push_back(d);
        }
    }
    return chunk_dets;
}

std::vector<Detection> parse_yolov8(float* output, int img_width, int img_height) {
    float scale_x = static_cast<float>(img_width) / INPUT_W;
    float scale_y = static_cast<float>(img_height) / INPUT_H;

    const int NUM_THREADS = 4;
    const int CHUNK_SIZE = NUM_ANCHORS / NUM_THREADS;  // 2100

    std::future<std::vector<Detection>> futures[NUM_THREADS];

    for (int t = 0; t < NUM_THREADS; ++t) {
        int start = t * CHUNK_SIZE;
        int end = (t == NUM_THREADS - 1) ? NUM_ANCHORS : start + CHUNK_SIZE;
        futures[t] = std::async(std::launch::async, parse_chunk,
                                output, start, end,
                                scale_x, scale_y,
                                img_width, img_height);
    }

    std::vector<Detection> raw_dets;
    for (int t = 0; t < NUM_THREADS; ++t) {
        auto chunk = futures[t].get();
        raw_dets.insert(raw_dets.end(), chunk.begin(), chunk.end());
    }

    apply_nms(raw_dets, IOU_THRESH);
    return raw_dets;
}
```

**YOLOv8 输出格式**：
```
output[0..8399]     = x_center (归一化 0-1)
output[8400..16799] = y_center
output[16800..25199] = width
output[25200..33599] = height
output[33600..42000+] = class scores (80 classes per anchor)
```

**为什么用 4 线程并行解码**：
- 8400 个锚点，每个需要计算 80 个类别的最大值
- 总计算量：8400 × 80 = 672,000 次比较
- 4 线程并行，每线程处理 2100 个锚点
- 使用 `std::async` 实现，简单高效

**`std::launch::async`**：强制在新线程中异步执行。

**坐标缩放**：
```cpp
d.x1 = std::max(0.0f, std::min(x1 * scale_x, static_cast<float>(img_width - 1)));
```
- `x1 * scale_x`：将归一化坐标缩放到原始图像尺寸
- `std::min(..., img_width-1)`：确保不超出图像边界
- `std::max(0.0f, ...)`：确保不小于 0

---

## 第475-494行：CUDA 加速预处理

```cpp
void preprocess_cuda(const cv::Mat& frame, float* d_input, cudaStream_t stream) {
    cv::cuda::Stream cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);

    cv::cuda::GpuMat gpu_frame, gpu_rgb, gpu_resized, gpu_float;
    gpu_frame.upload(frame, cv_stream);

    cv::cuda::cvtColor(gpu_frame, gpu_rgb, cv::COLOR_BGR2RGB, 0, cv_stream);
    cv::cuda::resize(gpu_rgb, gpu_resized, cv::Size(INPUT_W, INPUT_H), 0, 0,
                     cv::INTER_LINEAR, cv_stream);
    gpu_resized.convertTo(gpu_float, CV_32F, 1.0 / 255.0, cv_stream);

    std::vector<cv::cuda::GpuMat> channels(3);
    for (int c = 0; c < 3; ++c) {
        channels[c] = cv::cuda::GpuMat(INPUT_H, INPUT_W, CV_32F,
            d_input + c * INPUT_H * INPUT_W);
    }
    cv::cuda::split(gpu_float, channels, cv_stream);

    cv_stream.waitForCompletion();
}
```

**预处理流水线（全部在 GPU 上完成）**：

```
原始图像 (BGR, uint8)
  → upload to GPU
  → cvtColor (BGR → RGB)
  → resize (原始尺寸 → 640×640)
  → convertTo (uint8 → float32, 除以 255)
  → split (HWC → CHW 格式)
  → 结果在 d_input 中 (3×640×640 floats)
```

**为什么用 CUDA 预处理**：
- CPU 预处理（特别是 resize 和 cvtColor）在大分辨率图像上很耗时
- GPU 可以并行处理每个像素，速度更快
- 与 TensorRT 推理共享 GPU，减少 CPU-GPU 数据传输

**`cv::cuda::StreamAccessor::wrapStream`**：
- 将 CUDA stream 包装为 OpenCV 的 CUDA stream
- 确保 OpenCV 的 CUDA 操作与 TensorRT 的推理在同一个 stream 中

---

## 第499-546行：推理线程函数

```cpp
void inference_thread_func(
    nvinfer1::IExecutionContext* context,
    void** buffers,
    cudaStream_t stream,
    ThreadSafeQueue<InferenceResult>& result_queue,
    int input_idx, int output_idx,
    size_t output_size)
{
    ROS_INFO("[Visual] Inference thread started.");

    while (!stop_flag.load()) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty())
                latest_frame.copyTo(frame);
        }

        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        try {
            // ---- GPU 工作：预处理 + 推理 + D2H ----
            preprocess_cuda(frame, (float*)buffers[input_idx], stream);
            context->enqueueV2(buffers, stream, nullptr);

            InferenceResult result;
            result.output_data.resize(output_size / sizeof(float));
            result.img_width = frame.cols;
            result.img_height = frame.rows;
            result.timestamp = ros::Time::now().toSec();

            cudaMemcpyAsync(result.output_data.data(), buffers[output_idx],
                            output_size, cudaMemcpyDeviceToHost, stream);
            cudaStreamSynchronize(stream);

            // ---- 推入结果队列（主线程消费） ----
            result_queue.push(std::move(result));

        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("Inference thread error: " << e.what());
        }
    }

    ROS_INFO("[Visual] Inference thread stopped.");
}
```

**推理线程的完整流程**：

```
循环:
  ├─ 1. 从 latest_frame 获取最新帧（加锁）
  ├─ 2. 如果帧为空，休眠 1ms 后重试
  ├─ 3. GPU 预处理 (preprocess_cuda)
  ├─ 4. TensorRT 推理 (enqueueV2)
  ├─ 5. 分配 InferenceResult 内存
  ├─ 6. GPU→CPU 拷贝 (cudaMemcpyAsync)
  ├─ 7. 等待拷贝完成 (cudaStreamSynchronize)
  └─ 8. 推入结果队列
```

**为什么推理线程需要独立的 `IExecutionContext`**：
- TensorRT 的 `IExecutionContext` 是**线程不安全**的
- 每个使用推理的线程必须有自己的 context

---

## 第549-553行：LiDAR 回调

```cpp
void lidar_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    current_lidar.store(msg->range * 100); // 将米转换为厘米，原子存储
    return;
}
```

- 与 `main` 节点一样订阅 LiDAR 数据
- 使用 `atomic` 的 `store()` 方法安全地更新
- 单位转换：米 → 厘米

---

## 第556-587行：ROS 消息打包

```cpp
px4_controller::tbag make_visual_msg(const std::vector<Detection>& tracked)
{
    px4_controller::tbag msg;
    msg.Target1_Exist = 0;
    msg.Target2_Exist = 0;
    msg.Target3_Exist = 0;

    for (size_t i = 0; i < tracked.size() && i < 3; ++i) {
        const Detection& d = tracked[i];
        if (i == 0) {
            msg.Target1_Exist = 1; msg.Target1_PR = d.conf;
            msg.Target1_LU_x = d.x1; msg.Target1_LU_y = d.y1;
            msg.Target1_RU_x = d.x2; msg.Target1_RU_y = d.y1;
            msg.Target1_RD_x = d.x2; msg.Target1_RD_y = d.y2;
            msg.Target1_LD_x = d.x1; msg.Target1_LD_y = d.y2;
        } else if (i == 1) {
            // ... Target2 ...
        } else if (i == 2) {
            // ... Target3 ...
        }
    }
    return msg;
}
```

**将 Detection 转换为 tbag 消息**：
- `tbag` 消息包含 3 个目标的检测框信息
- 每个目标有 4 个角点（LU/RU/RD/LD）和置信度
- 最多填充 3 个目标

---

## 第590-877行：主函数

```cpp
int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_node");
    setlocale(LC_ALL,"");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<px4_controller::tbag>("IR", 1);
    // ...
}
```

### 初始化流程

```
1. ROS 初始化 (visual_node)
2. 创建发布者 → 话题 "IR"
3. 加载标定数据
4. 订阅 LiDAR
5. 加载 TensorRT 引擎
6. 创建推理 context/stream/buffers
7. 打开摄像头 (GStreamer)
8. 启动取帧线程
9. 启动推理线程
10. 初始化 SORT 跟踪器
11. 主循环（CPU 后处理 + 显示 + 发布）
12. 清理
```

### 第594行：创建发布者

```cpp
ros::Publisher pub = nh.advertise<px4_controller::tbag>("IR", 1);
```

- 发布到话题 `"IR"`
- 队列大小 1（只保留最新消息）
- `main` 节点的 `visual_sub` 订阅这个话题

### 第613-614行：读取参数

```cpp
nh.param<std::string>("engine_path", engine_path, "best20260630_yolov8n.engine");
nh.param<bool>("show_debug", show_debug, false);
```

- `engine_path`：TensorRT 引擎文件路径，可在 launch 文件中设置
- `show_debug`：是否显示调试窗口

### 第623-633行：加载引擎文件

```cpp
std::ifstream engine_file(engine_path, std::ios::binary);
engine_file.seekg(0, std::ios::end);
size_t size = engine_file.tellg();
engine_file.seekg(0, std::ios::beg);
std::vector<char> engine_data(size);
engine_file.read(engine_data.data(), size);
```

- 以二进制模式打开引擎文件
- 获取文件大小
- 读取整个文件到 `engine_data` 向量

### 第635-636行：反序列化引擎

```cpp
nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engine_data.data(), size);
```

- `createInferRuntime`：创建 TensorRT runtime
- `deserializeCudaEngine`：从内存中反序列化引擎

### 第643-650行：获取输入输出绑定

```cpp
int input_idx = engine->getBindingIndex("images");
int output_idx = engine->getBindingIndex("output0");
```

- `"images"`：YOLOv8 的输入张量名称
- `"output0"`：YOLOv8 的输出张量名称

### 第657-670行：创建推理资源

```cpp
nvinfer1::IExecutionContext* infer_context = engine->createExecutionContext();
void* infer_buffers[2];
cudaMalloc(&infer_buffers[input_idx], input_size);
cudaMalloc(&infer_buffers[output_idx], output_size);
cudaStream_t infer_stream;
cudaStreamCreate(&infer_stream);
```

- `IExecutionContext`：推理上下文（线程不安全）
- `cudaMalloc`：在 GPU 上分配输入输出缓冲区
- `cudaStreamCreate`：创建 CUDA 流

### 第678-682行：GStreamer 管道

```cpp
std::string gst_pipe =
    "v4l2src device=/dev/video0 ! "
    "image/jpeg, width=1920, height=1200, framerate=120/1 ! "
    "jpegdec ! videoconvert ! video/x-raw, format=BGR ! "
    "appsink max-buffers=1 drop=1";
```

- `v4l2src`：从 /dev/video0 读取视频
- `image/jpeg`：摄像头输出 JPEG 格式
- `width=1920, height=1200, framerate=120/1`：1920×1200 @ 120fps
- `jpegdec`：硬件 JPEG 解码
- `videoconvert`：格式转换
- `appsink max-buffers=1 drop=1`：最多缓存 1 帧，新帧到来时丢弃旧帧

### 第710-718行：启动线程

```cpp
std::thread t_capture(capture_thread, std::ref(cap));
std::thread t_inference(inference_thread_func,
                        infer_context, infer_buffers, infer_stream,
                        std::ref(result_queue),
                        input_idx, output_idx, output_size);
```

- 取帧线程：持续从摄像头读取最新帧
- 推理线程：持续对最新帧进行 GPU 推理

### 第740-857行：主循环

```cpp
while (ros::ok()) {
    InferenceResult result;
    bool got_result = result_queue.try_pop(result);

    if (got_result) {
        // CPU 后处理：解码 + NMS + SORT
        auto detections = parse_yolov8(result.output_data.data(),
                                       result.img_width, result.img_height);
        auto tracked = tracker.update(detections, result.img_width, result.img_height);

        // 30Hz 发布
        if (now_pub - last_publish_time >= PUBLISH_INTERVAL) {
            pub.publish(make_visual_msg(tracked));
            last_publish_time = now_pub;
        }

        // 调试显示
        if (show_debug) {
            // 绘制检测框、跟踪框、标定点
            // 显示 FPS
        }
    }

    ros::spinOnce();
    if (!got_result) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
```

**主循环只做 CPU 工作**：
- 解码 YOLOv8 输出（4 线程并行）
- NMS 去重
- SORT 跟踪
- 绘制显示
- 发布 ROS 消息

**30Hz 发布限制**：
```cpp
const double PUBLISH_INTERVAL = 1.0 / 30.0;
```
- 推理可能比 30Hz 更快
- 但主节点瞄准模块以 30Hz 运行
- 限制发布频率可以避免队列堆积

### 第860-876行：清理

```cpp
stop_flag.store(true);
if (t_capture.joinable()) t_capture.join();
if (t_inference.joinable()) t_inference.join();
cudaStreamDestroy(infer_stream);
cudaFree(infer_buffers[input_idx]);
cudaFree(infer_buffers[output_idx]);
infer_context->destroy();
engine->destroy();
runtime->destroy();
cap.release();
```

**清理顺序**：
1. 设置停止标志
2. 等待线程结束（join）
3. 释放 CUDA 资源
4. 释放 TensorRT 资源
5. 释放摄像头

---

## 总结：cvisual.cpp 的架构

```
┌─────────────────────────────────────────────────────────┐
│                   取帧线程 (capture_thread)               │
│  摄像头 → cap >> tmp → latest_frame (加锁更新)           │
└──────────────────────────┬──────────────────────────────┘
                           │ latest_frame
                           ▼
┌─────────────────────────────────────────────────────────┐
│                   推理线程 (inference_thread_func)         │
│  latest_frame → preprocess_cuda (GPU)                    │
│              → TensorRT 推理 (GPU)                       │
│              → cudaMemcpyAsync (GPU→CPU)                 │
│              → result_queue.push()                       │
└──────────────────────────┬──────────────────────────────┘
                           │ InferenceResult
                           ▼
┌─────────────────────────────────────────────────────────┐
│                   主线程 (main loop)                      │
│  result_queue.try_pop() → parse_yolov8() (4线程并行)     │
│                         → apply_nms()                    │
│                         → SORTTracker.update()           │
│                         → make_visual_msg()              │
│                         → pub.publish("IR")              │
│                         → 调试显示 (show_debug)           │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
                    main 节点的 visual_cb()
                           │
                           ▼
                    VisualData 全局变量
                           │
                           ▼
                    aim.cpp Positioning()