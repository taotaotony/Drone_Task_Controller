# calibration.h 逐行注释详解

---

## 文件概述

`calibration.h` 定义了**高度标定系统**的数据结构和接口。它解决了一个关键问题：**不同高度下，桶在图像中的像素位置不同**，需要根据 LiDAR 测量的高度实时计算瞄准中心。

核心功能：
1. **`HeightCalibration` 类**：存储高度→像素坐标的映射表，支持线性插值查询
2. **`loadCalibrationFromFile` 函数**：从文本文件加载标定数据
3. **`Vec2f` 结构体**：自定义二维向量，支持基本数学运算

---

## 第1-2行：头文件保护

```cpp
#ifndef HEIGHT_CALIBRATION_H
#define HEIGHT_CALIBRATION_H
```

---

## 第4行：核心头文件

```cpp
#include "main.h"
```

**作用**：引入 `main.h` 中的基础类型定义。

---

## 第5行：OpenCV 核心库

```cpp
#include <opencv2/opencv.hpp>
```

**作用**：引入 OpenCV 库的所有核心功能。

**为什么需要 OpenCV**：
- 使用 `cv::Point2f` 表示图像中的二维点
- `cv::Point2f` 是 `(float x, float y)` 的结构体，支持加减乘除运算

---

## 第6-9行：标准库

```cpp
#include <vector>
#include <utility>
#include <string>
#include <cmath>
```

**各头文件用途**：

| 头文件 | 用途 |
|--------|------|
| `<vector>` | `std::vector` 存储标定数据列表 |
| `<utility>` | `std::pair` 返回两个坐标点 |
| `<string>` | `std::string` 文件路径 |
| `<cmath>` | `std::sqrt` 等数学函数 |

---

## 第12-31行：HeightCalibration 类

```cpp
class HeightCalibration {
public:
    void addCalibrationData(double h, cv::Point2f p1, cv::Point2f p2);
    void sortAndValidate();
    std::pair<cv::Point2f, cv::Point2f> query(double h) const;
    size_t size() const;

private:
    std::vector<double> heights_;
    std::vector<cv::Point2f> pts1_;
    std::vector<cv::Point2f> pts2_;
};
```

### 设计思路

**为什么需要标定**：
- 相机安装在无人机上，与桶口有一定的偏移量
- 不同高度下，桶口在图像中的像素位置不同
- 需要建立"高度 → 像素坐标"的映射关系

**标定数据格式**（`calibration_data.txt`）：
```
#高度   桶1像素    桶2像素
59      (943,444)  (949,793)
64      (955,461)  (952,779)
...
```

**线性插值原理**：
```
给定高度 h，找到相邻的两个标定点 (h₁, p₁) 和 (h₂, p₂)
ratio = (h - h₁) / (h₂ - h₁)
p = p₁ + (p₂ - p₁) * ratio
```

### 第15行：添加标定数据

```cpp
void addCalibrationData(double h, cv::Point2f p1, cv::Point2f p2);
```

- `h`：标定高度（厘米）
- `p1`：桶1在图像中的像素坐标
- `p2`：桶2在图像中的像素坐标
- 数据按添加顺序存储，不保证有序

### 第18行：排序与验证

```cpp
void sortAndValidate();
```

- 按高度升序排序
- 必须在所有数据添加完毕后调用一次
- 排序后 `query()` 才能正确进行二分查找

### 第22行：核心查询接口

```cpp
std::pair<cv::Point2f, cv::Point2f> query(double h) const;
```

- 输入：当前高度 `h`（厘米）
- 输出：`(桶1像素坐标, 桶2像素坐标)`
- 如果 `h` 在标定范围内，线性插值
- 如果 `h` 超出范围，返回最近端点的值
- 如果数据为空，抛出 `std::runtime_error`

### 第28-30行：私有成员

```cpp
std::vector<double> heights_;   // 标定高度（升序）
std::vector<cv::Point2f> pts1_; // 对应的坐标1
std::vector<cv::Point2f> pts2_; // 对应的坐标2
```

- 三个向量长度相同，相同索引对应一组标定数据
- `heights_` 经过 `sortAndValidate()` 后保证升序

---

## 第38行：文件加载函数

```cpp
bool loadCalibrationFromFile(const std::string& filename, HeightCalibration& calib);
```

**参数**：
- `filename`：标定文件路径
- `calib`：引用方式传入的 `HeightCalibration` 对象，函数将数据填充到其中

**返回值**：
- `true`：成功加载至少一组数据
- `false`：文件无法打开或没有有效数据

**支持的文件格式**：
```
# 注释行（以 # 开头）
59 943 444 949 793    ← 空格分隔
59,943,444,949,793    ← 逗号分隔（CSV）
```

---

## 第42-62行：Vec2f 结构体

```cpp
struct Vec2f {
    float x, y;
    Vec2f() : x(0), y(0) {}
    Vec2f(float x_, float y_) : x(x_), y(y_) {}
    Vec2f operator+(const Vec2f& other) const { return Vec2f(x + other.x, y + other.y); }
    Vec2f operator-(const Vec2f& other) const { return Vec2f(x - other.x, y - other.y); }
    Vec2f operator*(float scalar) const { return Vec2f(x * scalar, y * scalar); }
    Vec2f& operator+=(const Vec2f& other) { x += other.x; y += other.y; return *this; }
    Vec2f& operator-=(const Vec2f& other) { x -= other.x; y -= other.y; return *this; }
    Vec2f& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }
    float dot(const Vec2f& other) const { return x * other.x + y * other.y; }
    float length() const { return std::sqrt(x*x + y*y); }
};
```

**为什么需要 Vec2f**：
- OpenCV 的 `cv::Point2f` 不支持 `*` 运算符（标量乘法）
- 自定义 `Vec2f` 提供了完整的向量运算支持
- 虽然当前代码中**没有使用** `Vec2f`，但为未来扩展预留

**运算符重载说明**：

| 运算符 | 作用 | 示例 |
|--------|------|------|
| `+` | 向量加法 | `v1 + v2` |
| `-` | 向量减法 | `v1 - v2` |
| `*` | 标量乘法 | `v * 2.5f` |
| `+=` | 向量加法赋值 | `v1 += v2` |
| `-=` | 向量减法赋值 | `v1 -= v2` |
| `*=` | 标量乘法赋值 | `v *= 2.5f` |
| `dot` | 点积 | `v1.dot(v2)` |
| `length` | 模长 | `v.length()` |

### 第62行：标量左乘

```cpp
inline Vec2f operator*(float scalar, const Vec2f& v) { return v * scalar; }
```

- 支持 `2.5f * v` 的写法（标量在左边）
- 因为成员函数 `operator*` 只能处理 `v * scalar`（标量在右边）
- 全局函数 `operator*` 处理 `scalar * v`

---

## 总结：标定系统的使用流程

```
1. 准备标定数据文件 calibration_data.txt
   └─ 在不同高度下记录桶的像素坐标

2. 程序启动时加载
   └─ loadCalibrationFromFile(file_path, calib)

3. 运行时查询
   └─ auto [p1, p2] = calib.query(lidar_height)
      └─ p1 = 桶1在当前高度的瞄准中心
      └─ p2 = 桶2在当前高度的瞄准中心

4. 用于低空瞄准
   └─ Positioning(ThrowHeight, 300, p1.x, p1.y, MaxVel_L, 1)