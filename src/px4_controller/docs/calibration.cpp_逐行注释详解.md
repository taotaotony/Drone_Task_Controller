# calibration.cpp 逐行注释详解

---

## 文件概述

`calibration.cpp` 实现了 `HeightCalibration` 类的所有方法以及文件加载函数。核心功能：

1. **数据管理**：添加标定数据、排序验证
2. **线性插值查询**：根据高度实时计算对应的像素坐标
3. **文件解析**：从文本文件读取标定数据，支持空格和逗号分隔

---

## 第1行：包含头文件

```cpp
#include "calibration.h"
```

---

## 第2-7行：标准库头文件

```cpp
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <iostream>
```

| 头文件 | 用途 |
|--------|------|
| `<fstream>` | `std::ifstream` 文件读取 |
| `<sstream>` | `std::stringstream` 字符串解析 |
| `<algorithm>` | `std::sort` 排序、`std::upper_bound` 二分查找 |
| `<numeric>` | `std::iota` 生成连续索引 |
| `<stdexcept>` | `std::runtime_error` 异常 |
| `<iostream>` | `std::cerr` / `std::cout` 控制台输出 |

---

## 第9-10行：使用命名空间

```cpp
using namespace cv;
using namespace std;
```

**作用**：避免每次使用 `cv::` 和 `std::` 前缀。

**为什么在 .cpp 中使用 `using namespace` 是可以的**：
- `using namespace` 在头文件中是禁忌（会污染所有包含者的命名空间）
- 在 .cpp 文件中使用是安全的，只影响当前编译单元

---

## 第16-20行：添加标定数据

```cpp
void HeightCalibration::addCalibrationData(double h, Point2f p1, Point2f p2) {
    heights_.push_back(h);
    pts1_.push_back(p1);
    pts2_.push_back(p2);
}
```

**作用**：将一组标定数据添加到向量末尾。

**为什么不排序插入**：
- 添加时保持插入顺序，不排序
- 等所有数据添加完毕后，调用 `sortAndValidate()` 统一排序
- 这样批量排序比每次插入都排序更高效

---

## 第22-47行：排序与验证

```cpp
void HeightCalibration::sortAndValidate() {
    if (heights_.empty()) return;

    // 生成索引并按高度排序
    vector<size_t> idx(heights_.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&](size_t i, size_t j) {
        return heights_[i] < heights_[j];
    });

    // 按排序后的索引重新排列数据
    vector<double> sorted_h;
    vector<Point2f> sorted_p1, sorted_p2;
    sorted_h.reserve(heights_.size());
    sorted_p1.reserve(heights_.size());
    sorted_p2.reserve(heights_.size());

    for (size_t i : idx) {
        sorted_h.push_back(heights_[i]);
        sorted_p1.push_back(pts1_[i]);
        sorted_p2.push_back(pts2_[i]);
    }

    heights_ = move(sorted_h);
    pts1_ = move(sorted_p1);
    pts2_ = move(sorted_p2);
}
```

### 第26-30行：索引排序法

```cpp
vector<size_t> idx(heights_.size());
iota(idx.begin(), idx.end(), 0);
sort(idx.begin(), idx.end(), [&](size_t i, size_t j) {
    return heights_[i] < heights_[j];
});
```

**为什么用索引排序而不是直接排序**：
- 需要同时排序三个向量（`heights_`、`pts1_`、`pts2_`）
- 直接排序 `heights_` 会丢失与其他向量的对应关系
- 索引排序法：生成 `[0, 1, 2, ...]` 的索引数组，按 `heights_[i]` 排序索引，再按排序后的索引重建三个向量

**`std::iota` 的作用**：
- 从起始值开始，依次递增赋值
- `iota(idx.begin(), idx.end(), 0)` → `idx = [0, 1, 2, 3, ...]`

### 第34-36行：预分配内存

```cpp
sorted_h.reserve(heights_.size());
sorted_p1.reserve(heights_.size());
sorted_p2.reserve(heights_.size());
```

- `reserve()` 预分配内存，避免 `push_back` 时多次扩容
- 提高性能

### 第44-46行：移动语义

```cpp
heights_ = move(sorted_h);
pts1_ = move(sorted_p1);
pts2_ = move(sorted_p2);
```

- `std::move` 将 `sorted_*` 的资源"移动"到成员变量中
- 避免拷贝，提高性能
- 移动后 `sorted_*` 变为空向量

---

## 第49-80行：核心查询函数

```cpp
pair<Point2f, Point2f> HeightCalibration::query(double h) const {
    if (heights_.empty()) {
        throw runtime_error("HeightCalibration: 标定数据为空，无法查询");
    }

    // 边界处理
    if (h <= heights_.front()) {
        return {pts1_.front(), pts2_.front()};
    }
    if (h >= heights_.back()) {
        return {pts1_.back(), pts2_.back()};
    }

    // 二分查找区间
    auto it = upper_bound(heights_.begin(), heights_.end(), h);
    int idx_high = int(it - heights_.begin());
    int idx_low = idx_high - 1;

    double h_low = heights_[idx_low];
    double h_high = heights_[idx_high];
    double ratio = (h - h_low) / (h_high - h_low);

    Point2f p1_low = pts1_[idx_low];
    Point2f p1_high = pts1_[idx_high];
    Point2f p2_low = pts2_[idx_low];
    Point2f p2_high = pts2_[idx_high];

    Point2f p1_interp = p1_low + (p1_high - p1_low) * ratio;
    Point2f p2_interp = p2_low + (p2_high - p2_low) * ratio;

    return {p1_interp, p2_interp};
}
```

### 第50-52行：空数据异常

```cpp
if (heights_.empty()) {
    throw runtime_error("HeightCalibration: 标定数据为空，无法查询");
}
```

- 如果标定数据为空，抛出异常
- 调用者需要捕获异常或确保数据非空

### 第55-60行：边界处理

```cpp
if (h <= heights_.front()) {
    return {pts1_.front(), pts2_.front()};
}
if (h >= heights_.back()) {
    return {pts1_.back(), pts2_.back()};
}
```

- 如果查询高度小于最小标定高度，返回最小高度的数据
- 如果查询高度大于最大标定高度，返回最大高度的数据
- 这种"夹紧"（clamp）行为确保不会返回无效值

### 第63-65行：二分查找

```cpp
auto it = upper_bound(heights_.begin(), heights_.end(), h);
int idx_high = int(it - heights_.begin());
int idx_low = idx_high - 1;
```

**`std::upper_bound` 的作用**：
- 在有序序列中查找**第一个大于** `h` 的元素
- 返回指向该元素的迭代器
- 时间复杂度 O(log n)

**示例**：
```
heights_ = [59, 64, 66, 71, 74, 77, 79, 85, 88, 91, 95, 97]
h = 70
upper_bound 返回指向 71 的迭代器
idx_high = 3（71 的索引）
idx_low = 2（66 的索引）
```

### 第69行：插值比例

```cpp
double ratio = (h - h_low) / (h_high - h_low);
```

- `ratio` 范围在 (0, 1) 之间
- 表示 `h` 在区间 `[h_low, h_high]` 中的相对位置

### 第76-77行：线性插值

```cpp
Point2f p1_interp = p1_low + (p1_high - p1_low) * ratio;
Point2f p2_interp = p2_low + (p2_high - p2_low) * ratio;
```

- `Point2f` 的 `+` 和 `-` 运算符是 OpenCV 提供的
- 插值公式：`p = p_low + (p_high - p_low) * ratio`

---

## 第82-84行：获取数据数量

```cpp
size_t HeightCalibration::size() const {
    return heights_.size();
}
```

---

## 第88-139行：文件加载函数

```cpp
bool loadCalibrationFromFile(const string& filename, HeightCalibration& calib) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "[HeightCalibration] 无法打开文件: " << filename << endl;
        return false;
    }

    string line;
    int line_num = 0;
    while (getline(file, line)) {
        line_num++;

        // 去除首尾空白
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line.empty() || line[0] == '#') continue;

        stringstream ss(line);
        double h, x1, y1, x2, y2;
        char comma;

        // 尝试按空格分隔
        if (ss >> h >> x1 >> y1 >> x2 >> y2) {
            calib.addCalibrationData(h, Point2f((float)x1, (float)y1), Point2f((float)x2, (float)y2));
        } else {
            // 尝试按逗号分隔 (CSV)
            ss.clear();
            ss.str(line);
            if (ss >> h >> comma >> x1 >> comma >> y1 >> comma >> x2 >> comma >> y2) {
                calib.addCalibrationData(h, Point2f((float)x1, (float)y1), Point2f((float)x2, (float)y2));
            } else {
                cerr << "[HeightCalibration] 警告: 第 " << line_num << " 行格式错误，已跳过" << endl;
            }
        }
    }

    file.close();

    if (calib.size() == 0) {
        cerr << "[HeightCalibration] 未读取到有效数据" << endl;
        return false;
    }

    calib.sortAndValidate();
    cout << "[HeightCalibration] 成功加载 " << calib.size() << " 组数据" << endl;
    return true;
}
```

### 第89-93行：打开文件

```cpp
ifstream file(filename);
if (!file.is_open()) {
    cerr << "[HeightCalibration] 无法打开文件: " << filename << endl;
    return false;
}
```

- `std::ifstream` 是输入文件流
- 如果文件不存在或无法打开，返回 `false`

### 第97行：逐行读取

```cpp
while (getline(file, line)) {
```

- `std::getline` 从文件中读取一行
- 返回 `false` 时表示文件读取完毕

### 第101-104行：去除空白和注释

```cpp
line.erase(0, line.find_first_not_of(" \t\r\n"));
line.erase(line.find_last_not_of(" \t\r\n") + 1);
if (line.empty() || line[0] == '#') continue;
```

- 去除行首和行尾的空白字符（空格、制表符、回车、换行）
- 跳过空行和以 `#` 开头的注释行

### 第106-127行：双格式解析

```cpp
stringstream ss(line);
double h, x1, y1, x2, y2;
char comma;

// 尝试按空格分隔
if (ss >> h >> x1 >> y1 >> x2 >> y2) {
    calib.addCalibrationData(...);
} else {
    // 尝试按逗号分隔 (CSV)
    ss.clear();
    ss.str(line);
    if (ss >> h >> comma >> x1 >> comma >> y1 >> comma >> x2 >> comma >> y2) {
        calib.addCalibrationData(...);
    } else {
        cerr << "格式错误" << endl;
    }
}
```

**为什么支持两种格式**：
- 空格分隔：手动编辑更方便
- 逗号分隔（CSV）：可以从 Excel 等软件导出

**解析逻辑**：
1. 先用 `>>` 尝试按空格解析 5 个数字
2. 如果失败，用 `clear()` 清除错误状态，`str(line)` 重置字符串
3. 再用 `>> comma` 跳过逗号的方式解析 CSV

---

## 总结：calibration.cpp 的完整流程

```
loadCalibrationFromFile(file, calib)
  │
  ├─ 打开文件
  ├─ 逐行读取
  │   ├─ 跳过空行和注释
  │   ├─ 解析空格分隔格式
  │   └─ 解析逗号分隔格式
  ├─ 关闭文件
  ├─ 检查是否有有效数据
  └─ sortAndValidate()
      │
      └─ 索引排序 → 重建三个向量

query(h)
  │
  ├─ 空数据检查
  ├─ 边界夹紧
  ├─ upper_bound 二分查找
  ├─ 线性插值计算
  └─ 返回 (p1, p2)