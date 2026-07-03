#ifndef HEIGHT_CALIBRATION_H
#define HEIGHT_CALIBRATION_H

#include "main.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>
#include <string>
#include <cmath>

// 标定查询类：存储高度 -> 两个坐标点的映射，支持线性插值
class HeightCalibration {
public:
    // 添加一组标定数据（高度h，坐标点p1、p2）
    void addCalibrationData(double h, cv::Point2f p1, cv::Point2f p2);

    // 按高度排序（必须在所有数据添加完毕后调用一次）
    void sortAndValidate();

    // 核心查询接口：给定高度，返回对应的两个坐标点（线性插值）
    // 如果高度超出范围，返回最近端点的值；若数据为空则抛出异常
    std::pair<cv::Point2f, cv::Point2f> query(double h) const;

    // 获取当前标定点数量
    size_t size() const;

private:
    std::vector<double> heights_;   // 标定高度（升序）
    std::vector<cv::Point2f> pts1_; // 对应的坐标1
    std::vector<cv::Point2f> pts2_; // 对应的坐标2
};

// ========== 辅助工具函数 ==========

// 从文本文件加载标定数据，自动填充到 HeightCalibration 对象中
// 支持空格或逗号分隔，忽略以 '#' 开头的注释行和空行
// 返回 true 表示成功加载至少一组数据，false 表示失败
bool loadCalibrationFromFile(const std::string& filename, HeightCalibration& calib);


// 自定义二维向量
struct Vec2f {
    float x, y;

    Vec2f() : x(0), y(0) {}
    Vec2f(float x_, float y_) : x(x_), y(y_) {}

    // 基本运算
    Vec2f operator+(const Vec2f& other) const { return Vec2f(x + other.x, y + other.y); }
    Vec2f operator-(const Vec2f& other) const { return Vec2f(x - other.x, y - other.y); }
    Vec2f operator*(float scalar) const { return Vec2f(x * scalar, y * scalar); }
    Vec2f& operator+=(const Vec2f& other) { x += other.x; y += other.y; return *this; }
    Vec2f& operator-=(const Vec2f& other) { x -= other.x; y -= other.y; return *this; }
    Vec2f& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }

    // 点积、模长等（按需添加）
    float dot(const Vec2f& other) const { return x * other.x + y * other.y; }
    float length() const { return std::sqrt(x*x + y*y); }
};

// 标量左乘
inline Vec2f operator*(float scalar, const Vec2f& v) { return v * scalar; }

#endif // HEIGHT_CALIBRATION_H