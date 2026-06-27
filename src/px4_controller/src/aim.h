#ifndef __AIM_H
#define __AIM_H

#include "main.h"
#include "pid.h"

// ── 类型定义 ────────────────────────────────
struct BarrelPosition
{
    double x, y;
    BarrelPosition(double x = 0, double y = 0) : x(x), y(y) {}
};

// ── 全局变量 extern 声明 ─────────────────────
extern ros::Time time_guard;

// ── 函数声明 ────────────────────────────────
double distance(const BarrelPosition& a, const BarrelPosition& b);
std::vector<BarrelPosition> robustKMeans(const std::vector<BarrelPosition>& data,
                                          int k, int maxIter = 100);
bool Positioning(double pZ, int tim, int CenterX, int CenterY,
                 double MaxVel, int BottleLabel);
void Locating();

#endif // __AIM_H