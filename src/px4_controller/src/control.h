#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "communication.h"

extern ros::Time last_request;

// ── 控制相关函数声明 ──────────────────────────
bool SetMode(std::string md);
bool PX4_SetMode(std::string md);
bool Arm();
bool SetPoint(double x, double y, double z);
bool SetVel(double vx, double vy, double pz);
bool CheckPosition(float x, float y, float z);
bool ThrowBottle(int cmd);

void ShowPosition(int delay);
void TakeOff(double waittime);
void Detect();
void GoHome();
void Land();
void SlowDescend(double px, double py, double Lh, double Nh);
void SlowMoveForward(double px, double pz, double Ly, double Ny);

#endif // __CONTROL_H
