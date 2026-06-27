// Drone.cpp — 无人机状态机实现
#include "Drone.h"
#include "control.h"       // TakeOff, GoHome, Land, Detect, SetPoint...
#include "aim.h"           // Locating, Positioning...
#include "communication.h" // current_state
#include <ros/ros.h>

namespace {
std::string StateName(DroneState s)
{
    switch (s) {
    case DroneState_NONE:     return "NONE";
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

// ── 构造函数 ──────────────────────────────────
Drone::Drone()
    : current_state_(DroneState_NONE)
    , previous_state_(DroneState_NONE)
{
    BuildTransitionTable();
    ROS_INFO("[Drone] 状态机初始化, 当前: %s", StateName(current_state_).c_str());
}

std::string Drone::GetStateName() const { return StateName(current_state_); }

// ── 转移表 ────────────────────────────────────
void Drone::BuildTransitionTable()
{
    transition_table_[DroneState_NONE]  = { DroneState_TAKEOFF };
    transition_table_[DroneState_TAKEOFF]  = { DroneState_GOAL, DroneState_ZHENCHA, DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_GOAL]     = { DroneState_MIAOZHUN, DroneState_ZHENCHA, DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_MIAOZHUN] = { DroneState_ZHENCHA, DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_ZHENCHA]  = { DroneState_RETURN, DroneState_LAND };
    transition_table_[DroneState_RETURN]   = { DroneState_LAND };
    transition_table_[DroneState_LAND]     = {};
}

bool Drone::IsTransitionValid(DroneState from, DroneState to) const
{
    if (from == to) return true;
    auto it = transition_table_.find(from);
    return it != transition_table_.end() && it->second.count(to);
}

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

// ── Web 远程命令 ──────────────────────────────
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

// ── 核心分发器 ────────────────────────────────
void Drone::HandleState()
{
    switch (current_state_)
    {    
    case DroneState_NONE:                         break;
    case DroneState_TAKEOFF:  ExecuteTakeOff();   break;
    case DroneState_GOAL:     ExecuteGoal();      break;
    case DroneState_RETURN:   ExecuteReturn();    break;
    case DroneState_LAND:     ExecuteLand();      break;
    case DroneState_ZHENCHA:  ExecuteZhencha();   break;
    case DroneState_MIAOZHUN: ExecuteMiaozhun();  break;
    }
}

// ── Entry / Exit 钩子 ─────────────────────────
void Drone::OnEnterState(DroneState state)
{
    ROS_INFO("[Drone] >>> 进入: %s", StateName(state).c_str());
}

void Drone::OnExitState(DroneState state)
{
    ROS_INFO("[Drone] <<< 退出: %s", StateName(state).c_str());
}

// ═══════════════════════════════════════════════
//  各状态执行函数（空壳 — 等待手动填充）
// ═══════════════════════════════════════════════

void Drone::ExecuteTakeOff()
{
    TakeOff(5);
    RequestTransition(DroneState_GOAL);
}

void Drone::ExecuteGoal()
{
    // 飞往投放区 + 识别目标桶 + 瞄准投放
    Locating();
    RequestTransition(DroneState_ZHENCHA);
}

void Drone::ExecuteReturn()
{
    GoHome();
    RequestTransition(DroneState_LAND);
}

void Drone::ExecuteLand()
{
    Land();
    // 终态，不再跳转
}

void Drone::ExecuteZhencha()
{
    Detect();
    RequestTransition(DroneState_RETURN);
}

void Drone::ExecuteMiaozhun()
{
    // 如果需要单独瞄准投放（不经过 Locating 的完整流程）
    Locating();
    RequestTransition(DroneState_ZHENCHA);
}