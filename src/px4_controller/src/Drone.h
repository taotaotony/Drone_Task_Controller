#ifndef DRONE_H
#define DRONE_H

#include <httplib.h>
#include <string>
#include <map>
#include <set>

// ── 无人机状态枚举 ────────────────────────────
enum DroneState
{
    DroneState_NONE  = -1,     // 空状态
    DroneState_TAKEOFF  = 0,   // 起飞
    DroneState_GOAL     = 1,   // 飞往目标点
    DroneState_RETURN   = 2,   // 返航
    DroneState_LAND     = 3,   // 降落
    DroneState_ZHENCHA  = 4,   // 侦察
    DroneState_MIAOZHUN = 5    // 瞄准投放
};

// ── 无人机状态机 ──────────────────────────────
class Drone
{
public:
    Drone();

    // ── 核心接口 ──────────────────────────
    /// ROS 主循环中每帧调用，根据当前状态执行相应操作
    void HandleState();

    /// Web 远程命令入口（/setstat 路由回调）
    void UpdateState(const httplib::Request& req, httplib::Response& res);

    // ── 状态查询 ──────────────────────────
    DroneState GetState() const { return current_state_; }
    std::string GetStateName() const;
    bool IsState(DroneState s) const { return current_state_ == s; }

    // ── 内部状态转移 ──────────────────────
    /// 请求状态跳转（带合法性验证），返回是否成功
    bool RequestTransition(DroneState new_state);

protected:
    // ════════════════════════════════════════
    //  各状态执行函数（空壳，需手动填充逻辑）
    // ════════════════════════════════════════

    /// 进入某状态时调用一次（初始化、日志等）
    virtual void OnEnterState(DroneState state);

    /// 离开某状态时调用一次（清理、收尾）
    virtual void OnExitState(DroneState state);

    /// 起飞流程：连接飞控 → 切模式 → 解锁 → 起飞
    virtual void ExecuteTakeOff();

    /// 飞往目标流程：导航至预设/手动目标点
    virtual void ExecuteGoal();

    /// 返航流程：中断当前任务，返回起飞点
    virtual void ExecuteReturn();

    /// 降落流程：切 LAND 模式 → 等待落地
    virtual void ExecuteLand();

    /// 侦察流程：按照航线扫描目标区域并识别
    virtual void ExecuteZhencha();

    /// 瞄准投放流程：视觉锁定目标桶 → PID 瞄准 → 投放
    virtual void ExecuteMiaozhun();

private:
    DroneState current_state_;
    DroneState previous_state_;

    /// 状态转移表：每个状态允许跳转到的目标状态集合
    std::map<DroneState, std::set<DroneState>> transition_table_;

    /// 构建转移规则
    void BuildTransitionTable();

    /// 判断转移是否合法
    bool IsTransitionValid(DroneState from, DroneState to) const;
};

#endif // DRONE_H

