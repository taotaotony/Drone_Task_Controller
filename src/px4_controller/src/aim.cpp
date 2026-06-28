// aim.cpp — 瞄准、识别与投放逻辑实现
#include "aim.h"
#include "communication.h"

ros::Time time_guard;

// ── 两点间欧氏距离 ────────────────────────────
double distance(const BarrelPosition& a, const BarrelPosition& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

// ── 鲁棒 K-means 聚类（单次运行） ─────────────
std::vector<BarrelPosition> robustKMeans(const std::vector<BarrelPosition>& data,
                                          int k, int maxIter)
{
    if (data.empty()) return {};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, data.size() - 1);

    std::vector<BarrelPosition> centers;
    for (int i = 0; i < k; ++i) {
        centers.push_back(data[dis(gen)]);
    }

    std::vector<BarrelPosition> newCenters = centers;
    int iter = 0;

    while (iter++ < maxIter) {
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

        bool converged = true;
        for (int i = 0; i < k; ++i) {
            if (clusters[i].empty()) continue;

            std::vector<double> dists;
            for (const auto& p : clusters[i]) {
                dists.push_back(distance(p, centers[i]));
            }

            std::sort(dists.begin(), dists.end());
            double threshold = dists[static_cast<size_t>(dists.size() * 0.9)];

            std::vector<BarrelPosition> inliers;
            for (const auto& p : clusters[i]) {
                if (distance(p, centers[i]) <= threshold) {
                    inliers.push_back(p);
                }
            }

            double sumX = 0, sumY = 0;
            for (const auto& p : inliers) {
                sumX += p.x;
                sumY += p.y;
            }
            newCenters[i] = BarrelPosition(sumX / inliers.size(),
                                           sumY / inliers.size());

            if (distance(newCenters[i], centers[i]) > 1e-5) {
                converged = false;
            }
        }

        centers = newCenters;
        if (converged) break;
    }
    return centers;
}

// ── 瞄准定位函数 ──────────────────────────────
bool Positioning(double pZ, int tim, int CenterX, int CenterY,
                 double MaxVel, int BottleLabel)
{
    ROS_INFO("InPositioning Height=%.2f   Time=%d  Bottle=%d",
             pZ, tim, BottleLabel);
    ros::Rate pid_rate(30);
    int noPos = -1;
    int PosinnoPos = 0;

    while (tim && ros::ok())
    {
        if (VisualData.Target1_Exist || VisualData.Target2_Exist ||
            VisualData.Target3_Exist)
        {
            if (noPos != -1) {
                PosinnoPos++;
                if (PosinnoPos >= 10) { noPos = -1; PosinnoPos = 0; }
            }

            float B1X = (VisualData.Target1_LU_x + VisualData.Target1_RD_x) / 2;
            float B1Y = (VisualData.Target1_LU_y + VisualData.Target1_RD_y) / 2;
            float B2X = (VisualData.Target2_LU_x + VisualData.Target2_RD_x) / 2;
            float B2Y = (VisualData.Target2_LU_y + VisualData.Target2_RD_y) / 2;
            float B3X = (VisualData.Target3_LU_x + VisualData.Target3_RD_x) / 2;
            float B3Y = (VisualData.Target3_LU_y + VisualData.Target3_RD_y) / 2;

            float D1 = (B1X - CenterX) * (B1X - CenterX) +
                       (B1Y - CenterY) * (B1Y - CenterY);
            float D2 = (B2X - CenterX) * (B2X - CenterX) +
                       (B2Y - CenterY) * (B2Y - CenterY);
            float D3 = (B3X - CenterX) * (B3X - CenterX) +
                       (B3Y - CenterY) * (B3Y - CenterY);

            float BarrelX = B1X, BarrelY = B1Y;
            if (D2 < D1 && VisualData.Target2_Exist) {
                BarrelX = B2X; BarrelY = B2Y;
            }
            if (D3 < D2 && D3 < D1 && VisualData.Target3_Exist) {
                BarrelX = B3X; BarrelY = B3Y;
            }

            if ((abs(BarrelX - CenterX) * abs(BarrelX - CenterX) +
                 abs(BarrelY - CenterY) * abs(BarrelY - CenterY) <=
                 MinPx * MinPx) && (BottleLabel != 0))
            {
                SetVel(0, 0, pZ);
                ThrowBottle(BottleLabel);
                ROS_INFO("提前投放 指令%d", BottleLabel);
                ros::spinOnce();
                ros::Duration(1).sleep();
                return true;
            }

            float out_vel_x = -PID_Calculate(&pos_pid_xy, CenterX, BarrelX);
            ROS_WARN("未限制输出Vx:%.5f", out_vel_x);
            if (abs(out_vel_x) < DeadZone) out_vel_x = 0;
            if (out_vel_x > MaxVel) out_vel_x = MaxVel;
            if (out_vel_x < -MaxVel) out_vel_x = -MaxVel;

            float out_vel_y = PID_Calculate(&pos_pid_xy, CenterY, BarrelY);
            ROS_WARN("未限制输出Vy:%.5f", out_vel_y);
            if (abs(out_vel_y) < DeadZone) out_vel_y = 0;
            if (out_vel_y > MaxVel) out_vel_y = MaxVel;
            if (out_vel_y < -MaxVel) out_vel_y = -MaxVel;

            SetVel(out_vel_x, out_vel_y, pZ);
            ROS_INFO("飞机当前位置：%.2f,%.2f,%.2f",
                     PX4_Position.x, PX4_Position.y, PX4_Position.z);
            ROS_INFO("飞机当前速度：%.2f,%.2f,%.2f",
                     PX4_Velocity.vx, PX4_Velocity.vy, PX4_Velocity.vz);
            ROS_INFO("距离投放中心最近桶的中心位置:%.2f,%.2f",
                     BarrelX, BarrelY);
            ROS_INFO("输出速度xy %.3f %.3f", out_vel_x, out_vel_y);
        }
        else
        {
            noPos++;
            if (noPos >= 150) { ros::spinOnce(); return false; }
        }
        tim--;
        pid_rate.sleep();
        ros::spinOnce();
    }

    if (BottleLabel != 0)
    {
        SetVel(0, 0, pZ);
        ThrowBottle(BottleLabel);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        return true;
    }
    return true;
}

// ── 定位与投放主流程 ──────────────────────────
void Locating()
{
    time_guard = ros::Time::now();
    ROS_INFO("飞往投放区");
    SetVel(0, 1.8, InitialHeight);
    ShowPosition(17);
    SetPoint(0, TakeofftoThrow, InitialHeight);
    ShowPosition(5);

    ROS_INFO(">>>>>>>开始识别桶<<<<<<<");
    float B1X, B1Y, B2X, B2Y, B3X, B3Y;
    float B1X_LD, B1Y_LD, B2X_LD, B2Y_LD, B3X_LD, B3Y_LD;
    std::vector<BarrelPosition> allPositions;

    for (int t = 100; t > 0; t--)
    {
        B1X = (VisualData.Target1_LU_x + VisualData.Target1_RD_x) / 2;
        B2X = (VisualData.Target2_LU_x + VisualData.Target2_RD_x) / 2;
        B3X = (VisualData.Target3_LU_x + VisualData.Target3_RD_x) / 2;
        B1Y = (VisualData.Target1_LU_y + VisualData.Target1_RD_y) / 2;
        B2Y = (VisualData.Target2_LU_y + VisualData.Target2_RD_y) / 2;
        B3Y = (VisualData.Target3_LU_y + VisualData.Target3_RD_y) / 2;
        if (B1X != 0 && B1Y != 0) allPositions.push_back({B1X, B1Y});
        if (B2X != 0 && B2Y != 0) allPositions.push_back({B2X, B2Y});
        if (B3X != 0 && B3Y != 0) allPositions.push_back({B3X, B3Y});
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    // 过滤数据
    std::vector<BarrelPosition> filteredData;
    for (const auto& p : allPositions) {
        if (p.x > 0 && p.y > 0 && p.x <= CamX && p.y <= CamY)
            filteredData.push_back(p);
    }

    ROS_INFO("=======记录到的目标桶中心像素坐标=======");
    for (size_t i = 0; i < filteredData.size(); i++)
        ROS_INFO("%.2f,%.2f | ", filteredData[i].x, filteredData[i].y);
    ROS_INFO("总记录点数: %d", (int)filteredData.size());

    // 鲁棒聚类
    std::vector<BarrelPosition> bestCenters;
    int k = 3, numRuns = 5;
    double bestWCSS = std::numeric_limits<double>::max();
    for (int run = 0; run < numRuns; ++run) {
        auto centers = robustKMeans(filteredData, k);
        double wcss = 0;
        for (const auto& p : filteredData) {
            double minDist = std::numeric_limits<double>::max();
            for (const auto& c : centers)
                minDist = std::min(minDist, distance(p, c));
            wcss += minDist * minDist;
        }
        if (wcss < bestWCSS) { bestWCSS = wcss; bestCenters = centers; }
    }

    std::sort(bestCenters.begin(), bestCenters.end(),
              [](const BarrelPosition& a, const BarrelPosition& b) {
                  return a.x < b.x; });

    ROS_INFO("聚类后目标桶坐标:");
    ROS_INFO("1号:(%.2f,%.2f)", bestCenters[0].x, bestCenters[0].y);
    ROS_INFO("2号:(%.2f,%.2f)", bestCenters[1].x, bestCenters[1].y);
    ROS_INFO("3号:(%.2f,%.2f)", bestCenters[2].x, bestCenters[2].y);

    B1X_LD = bestCenters[0].x / 160 - 4;
    B2X_LD = bestCenters[1].x / 160 - 4;
    B3X_LD = bestCenters[2].x / 160 - 4;
    B1Y_LD = (5 - bestCenters[0].y / 160) - 2.5 + TakeofftoThrow;
    B2Y_LD = (5 - bestCenters[1].y / 160) - 2.5 + TakeofftoThrow;
    B3Y_LD = (5 - bestCenters[2].y / 160) - 2.5 + TakeofftoThrow;

    ROS_INFO("飞往目标桶坐标如下:");
    ROS_INFO("1号:(%.2f,%.2f)", B1X_LD, B1Y_LD);
    ROS_INFO("2号:(%.2f,%.2f)", B2X_LD, B2Y_LD);
    ROS_INFO("3号:(%.2f,%.2f)", B3X_LD, B3Y_LD);

    // 投放开始
    ROS_INFO(">>>>>>开始瞄准投放<<<<<<");
    SetPoint(B1X_LD, B1Y_LD, InitialHeight);
    ShowPosition(3);
    PID_Init(&pos_pid_xy, Kp_H, Ki_H, Kd_H);
    Positioning(InitialHeight, 90, CamX / 2, CamY / 2, MaxVel_H, 0);
    SlowDescend(PX4_Position.x, PX4_Position.y, InitialHeight, ThrowMidHeight);
    Positioning(ThrowMidHeight, 90, CamX / 2, CamY / 2, MaxVel_H, 0);
    SlowDescend(PX4_Position.x, PX4_Position.y, ThrowMidHeight, ThrowHeight);
    PID_Init(&pos_pid_xy, Kp_L, Ki_L, Kd_L);

    if (!Positioning(ThrowHeight, 300, Throw1_X, Throw1_Y, MaxVel_L, 1))
    {
        PID_Init(&pos_pid_xy, Kp_H, Ki_H, Kd_H);
        SetPoint(B1X_LD, B1Y_LD, ThrowMidHeight);
        Positioning(ThrowMidHeight, 90, CamX / 2, CamY / 2, MaxVel_H, 0);
        SlowDescend(PX4_Position.x, PX4_Position.y, ThrowMidHeight, ThrowHeight);
        PID_Init(&pos_pid_xy, Kp_L, Ki_L, Kd_L);
        if (!Positioning(ThrowHeight, 300, Throw1_X, Throw1_Y, MaxVel_L, 1))
        {
            ThrowBottle(1);
            ROS_INFO("实在没瞄准，放弃本次投放，进行下一次投放");
        }
    }

    ros::Duration(1).sleep();
    SetPoint(B2X_LD, B2Y_LD, InitialHeight);
    PID_Init(&pos_pid_xy, Kp_H, Ki_H, Kd_H);
    ros::Duration(3).sleep();
    Positioning(InitialHeight, 90, CamX / 2, CamY / 2, MaxVel_H, 0);
    SlowDescend(PX4_Position.x, PX4_Position.y, InitialHeight, ThrowMidHeight);
    Positioning(ThrowMidHeight, 30, CamX / 2, CamY / 2, MaxVel_H, 0);
    SlowDescend(PX4_Position.x, PX4_Position.y, ThrowMidHeight, ThrowHeight);
    PID_Init(&pos_pid_xy, Kp_L, Ki_L, Kd_L);

    if (!Positioning(ThrowHeight, 300, Throw2_X, Throw2_Y, MaxVel_L, 3))
    {
        PID_Init(&pos_pid_xy, Kp_H, Ki_H, Kd_H);
        SetPoint(B2X_LD, B2Y_LD, ThrowMidHeight);
        Positioning(ThrowMidHeight, 90, CamX / 2, CamY / 2, MaxVel_H, 0);
        SlowDescend(PX4_Position.x, PX4_Position.y, ThrowMidHeight, ThrowHeight);
        PID_Init(&pos_pid_xy, Kp_L, Ki_L, Kd_L);
        if (!Positioning(ThrowHeight, 300, Throw2_X, Throw2_Y, MaxVel_L, 3))
        {
            ThrowBottle(3);
            ROS_INFO("实在没瞄准，放弃本次投放，进行侦察");
        }
    }
}