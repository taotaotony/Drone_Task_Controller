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
//瞄准函数
bool Positioning(double pZ,int tim,int CenterX,int CenterY,double MaxVel,int BottleLabel)
{
    ROS_INFO("InPositioning Height=%.2f   Time=%d  Bottle=%d",pZ,tim,BottleLabel);
    ros::Rate pid_rate(30);
    int noPos=-1;  //检测瞄准时是否出现瞄歪了导致看不到桶的情况
    int PosinnoPos=0;  //检测瞄准时出现瞄歪了导致看不到桶的情况下，又看到桶的情况，如果是确实又看到了桶就退出没看到桶的处理方式，确保不会因为视觉模型的一个误识别导致noPos归零进而导致时间很长不去瞄准的情况
    while(tim&&ros::ok())
    {
        if(VisualData.Target1_Exist||VisualData.Target2_Exist||VisualData.Target3_Exist)
        {
            if(noPos!=-1)//目的是防止没看到桶，解决方案：没看到桶一定次数就起来回到中等高度重新去瞄准
            {
                PosinnoPos++;
                if(PosinnoPos>=10)
                {
                    noPos=-1;
                    PosinnoPos=0;
                }
            }
            float Barrel1X=(VisualData.Target1_LU_x+VisualData.Target1_RD_x)/2;
            float Barrel1Y=(VisualData.Target1_LU_y+VisualData.Target1_RD_y)/2;
            float Barrel2X=(VisualData.Target2_LU_x+VisualData.Target2_RD_x)/2;
            float Barrel2Y=(VisualData.Target2_LU_y+VisualData.Target2_RD_y)/2;
            float Barrel3X=(VisualData.Target3_LU_x+VisualData.Target3_RD_x)/2;
            float Barrel3Y=(VisualData.Target3_LU_y+VisualData.Target3_RD_y)/2;
            float DcenterB1=(Barrel1X-CenterX)*(Barrel1X-CenterX)+(Barrel1Y-CenterY)*(Barrel1Y-CenterY);
            float DcenterB2=(Barrel2X-CenterX)*(Barrel2X-CenterX)+(Barrel2Y-CenterY)*(Barrel2Y-CenterY);
            float DcenterB3=(Barrel3X-CenterX)*(Barrel3X-CenterX)+(Barrel3Y-CenterY)*(Barrel3Y-CenterY);

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
            if((abs(BarrelX-CenterX)*abs(BarrelX-CenterX)+abs(BarrelY-CenterY)*abs(BarrelY-CenterY)<=MinPx*MinPx)&&(BottleLabel!=0))
            {
                SetVel(0,0,pZ);
                ThrowBottle(BottleLabel);
                ROS_INFO("提前投放 指令%d",BottleLabel);
                ros::spinOnce();
                ros::Duration(1).sleep();
                return true;
            }
            float out_vel_x=-PID_Calculate(&pos_pid_xy,CenterX,BarrelX);
            ROS_WARN("未限制输出Vx:%.5f",out_vel_x);
            if(abs(out_vel_x)<DeadZone) out_vel_x=0;
            if(out_vel_x>MaxVel) out_vel_x=MaxVel;
            if(out_vel_x<-MaxVel) out_vel_x=-MaxVel;
            float out_vel_y=PID_Calculate(&pos_pid_xy,CenterY,BarrelY);
            ROS_WARN("未限制输出Vy:%.5f",out_vel_y);
            if(abs(out_vel_y)<DeadZone) out_vel_y=0;
            if(out_vel_y>MaxVel) out_vel_y=MaxVel;
            if(out_vel_y<-MaxVel) out_vel_y=-MaxVel;
            SetVel(out_vel_x,out_vel_y,pZ);
            ROS_INFO("飞机当前位置：%.2f,%.2f,%.2f",PX4_Position.x,PX4_Position.y,PX4_Position.z);
            ROS_INFO("飞机当前速度：%.2f,%.2f,%.2f",PX4_Velocity.vx,PX4_Velocity.vy,PX4_Velocity.vz);
            ROS_INFO("距离投放中心最近桶的中心位置:%.2f,%.2f",BarrelX,BarrelY);
            ROS_INFO("输出速度xy %.3f %.3f",out_vel_x,out_vel_y);
        }
        else 
        {
            noPos++;
            if(noPos>=150)
            {
                ros::spinOnce();
                return false;
            }
        }
        tim--;
        pid_rate.sleep();
        ros::spinOnce();
    }
    if(BottleLabel!=0)
    {
        SetVel(0,0,pZ);
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
    time_guard=ros::Time::now();
    ROS_INFO("飞往投放区");
    SetVel(0,1.8,InitialHeight);
    ShowPosition(17);
    SetPoint(0,TakeofftoThrow,InitialHeight);
    ShowPosition(5);

 //   SlowMoveForward(0,InitialHeight,0,TakeofftoThrow);
    int num=2;
    ROS_INFO(">>>>>>>开始识别桶<<<<<<<");
    float Barrel1X,Barrel1Y,Barrel2X,Barrel2Y,Barrel3X,Barrel3Y;
    float Barrel1X_Coord_LD,Barrel1Y_Coord_LD,Barrel2X_Coord_LD,Barrel2Y_Coord_LD,Barrel3X_Coord_LD,Barrel3Y_Coord_LD;
    std::vector<BarrelPosition> allPositions;
    for(int t=100;t>0;t--)
    {
        Barrel1X=(VisualData.Target1_LU_x+VisualData.Target1_RD_x)/2;
        Barrel2X=(VisualData.Target2_LU_x+VisualData.Target2_RD_x)/2;
        Barrel3X=(VisualData.Target3_LU_x+VisualData.Target3_RD_x)/2;
        Barrel1Y=(VisualData.Target1_LU_y+VisualData.Target1_RD_y)/2;
        Barrel2Y=(VisualData.Target2_LU_y+VisualData.Target2_RD_y)/2;
        Barrel3Y=(VisualData.Target3_LU_y+VisualData.Target3_RD_y)/2;

                    
        /*/ROS_INFO("111111");
        ROS_INFO("Barrel1X: %f", Barrel1X);
        ROS_INFO("Barrel2X: %f", Barrel2X);
        ROS_INFO("Barrel3X: %f", Barrel3X);
        ROS_INFO("Barrel1Y: %f", Barrel1Y);
        ROS_INFO("Barrel2Y: %f", Barrel2Y);
        ROS_INFO("Barrel3Y: %f", Barrel3Y); /*/       

        if(Barrel1X != 0 && Barrel1Y != 0){allPositions.push_back({Barrel1X,Barrel1Y});}
        if(Barrel2X != 0 && Barrel2Y != 0){allPositions.push_back({Barrel2X,Barrel2Y});}
        if(Barrel3X != 0 && Barrel3Y != 0){allPositions.push_back({Barrel3X,Barrel3Y});} 
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    if(allPositions.size()==0)
    {
        SetPoint(0,TakeofftoThrow,InitialHeight+1);
        for(int t=100;t>0;t--)
        {
            Barrel1X=(VisualData.Target1_LU_x+VisualData.Target1_RD_x)/2;
            Barrel2X=(VisualData.Target2_LU_x+VisualData.Target2_RD_x)/2;
            Barrel3X=(VisualData.Target3_LU_x+VisualData.Target3_RD_x)/2;
            Barrel1Y=(VisualData.Target1_LU_y+VisualData.Target1_RD_y)/2;
            Barrel2Y=(VisualData.Target2_LU_y+VisualData.Target2_RD_y)/2;
            Barrel3Y=(VisualData.Target3_LU_y+VisualData.Target3_RD_y)/2;


            
            if(Barrel1X != 0 && Barrel1Y != 0){allPositions.push_back({Barrel1X,Barrel1Y});}
            if(Barrel2X != 0 && Barrel2Y != 0){allPositions.push_back({Barrel2X,Barrel2Y});}
            if(Barrel3X != 0 && Barrel3Y != 0){allPositions.push_back({Barrel3X,Barrel3Y});} 
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        if(allPositions.size()==0)
        {
            ROS_ERROR("很遗憾，失败了，放弃任务返航！");
            ThrowBottle(1);
            ThrowBottle(3);
            ros::Duration(3).sleep();
            SetPoint(0,0,InitialHeight);
            ros::Duration(10).sleep();
            Land();
            ros::Duration(1000).sleep();
        }
    }
    // 多次运行聚类选择最佳结果
    const int RUNS = 50;
    std::vector<BarrelPosition> bestCenters;
    double bestWCSS = std::numeric_limits<double>::max();
    
    for (int run = 0; run < RUNS; ++run) {
        auto centers = robustKMeans(allPositions, 3, 100);
        
        // 计算类内平方和（只考虑90%内点）
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
    
    // 按x坐标排序输出
    std::sort(bestCenters.begin(), bestCenters.end(), 
        [](const BarrelPosition& a, const BarrelPosition& b) { return a.x < b.x; });

    ROS_INFO("聚类后目标桶坐标:\n");
    ROS_INFO("1号:(%.2f,%.2f)\n",bestCenters[0].x,bestCenters[0].y);
    ROS_INFO("2号:(%.2f,%.2f)\n",bestCenters[1].x,bestCenters[1].y);
    ROS_INFO("3号:(%.2f,%.2f)\n",bestCenters[2].x,bestCenters[2].y);
    Barrel1X_Coord_LD = bestCenters[0].x / 430.857 - 2.228;
    Barrel2X_Coord_LD = bestCenters[1].x / 430.857 - 2.228;
    Barrel3X_Coord_LD = bestCenters[2].x / 430.857 - 2.228;
    Barrel1Y_Coord_LD = (1.393 - bestCenters[0].y / 430.857) + TakeofftoThrow;
    Barrel2Y_Coord_LD = (1.393 - bestCenters[1].y / 430.857) + TakeofftoThrow;
    Barrel3Y_Coord_LD = (1.393 - bestCenters[2].y / 430.857) + TakeofftoThrow;

    ROS_INFO("飞往目标桶坐标如下:\n");
    ROS_INFO("1号:(%.2f,%.2f)\n",Barrel1X_Coord_LD,Barrel1Y_Coord_LD);
    ROS_INFO("2号:(%.2f,%.2f)\n",Barrel2X_Coord_LD,Barrel2Y_Coord_LD);
    ROS_INFO("3号:(%.2f,%.2f)\n",Barrel3X_Coord_LD,Barrel3Y_Coord_LD);

    //投放开始
    ROS_INFO(">>>>>>开始瞄准投放<<<<<<");
    SetPoint(Barrel1X_Coord_LD,Barrel1Y_Coord_LD,InitialHeight);
    ShowPosition(3);
    PID_Init(&pos_pid_xy,Kp_H,Ki_H,Kd_H);
    Positioning(InitialHeight,90,CamX/2,CamY/2,MaxVel_H,0);
    SlowDescend(PX4_Position.x,PX4_Position.y,InitialHeight,ThrowMidHeight);
    Positioning(ThrowMidHeight,90,CamX/2,CamY/2,MaxVel_H,0);
    SlowDescend(PX4_Position.x,PX4_Position.y,ThrowMidHeight,ThrowHeight);
    PID_Init(&pos_pid_xy,Kp_L,Ki_L,Kd_L);
    if(Positioning(ThrowHeight,300,Throw1_X,Throw1_Y,MaxVel_L,1)==false)
    {
        PID_Init(&pos_pid_xy,Kp_H,Ki_H,Kd_H);
        SetPoint(Barrel1X_Coord_LD,Barrel1Y_Coord_LD,ThrowMidHeight);
        Positioning(ThrowMidHeight,90,CamX/2,CamY/2,MaxVel_H,0);
        SlowDescend(PX4_Position.x,PX4_Position.y,ThrowMidHeight,ThrowHeight);
        PID_Init(&pos_pid_xy,Kp_L,Ki_L,Kd_L);
        if(Positioning(ThrowHeight,300,Throw1_X,Throw1_Y,MaxVel_L,1)==false)
        {
            ThrowBottle(1);
            ROS_INFO("实在没瞄准，放弃本次投放，进行下一次投放");
        }
    }
    ros::Duration(1).sleep();
    SetPoint(Barrel2X_Coord_LD,Barrel2Y_Coord_LD,InitialHeight);
    PID_Init(&pos_pid_xy,Kp_H,Ki_H,Kd_H);
    ros::Duration(3).sleep();
    Positioning(InitialHeight,90,CamX/2,CamY/2,MaxVel_H,0);
    SlowDescend(PX4_Position.x,PX4_Position.y,InitialHeight,ThrowMidHeight);
    Positioning(ThrowMidHeight,30,CamX/2,CamY/2,MaxVel_H,0);
    SlowDescend(PX4_Position.x,PX4_Position.y,ThrowMidHeight,ThrowHeight);
    PID_Init(&pos_pid_xy,Kp_L,Ki_L,Kd_L);
    if(Positioning(ThrowHeight,300,Throw2_X,Throw2_Y,MaxVel_L,3)==false)
    {
        PID_Init(&pos_pid_xy,Kp_H,Ki_H,Kd_H);
        SetPoint(Barrel2X_Coord_LD,Barrel2Y_Coord_LD,ThrowMidHeight);
        Positioning(ThrowMidHeight,90,CamX/2,CamY/2,MaxVel_H,0);
        SlowDescend(PX4_Position.x,PX4_Position.y,ThrowMidHeight,ThrowHeight);
        PID_Init(&pos_pid_xy,Kp_L,Ki_L,Kd_L);
        if(Positioning(ThrowHeight,300,Throw2_X,Throw2_Y,MaxVel_L,3)==false)
        {
            ThrowBottle(3);
            ROS_INFO("实在没瞄准，放弃本次投放，进行侦察");
        }
    }
}

