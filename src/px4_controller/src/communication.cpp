// communication.cpp — 回调函数及全局变量定义
#include "communication.h"

// ── 全局变量定义 ──────────────────────────────
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient set_position_client;
ros::ServiceClient throw_client;
ros::Subscriber state_sub;
ros::Subscriber visual_sub;
ros::Subscriber pos_sub;
ros::Subscriber vel_sub;
ros::Subscriber imu_sub;
ros::Subscriber lidar_sub;

mavros_msgs::State current_state;
sensor_msgs::Imu imu_msg;
bool yaw_initialized = false;
geometry_msgs::TwistStamped vel_msg;

struct VisualData VisualData;

// ── 回调函数实现 ──────────────────────────────
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double bodyX, bodyY;
    ENUToBody(msg->pose.position.x, msg->pose.position.y, bodyX, bodyY);
    PX4_Position.x = bodyX;
    PX4_Position.y = bodyY;
    PX4_Position.z = msg->pose.position.z;
    // 电子围栏功能
    if(Electronic_Fence_ENABLE)
    {
        if(PX4_Position.x > Electronic_Fence_X_MAX || PX4_Position.x < Electronic_Fence_X_Min ||
           PX4_Position.y > Electronic_Fence_Y_MAX || PX4_Position.y < Electronic_Fence_Y_Min)
        {
            ROS_ERROR("[PX4] 电子围栏触发，飞机即将悬停...");
            SetPoint(PX4_Position.x, PX4_Position.y, PX4_Position.z);
            drone.RequestTransition(DroneState_WAITING);
        }
    }
}

void visual_cb(const px4_controller::tbag::ConstPtr& msg)
{
    VisualData.Target1_Exist = msg->Target1_Exist;
    VisualData.Target1_PR    = msg->Target1_PR;
    VisualData.Target1_LU_x  = msg->Target1_LU_x;
    VisualData.Target1_LU_y  = msg->Target1_LU_y;
    VisualData.Target1_RU_x  = msg->Target1_RU_x;
    VisualData.Target1_RU_y  = msg->Target1_RU_y;
    VisualData.Target1_RD_x  = msg->Target1_RD_x;
    VisualData.Target1_RD_y  = msg->Target1_RD_y;
    VisualData.Target1_LD_x  = msg->Target1_LD_x;
    VisualData.Target1_LD_y  = msg->Target1_LD_y;
    VisualData.Target2_Exist = msg->Target2_Exist;
    VisualData.Target2_PR    = msg->Target2_PR;
    VisualData.Target2_LU_x  = msg->Target2_LU_x;
    VisualData.Target2_LU_y  = msg->Target2_LU_y;
    VisualData.Target2_RU_x  = msg->Target2_RU_x;
    VisualData.Target2_RU_y  = msg->Target2_RU_y;
    VisualData.Target2_RD_x  = msg->Target2_RD_x;
    VisualData.Target2_RD_y  = msg->Target2_RD_y;
    VisualData.Target2_LD_x  = msg->Target2_LD_x;
    VisualData.Target2_LD_y  = msg->Target2_LD_y;
    VisualData.Target3_Exist = msg->Target3_Exist;
    VisualData.Target3_PR    = msg->Target3_PR;
    VisualData.Target3_LU_x  = msg->Target3_LU_x;
    VisualData.Target3_LU_y  = msg->Target3_LU_y;
    VisualData.Target3_RU_x  = msg->Target3_RU_x;
    VisualData.Target3_RU_y  = msg->Target3_RU_y;
    VisualData.Target3_RD_x  = msg->Target3_RD_x;
    VisualData.Target3_RD_y  = msg->Target3_RD_y;
    VisualData.Target3_LD_x  = msg->Target3_LD_x;
    VisualData.Target3_LD_y  = msg->Target3_LD_y;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_msg = *msg;
    if (!yaw_initialized)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        initial_yaw = yaw;
        yaw_initialized = true;
        ROS_INFO_STREAM("\033[32m" << "[IMU] Initial Yaw: " << initial_yaw << " radians" << "\033[0m");
    }
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    vel_msg = *msg;
    PX4_Velocity.vx = vel_msg.twist.linear.x;
    PX4_Velocity.vy = vel_msg.twist.linear.y;
    PX4_Velocity.vz = vel_msg.twist.linear.z;
}

void lidar_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("[Lidar] 距离: %.4f 米", msg->range);
}
