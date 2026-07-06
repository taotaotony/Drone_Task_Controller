#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "main.h"
#include "Drone.h"
#include "VisualData.h"

// ── ROS 通信对象 (extern 声明) ────────────────
extern ros::ServiceClient arming_client;
extern ros::ServiceClient set_mode_client;
extern ros::ServiceClient set_position_client;
extern ros::ServiceClient throw_client;
extern ros::Subscriber state_sub;
extern ros::Subscriber visual_sub;
extern ros::Subscriber pos_sub;
extern ros::Subscriber vel_sub;
extern ros::Subscriber imu_sub;
extern ros::Subscriber lidar_sub;

extern mavros_msgs::State current_state;
extern sensor_msgs::Imu imu_msg;
extern bool yaw_initialized;
extern geometry_msgs::TwistStamped vel_msg;

extern Drone drone;

extern struct VisualData VisualData;

// ── 回调函数声明 ──────────────────────────────
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void visual_cb(const px4_controller::tbag::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
void lidar_cb(const sensor_msgs::Range::ConstPtr& msg);

#endif // __COMMUNICATION_H