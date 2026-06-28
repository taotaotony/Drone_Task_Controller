#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_controller/position.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>

ros::Publisher local_pose_pub;
ros::Publisher vel_pub;
geometry_msgs::PoseStamped pose; //ENU
//geometry_msgs::Twist vel_msg;//全局速度
mavros_msgs::PositionTarget vel_msg;

int pub_mode=0;//0为发布坐标点，1为发布速度
double initial_yaw;
bool SetPosition(px4_controller::position::Request& req,
                 px4_controller::position::Response& resp)
{
    pub_mode=req.mode;
    pose.pose.position.x = req.x;
    pose.pose.position.y = req.y;
    pose.pose.position.z = req.z;
    pose.pose.orientation.w=req.qw;
    pose.pose.orientation.x=req.qx;
    pose.pose.orientation.y=req.qy;
    pose.pose.orientation.z=req.qz;
    vel_msg.velocity.x = req.vx;
    vel_msg.velocity.y = req.vy;
    vel_msg.velocity.z = 0;
    vel_msg.position.z = req.z;
    initial_yaw=req.initial_yaw;
    ROS_INFO("发布ENU-Position: %f %f %f,Vel %f %f ",req.x,req.y,req.z,req.vx,req.vy);
    resp.success = true;
    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"SetPoint");
    setlocale(LC_ALL,"");

    ros::Time::init();
    ros::NodeHandle nh;
    local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
			("/mavros/setpoint_position/local",10);
    vel_pub = nh.advertise<mavros_msgs::PositionTarget>
			("/mavros/setpoint_raw/local",10);
    ros::ServiceServer server = nh.advertiseService("SendPosition",SetPosition);
    ROS_INFO("位置速度发布服务成功启动!");
    ros::Rate loop_rate(10);
    vel_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;  //Attention!!! NED  not ENU
    vel_msg.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_PX | 
        mavros_msgs::PositionTarget::IGNORE_PY | //Attention!!! Do not IGNORE VZ
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    //vel_msg.yaw_rate=0.0;
    //vel_msg.yaw=initial_yaw;
    while(ros::ok())
    {
        if(pub_mode==0)//pos
        {
            local_pose_pub.publish(pose);
        }
        else//mode==1 vel
        {
            vel_msg.header.stamp = ros::Time::now();
            vel_pub.publish(vel_msg);
        }
        //ROS_INFO("pub");
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
