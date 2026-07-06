#include "main.h"                   //大部分全局变量
#include "WebServer.h"              //PID网络调参函数
#include "TelemetryBroadcaster.h"   //UDP遥测广播
#include "communication.h"          //通信相关函数
#include "aim.h"                    //瞄准相关函数
#include "control.h"                //飞控控制指令相关函数
#include "Drone.h"                  //无人机状态机相关函数
#include "calibration.h"            //参数标定器
// ── 全局变量定义 (main.h 中 extern 声明) ──────
struct Position PX4_Position;              //当前飞机坐标，由回调函数更新
struct Velocity PX4_Velocity;              //当前飞机速度，由回调函数更新
PIDController pos_pid_xy;
double initial_yaw;

Drone drone;                                   // 定义无人机状态机对象
WebServer webserver(drone);                    // 定义网络服务器对象
TelemetryBroadcaster telemetry_broadcaster;    // UDP 遥测广播对象

HeightCalibration calib;

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"main");
    setlocale(LC_ALL,"");
    ros::Time::init();
    ros::NodeHandle nh;

    /*>>>外部参数<<<*/
    bool InGame;
    nh.param<bool>("InGame", InGame, true);   // 比赛模式，默认启动
    drone.setgamemode(InGame);
    // 加载标定参数
    std::string package_path = ros::package::getPath("px4_controller");
    std::string file_path = package_path + "/config/calibration_data.txt";
    if (loadCalibrationFromFile(file_path, calib)) 
    {
        ROS_INFO_STREAM("\033[32m" << "[Calibration] 标定参数读取成功!" << "\033[0m");
    }
    else
    {
        ROS_ERROR("[Calibration] 标定参数读取错误!!!");
    }
    /*>>>全局变量赋值<<<*/
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");                             // 解锁服务通信
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");                                 // 切换模式服务通信
    set_position_client = nh.serviceClient<px4_controller::position>("SendPosition");                             // 发送目标点服务通信
    throw_client = nh.serviceClient<px4_controller::throwcmd>("ThrowCmd");                                        // 投放服务通信
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    visual_sub=nh.subscribe("IR",1,visual_cb);////队列是一
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,pos_cb);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,imu_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,vel_cb);
    lidar_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub",10,lidar_cb);
    last_request=ros::Time::now();
    ros::Rate rate(40.0);
    
    webserver.start();
    telemetry_broadcaster.start();

    ConnectPX4();
    // ── 状态机主循环 ──────────────────────────
    while (ros::ok())
    {
        drone.HandleState();    // 每帧分发到当前状态的 Execute*()
        ros::spinOnce();
        rate.sleep();
    }
}
