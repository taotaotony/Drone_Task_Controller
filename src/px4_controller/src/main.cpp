#include "main.h"                   //大部分全局变量
#include "WebServer.h"            //PID网络调参函数
#include "communication.h"          //通信相关函数
#include "aim.h"                    //瞄准相关函数
#include "control.h"                //飞控控制指令相关函数
#include "Drone.h"                  //无人机状态机相关函数

// ── 全局变量定义 (main.h 中 extern 声明) ──────
struct Position PX4_Position;              //当前飞机坐标，由回调函数更新
struct Velocity PX4_Velocity;              //当前飞机速度，由回调函数更新
PIDController pos_pid_xy;
double initial_yaw;

Drone drone;                                   // 定义无人机状态机对象
WebServer webserver(drone);                    // 定义网络服务器对象

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"main");
    setlocale(LC_ALL,"");
    ros::Time::init();
    ros::NodeHandle nh;
    /*>>>全局变量赋值<<<*/
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");                             // 解锁服务通信
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");                                 // 切换模式服务通信
    set_position_client = nh.serviceClient<px4_controller::position>("SendPosition");                             // 发送目标点服务通信
    throw_client = nh.serviceClient<px4_controller::throwcmd>("ThrowCmd");                                        // 投放服务通信
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    visual_sub=nh.subscribe("IR",10,visual_cb);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,pos_cb);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,imu_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,vel_cb);
    last_request=ros::Time::now();
    ros::Rate rate(30.0);
    
    webserver.start();
    ros::Duration(5).sleep();
    ThrowBottle(1);
    ConnectPX4();
    // ── 状态机主循环 ──────────────────────────
    while (ros::ok())
    {
        drone.HandleState();    // 每帧分发到当前状态的 Execute*()
        ros::spinOnce();
        rate.sleep();
    }
}


    /*/视觉测试部分
    while(1)
    {
        std::cout<<VisualData.Target1_Exist<<std::endl;
        std::cout<<VisualData.Target2_Exist<<std::endl;
        std::cout<<VisualData.Target3_Exist<<std::endl;
        //计算中心点
        int c1x=(VisualData.Target1_LU_x+VisualData.Target1_RD_x)/2;
        int c1y=(VisualData.Target1_LU_y+VisualData.Target1_RD_y)/2;
        std::cout<<"1桶("<<c1x<<","<<c1y<<")"<<std::endl;
        std::cout<<"1桶RDx:"<<VisualData.Target1_RD_x<<std::endl;
        ros::spinOnce();
        
        ros::Duration(1).sleep();
    }
*/

//投放测试部分


    // ros::Duration(5).sleep();
    // ThrowBottle(1);
    // ros::Duration(5).sleep();
    //ThrowBottle(3);
    // ros::Duration(5).sleep();
    //Positioning();


    


    //降低投放测试
    // SlowDescend(PX4_Position.x,PX4_Position.y,3.5,0.8);
    // ThrowBottle(1);
    // ShowPosition(3);
    // SetPoint(PX4_Position.x,PX4_Position.y,3.5);
    // ShowPosition(10);
    // SlowDescend(PX4_Position.x,PX4_Position.y,3.5,0.8);
    // ThrowBottle(3);
    // ShowPosition(30999);

