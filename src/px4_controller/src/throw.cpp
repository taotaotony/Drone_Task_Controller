#include <ros/ros.h>
#include <px4_controller/throwcmd.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/termios.h>
 
#define DEV_NAME  "/dev/ttyUSB0"  //串口名称

char A[]="0";  //1号到中间
char B[]="1";  //1号投放
char C[]="2";  //2号到中间
char D[]="3";  //2号投放
char E[]="4";  //投放完复位
int fd; //串口状态
bool ThrowBottle(px4_controller::throwcmd::Request& req,
                 px4_controller::throwcmd::Response& resp);

void SerialSendData(int cmd)
{
    switch(cmd)
    {
        case 0:{int r = write(fd, A, sizeof(A));ROS_INFO("投放指令：1号到中间");break;}
        case 1:{int r = write(fd, B, sizeof(B));ROS_INFO("投放指令：1号投放");break;}
        case 2:{int r = write(fd, C, sizeof(C));ROS_INFO("投放指令：2号到中间");break;}
        case 3:{int r = write(fd, D, sizeof(D));ROS_INFO("投放指令：2号投放");break;}
        case 4:{int r = write(fd, E, sizeof(E));ROS_INFO("投放指令：投放完毕复位");break;}
    }
}

int main (int argc, char *argv[])
{
    ros::init(argc,argv,"throw");
    setlocale(LC_ALL,"");
    ros::Time::init();
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("ThrowCmd",ThrowBottle);
    ROS_INFO("投放服务成功启动!");
    ros::Rate loop_rate(10);

	fd = open(DEV_NAME, O_RDWR | O_NOCTTY);  //打开串口
    if(fd < 0) {
        perror(DEV_NAME);
        return -1;
    }
    else
    {
        ROS_INFO("投放串口成功启动！");
    }
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
	return 0;
}

bool ThrowBottle(px4_controller::throwcmd::Request& req,
                 px4_controller::throwcmd::Response& resp)
{
    int cmd=req.cmd;
    SerialSendData(cmd);
    resp.success = true;
    return true;
}
