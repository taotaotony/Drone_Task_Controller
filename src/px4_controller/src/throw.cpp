#include <ros/ros.h>
#include <px4_controller/throwcmd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <JetsonGPIO.h>

using namespace std;

bool ThrowBottle(px4_controller::throwcmd::Request& req,
                 px4_controller::throwcmd::Response& resp);

class Servo {
private:
    int pin;          // 物理引脚号（BOARD模式）
    int frequency;
    float minDuty;
    float maxDuty;
    GPIO::PWM* pwm;

    // 将角度（0~180）映射为占空比（%）
    float angleToDutyCycle(int angle) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        return minDuty + (maxDuty - minDuty) * (angle / 180.0f);
    }

public:
    // 构造函数：pin为物理引脚号，minPulseUs和maxPulseUs为0°和180°对应的脉宽（微秒）
    Servo(int pin, int minPulseUs, int maxPulseUs, int freq = 50)
        : pin(pin), frequency(freq), pwm(nullptr) {
        float periodUs = 1000000.0f / frequency; // 周期（微秒）
        minDuty = (minPulseUs / periodUs) * 100.0f;
        maxDuty = (maxPulseUs / periodUs) * 100.0f;
    }

    // 初始化GPIO并启动PWM
    bool begin() {
        try {
            GPIO::setup(pin, GPIO::OUT);  // 因为已设置BOARD模式，pin即为物理编号
            pwm = new GPIO::PWM(pin, frequency);
            pwm->start(0);
            return true;
        } catch (const std::exception& e) {
            cerr << "Failed to initialize servo on pin " << pin << ": " << e.what() << endl;
            return false;
        }
    }

    // 设置角度（0~180）
    void setAngle(int angle) {
        if (pwm == nullptr) {
            cerr << "Servo not initialized. Call begin() first." << endl;
            return;
        }
        float duty = angleToDutyCycle(angle);
        pwm->ChangeDutyCycle(duty);
    }

    // 停止PWM并释放资源
    void stop() {
        if (pwm != nullptr) {
            pwm->stop();
            delete pwm;
            pwm = nullptr;
        }
    }

    ~Servo() {
        stop();
    }
};

int main( int argc, char *argv[] ) 
{
    ros::init(argc,argv,"throw");
    setlocale(LC_ALL,"");
    ros::Time::init();
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("ThrowCmd",ThrowBottle);
    ROS_INFO("[Throw] 投放服务成功启动!");
    ros::Rate loop_rate(10);

    // 设置为BOARD模式（物理引脚编号）
    GPIO::setmode(GPIO::BOARD);

    // 创建两个舵机对象
    // 参数: (物理引脚号, 0度脉宽(us), 180度脉宽(us))
    // 常见SG90舵机: 500us ~ 2500us
    // 建议使用支持硬件PWM的引脚：物理32 (PWM0) 和 物理33 (PWM1)
    // 如果你坚持使用15，请自行确认其是否支持硬件PWM
    Servo servo1(15, 500, 2500);  // 物理引脚15（可能不支持硬件PWM）
    Servo servo2(33, 500, 2500);  // 物理引脚33（支持硬件PWM1）

    if (!servo1.begin()) {
        cerr << "Failed to initialize servo1. Exiting." << endl;
        return -1;
    }
    if (!servo2.begin()) {
        cerr << "Failed to initialize servo2. Exiting." << endl;
        return -1;
    }

    cout << "Servo test started (BOARD mode). Press Ctrl+C to exit." << endl;

    try {
        while (true) {
            cout << "Servo1: 0°, Servo2: 180°" << endl;
            servo1.setAngle(0);
            servo2.setAngle(180);
            this_thread::sleep_for(chrono::seconds(2));

            cout << "Servo1: 90°, Servo2: 90°" << endl;
            servo1.setAngle(90);
            servo2.setAngle(90);
            this_thread::sleep_for(chrono::seconds(2));

            cout << "Servo1: 180°, Servo2: 0°" << endl;
            servo1.setAngle(180);
            servo2.setAngle(0);
            this_thread::sleep_for(chrono::seconds(2));
        }
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    servo1.stop();
    servo2.stop();
    GPIO::cleanup();
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
    //SerialSendData(cmd);
    resp.success = true;
    return true;
}
