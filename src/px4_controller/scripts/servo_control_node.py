#!/usr/bin/env python3
import rospy
from px4_controller.srv import throwcmd, throwcmdResponse
from adafruit_servokit import ServoKit  #引入PCA9685库
from time import sleep

Angle_Open = 60
Angle_Close = 120
kit = ServoKit(channels=16)   // 舵机控制板PCA9685对象

def handle_throw_cmd(req):
    resp = throwcmdResponse()
    try:
        cmd = req.cmd
        if cmd == 1:
            rospy.loginfo("[Throw] Servo1 work!")
            kit.servo[0].angle = Angle_Close
            sleep(1)
            kit.servo[0].angle = Angle_Open
            sleep(1)
            kit.servo[0].angle = Angle_Close
            resp.success = True
        elif cmd == 2:
            rospy.loginfo("[Throw] Servo2 work!")
            kit.servo[1].angle = Angle_Close
            sleep(1)
            kit.servo[1].angle = Angle_Open
            sleep(1)
            kit.servo[1].angle = Angle_Close
            resp.success = True
        else:
            rospy.logwarn("Unknown command: {}".format(cmd))
            resp.success = False
    except Exception as e:
        rospy.logerr("Service error: {}".format(e))
        resp.success = False
    return resp

if __name__ == "__main__":
    rospy.init_node("servo_control_node", anonymous=False)
    
    sleep(1)

    service = rospy.Service("ThrowCmd", throwcmd, handle_throw_cmd)
    rospy.loginfo("[Throw] Service ready. Waiting for commands...")
    rospy.spin()
    rospy.loginfo("[Throw] Node terminated.")