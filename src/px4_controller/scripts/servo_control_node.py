#!/usr/bin/env python3
import rospy
from px4_controller.srv import throwcmd, throwcmdResponse
import Jetson.GPIO as GPIO
from time import sleep

output_pin = [15, 33]
servo1 = None  # pin 15
servo2 = None  # pin 33
OpenAngle  = 20
CloseAngle = 120

def handle_throw_cmd(req):
    resp = throwcmdResponse()
    try:
        cmd = req.cmd
        if cmd == 1:
            rospy.loginfo("Servo1 action: open/close/open")
            servo1.ChangeDutyCycle(2.5 + OpenAngle * 10 / 180)
            sleep(1)
            servo1.ChangeDutyCycle(2.5 + CloseAngle * 10 / 180)
            sleep(1)
            servo1.ChangeDutyCycle(2.5 + OpenAngle * 10 / 180)
            resp.success = True
        elif cmd == 2:
            rospy.loginfo("Servo2 action: open/close/open")
            servo2.ChangeDutyCycle(2.5 + OpenAngle * 10 / 180)
            sleep(1)
            servo2.ChangeDutyCycle(2.5 + CloseAngle * 10 / 180)
            sleep(1)
            servo2.ChangeDutyCycle(2.5 + OpenAngle * 10 / 180)
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
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(output_pin, GPIO.OUT)
    servo1 = GPIO.PWM(output_pin[0], 50)
    servo2 = GPIO.PWM(output_pin[1], 50)
    servo1.start(0)
    servo2.start(0)
    sleep(1)

    service = rospy.Service("ThrowCmd", throwcmd, handle_throw_cmd)
    rospy.loginfo("[Throw] Service ready. Waiting for commands...")
    rospy.spin()

    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
    rospy.loginfo("Node terminated.")