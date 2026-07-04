#!/usr/bin/env python3
import sys
from time import sleep

import rospy
from px4_controller.srv import throwcmd, throwcmdResponse

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError:
        SMBus = None


controller = None
servo_channels = [1, 2]
open_angle = 20
close_angle = 120
move_delay = 1.0


def parse_int_param(value, name):
    try:
        if isinstance(value, str):
            return int(value, 0)
        return int(value)
    except (TypeError, ValueError):
        raise ValueError("{} must be an integer, got {!r}".format(name, value))


def parse_servo_channels(value):
    if isinstance(value, str):
        value = [item.strip() for item in value.split(",") if item.strip()]

    if len(value) < 2:
        raise ValueError("~servo_channels must contain at least two channels")

    return [parse_int_param(item, "~servo_channels") for item in value[:2]]


class I2CServoController:
    """Send servo angle commands to the external control board over I2C."""

    def __init__(self, bus_id, address, command_register, retries, retry_delay):
        if SMBus is None:
            raise RuntimeError(
                "Python I2C module not found. Install python3-smbus or smbus2."
            )

        self.bus_id = bus_id
        self.address = address
        self.command_register = command_register
        self.retries = max(1, retries)
        self.retry_delay = max(0.0, retry_delay)
        self.bus = SMBus(bus_id)

    @staticmethod
    def clamp_angle(angle):
        return max(0, min(180, int(round(angle))))

    def set_angle(self, channel, angle):
        channel = parse_int_param(channel, "servo channel")
        angle = self.clamp_angle(angle)
        payload = [channel & 0xFF, angle & 0xFF]
        last_error = None

        for attempt in range(1, self.retries + 1):
            try:
                self.bus.write_i2c_block_data(
                    self.address, self.command_register, payload
                )
                rospy.logdebug(
                    "[Throw] I2C bus=%d addr=0x%02X reg=0x%02X channel=%d angle=%d",
                    self.bus_id,
                    self.address,
                    self.command_register,
                    channel,
                    angle,
                )
                return
            except OSError as exc:
                last_error = exc
                if attempt < self.retries:
                    rospy.logwarn(
                        "[Throw] I2C write failed (%s), retry %d/%d",
                        exc,
                        attempt,
                        self.retries,
                    )
                    sleep(self.retry_delay)

        raise RuntimeError(
            "I2C write failed after {} attempt(s): {}".format(
                self.retries, last_error
            )
        )

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None


def throw_with_servo(channel):
    controller.set_angle(channel, open_angle)
    sleep(move_delay)
    controller.set_angle(channel, close_angle)
    sleep(move_delay)
    controller.set_angle(channel, open_angle)


def handle_throw_cmd(req):
    resp = throwcmdResponse()
    try:
        cmd = req.cmd
        if cmd == 1:
            rospy.loginfo("[Throw] Start throw with servo 1")
            throw_with_servo(servo_channels[0])
            resp.success = True
        elif cmd == 2:
            rospy.loginfo("[Throw] Start throw with servo 2")
            throw_with_servo(servo_channels[1])
            resp.success = True
        else:
            rospy.logwarn("[Throw] Unknown command: %s", cmd)
            resp.success = False
    except Exception as exc:
        rospy.logerr("[Throw] Service error: %s", exc)
        resp.success = False
    return resp


def shutdown():
    if controller is not None:
        controller.close()


if __name__ == "__main__":
    rospy.init_node("servo_control_node", anonymous=False)

    try:
        bus_id = parse_int_param(rospy.get_param("~i2c_bus", 1), "~i2c_bus")
        address = parse_int_param(rospy.get_param("~i2c_address", "0x08"), "~i2c_address")
        command_register = parse_int_param(
            rospy.get_param("~command_register", "0x01"), "~command_register"
        )
        servo_channels = parse_servo_channels(
            rospy.get_param("~servo_channels", [1, 2])
        )
        open_angle = parse_int_param(rospy.get_param("~open_angle", 20), "~open_angle")
        close_angle = parse_int_param(
            rospy.get_param("~close_angle", 120), "~close_angle"
        )
        move_delay = float(rospy.get_param("~move_delay", 1.0))
        retries = parse_int_param(rospy.get_param("~i2c_retries", 3), "~i2c_retries")
        retry_delay = float(rospy.get_param("~i2c_retry_delay", 0.05))

        controller = I2CServoController(
            bus_id=bus_id,
            address=address,
            command_register=command_register,
            retries=retries,
            retry_delay=retry_delay,
        )
    except Exception as exc:
        rospy.logfatal("[Throw] Failed to initialize I2C servo controller: %s", exc)
        sys.exit(1)

    rospy.on_shutdown(shutdown)
    service = rospy.Service("ThrowCmd", throwcmd, handle_throw_cmd)
    rospy.loginfo(
        "[Throw] Service ready. I2C bus=%d addr=0x%02X reg=0x%02X channels=%s",
        controller.bus_id,
        controller.address,
        controller.command_register,
        servo_channels,
    )
    rospy.spin()
