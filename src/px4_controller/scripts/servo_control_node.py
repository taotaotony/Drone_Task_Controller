#!/usr/bin/env python3
import rospy
from px4_controller.srv import throwcmd, throwcmdResponse

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


DEFAULT_I2C_BUS = 1
DEFAULT_I2C_ADDRESS = 0x08
DEFAULT_THROW_COMMAND = 0x01

i2c_controller = None


def parse_int_param(value):
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


class I2CThrowController:
    def __init__(self):
        self.bus_id = parse_int_param(rospy.get_param("~i2c_bus", DEFAULT_I2C_BUS))
        self.address = parse_int_param(
            rospy.get_param("~i2c_address", DEFAULT_I2C_ADDRESS)
        )
        self.throw_command = parse_int_param(
            rospy.get_param("~throw_command", DEFAULT_THROW_COMMAND)
        )
        self.write_mode = rospy.get_param("~i2c_write_mode", "byte")
        self.retries = int(rospy.get_param("~i2c_retries", 2))
        self.bus = SMBus(self.bus_id)

        rospy.loginfo(
            "[Throw] I2C ready: bus=%d address=0x%02X mode=%s throw_command=0x%02X",
            self.bus_id,
            self.address,
            self.write_mode,
            self.throw_command,
        )

    def throwbottle(self, cmd):
        if cmd < 0 or cmd > 255:
            raise ValueError("I2C command must be in range 0..255")

        last_error = None
        for attempt in range(self.retries + 1):
            try:
                self._write_throw_command(cmd)
                return True
            except OSError as error:
                last_error = error
                rospy.logwarn(
                    "[Throw] I2C write failed on attempt %d/%d: %s",
                    attempt + 1,
                    self.retries + 1,
                    error,
                )

        raise last_error

    def _write_throw_command(self, cmd):
        if self.write_mode == "byte":
            self.bus.write_byte(self.address, cmd)
        elif self.write_mode == "byte_data":
            self.bus.write_byte_data(self.address, self.throw_command, cmd)
        elif self.write_mode == "block":
            self.bus.write_i2c_block_data(self.address, self.throw_command, [cmd])
        else:
            raise ValueError("Unsupported I2C write mode: {}".format(self.write_mode))

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None


def throwbottle(cmd):
    if i2c_controller is None:
        raise RuntimeError("I2C controller is not initialized")
    return i2c_controller.throwbottle(cmd)


def handle_throw_cmd(req):
    resp = throwcmdResponse()
    try:
        cmd = int(req.cmd)
        rospy.loginfo("[Throw] Send throwbottle command over I2C: %d", cmd)
        resp.success = throwbottle(cmd)
    except Exception as error:
        rospy.logerr("[Throw] Service error: {}".format(error))
        resp.success = False
    return resp


if __name__ == "__main__":
    rospy.init_node("servo_control_node", anonymous=False)
    i2c_controller = I2CThrowController()

    service = rospy.Service("ThrowCmd", throwcmd, handle_throw_cmd)
    rospy.loginfo("[Throw] Service ready. Waiting for commands...")

    try:
        rospy.spin()
    finally:
        i2c_controller.close()
        rospy.loginfo("[Throw] Node terminated.")
