#!/usr/bin/env python3
import threading
from time import sleep

import rospy
from px4_controller.srv import throwcmd, throwcmdResponse

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
PRESCALE = 0xFE

MODE1_RESTART = 0x80
MODE1_SLEEP = 0x10
MODE1_AUTO_INCREMENT = 0x20
MODE2_OUTDRV = 0x04

DEFAULT_I2C_BUS = 1
DEFAULT_PCA9685_ADDRESS = 0x40
DEFAULT_FREQUENCY_HZ = 50
DEFAULT_MIN_PULSE_US = 500
DEFAULT_MAX_PULSE_US = 2500
DEFAULT_OPEN_ANGLE = 20
DEFAULT_CLOSE_ANGLE = 120
DEFAULT_ACTION_DELAY_SEC = 1.0

servo_controller = None


def parse_int_param(value):
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class PCA9685ServoController:
    def __init__(self):
        self.bus_id = parse_int_param(rospy.get_param("~i2c_bus", DEFAULT_I2C_BUS))
        self.address = parse_int_param(
            rospy.get_param("~i2c_address", DEFAULT_PCA9685_ADDRESS)
        )
        self.frequency_hz = int(rospy.get_param("~frequency_hz", DEFAULT_FREQUENCY_HZ))
        self.min_pulse_us = int(
            rospy.get_param("~min_pulse_us", DEFAULT_MIN_PULSE_US)
        )
        self.max_pulse_us = int(
            rospy.get_param("~max_pulse_us", DEFAULT_MAX_PULSE_US)
        )
        self.open_angle = int(rospy.get_param("~open_angle", DEFAULT_OPEN_ANGLE))
        self.close_angle = int(rospy.get_param("~close_angle", DEFAULT_CLOSE_ANGLE))
        self.action_delay_sec = float(
            rospy.get_param("~action_delay_sec", DEFAULT_ACTION_DELAY_SEC)
        )
        self.command_channels = self._load_command_channels()
        self._validate_config()
        self.lock = threading.Lock()
        self.bus = SMBus(self.bus_id)

        self._initialize_pca9685()

        rospy.loginfo(
            "[Throw] PCA9685 ready: bus=%d address=0x%02X frequency=%dHz",
            self.bus_id,
            self.address,
            self.frequency_hz,
        )

    def _load_command_channels(self):
        channels = rospy.get_param("~command_channels", [])
        return [parse_int_param(channel) for channel in channels]

    def _validate_config(self):
        if self.frequency_hz <= 0:
            raise ValueError("frequency_hz must be greater than 0")
        if self.min_pulse_us >= self.max_pulse_us:
            raise ValueError("min_pulse_us must be smaller than max_pulse_us")
        if self.action_delay_sec < 0:
            raise ValueError("action_delay_sec must not be negative")
        for channel in self.command_channels:
            if channel < 0 or channel > 15:
                raise ValueError("command_channels must contain values from 0 to 15")

    def _initialize_pca9685(self):
        self.bus.write_byte_data(self.address, MODE1, 0x00)
        sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, MODE1_AUTO_INCREMENT)
        self.bus.write_byte_data(self.address, MODE2, MODE2_OUTDRV)
        self._set_pwm_frequency(self.frequency_hz)
        self.release_all()

    def _set_pwm_frequency(self, frequency_hz):
        prescale_value = int(round(25000000.0 / (4096 * frequency_hz)) - 1)
        if prescale_value < 3 or prescale_value > 255:
            raise ValueError(
                "frequency_hz is outside PCA9685 prescale range: {}".format(
                    frequency_hz
                )
            )
        old_mode = self.bus.read_byte_data(self.address, MODE1)
        sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP

        self.bus.write_byte_data(self.address, MODE1, sleep_mode)
        self.bus.write_byte_data(self.address, PRESCALE, prescale_value)
        self.bus.write_byte_data(self.address, MODE1, old_mode)
        sleep(0.005)
        self.bus.write_byte_data(
            self.address, MODE1, old_mode | MODE1_RESTART | MODE1_AUTO_INCREMENT
        )

    def throwbottle(self, cmd):
        channel = self._command_to_channel(cmd)
        rospy.loginfo("[Throw] cmd=%d -> PCA9685 channel=%d", cmd, channel)

        with self.lock:
            self.set_servo_angle(channel, self.open_angle)
            sleep(self.action_delay_sec)
            self.set_servo_angle(channel, self.close_angle)
            sleep(self.action_delay_sec)
            self.set_servo_angle(channel, self.open_angle)

        return True

    def _command_to_channel(self, cmd):
        if self.command_channels:
            index = cmd - 1
            if index < 0 or index >= len(self.command_channels):
                raise ValueError("Unsupported throw command: {}".format(cmd))
            channel = self.command_channels[index]
        else:
            channel = cmd - 1

        if channel < 0 or channel > 15:
            raise ValueError("PCA9685 channel must be in range 0..15")
        return channel

    def set_servo_angle(self, channel, angle):
        angle = clamp(angle, 0, 180)
        pulse_us = self.min_pulse_us + (
            (self.max_pulse_us - self.min_pulse_us) * angle / 180.0
        )
        self.set_servo_pulse(channel, pulse_us)

    def set_servo_pulse(self, channel, pulse_us):
        pulse_us = clamp(pulse_us, self.min_pulse_us, self.max_pulse_us)
        ticks = int(round(pulse_us * self.frequency_hz * 4096 / 1000000.0))
        ticks = clamp(ticks, 0, 4095)
        self._set_pwm(channel, 0, ticks)

    def _set_pwm(self, channel, on_tick, off_tick):
        register = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(
            self.address,
            register,
            [
                on_tick & 0xFF,
                on_tick >> 8,
                off_tick & 0xFF,
                off_tick >> 8,
            ],
        )

    def _release_pwm(self, channel):
        register = LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.address, register, [0, 0, 0, 0x10])

    def release_all(self):
        for channel in range(16):
            self._release_pwm(channel)

    def close(self):
        if self.bus is not None:
            self.release_all()
            self.bus.close()
            self.bus = None


def throwbottle(cmd):
    if servo_controller is None:
        raise RuntimeError("PCA9685 controller is not initialized")
    return servo_controller.throwbottle(cmd)


def handle_throw_cmd(req):
    resp = throwcmdResponse()
    try:
        cmd = int(req.cmd)
        resp.success = throwbottle(cmd)
    except Exception as error:
        rospy.logerr("[Throw] Service error: {}".format(error))
        resp.success = False
    return resp


if __name__ == "__main__":
    rospy.init_node("servo_control_node", anonymous=False)
    servo_controller = PCA9685ServoController()

    service = rospy.Service("ThrowCmd", throwcmd, handle_throw_cmd)
    rospy.loginfo("[Throw] Service ready. Waiting for commands...")

    try:
        rospy.spin()
    finally:
        servo_controller.close()
        rospy.loginfo("[Throw] Node terminated.")
