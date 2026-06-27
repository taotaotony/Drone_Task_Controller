from pymavlink import mavutil
mav = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
mav.mav.command_long_send(
    target_system=1, target_component=1,
    command=183, confirmation=0,
    param1=1, param2=1500, param3=0, param4=0, param5=0, param6=0, param7=0
)