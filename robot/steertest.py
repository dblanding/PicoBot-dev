# steertest.py
"""
An exploration of how to optimize turning (in place) with reduced
oscillation and hunting by using a PID loop within bang-bang limits.

When turning in place (linear_speed = 0), the P-gain needs to be
large enough to push past the static friction of the motors when
trying to reach the goal_angle from just slightly beyond ANGLE_TOL.
The problem is that this required P-gain would be monstrously large
for larger values of yaw_error. To address this issue, the steering
correction is bounded to be within MAX_ANG_SPD limits.

Incidentally, this is only a problem when turning in place.
When driving at a finite linear speed, there is no static friction
because the motors are already turning, so smaller gain values
will work fine when steering along a track toward a goal_angle.
"""
from math import pi
from machine import I2C, Pin, UART
import motors
from parameters import ANGLE_TOL, P_TURN_GAIN, D_TURN_GAIN, MAX_ANG_SPD
from bno08x_i2c import *
import time

# setup onboard LED
led = machine.Pin("LED", machine.Pin.OUT)

# set up BNO08x IMU on i2c1
i2c1 = I2C(1, sda=Pin(14), scl=Pin(15), freq=100000, timeout=200000 )
print("I2C Device found at address : ",i2c1.scan(),"\n")
bno = BNO08X_I2C(i2c1, debug=False)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
#bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)


def get_imu_data():
    """return gyro_z and yaw data from IMU"""
    *_, gz = bno.gyro  # rad/sec (+ turning left)
    *_, heading = bno.euler  # deg
    yaw = heading * pi / 180  # convert to radians
    # match pose angle numerically
    if yaw < -pi:
        yaw += 2 * pi
    return gz, yaw

def turn(goal_angle, gz, yaw):
    """
    Return ang_spd needed to drive motors in order to
    turn in place to goal_angle (radians).
    Positive angles are to the left, negative to the right
    """
    # calculate proper ang_spd to steer to goal_angle
    yaw_err = yaw - goal_angle
    p = -(yaw_err * P_TURN_GAIN)  # proportional term
    d = -(gz * D_TURN_GAIN)  # derivative term
    ang_spd = p + d
    
    # limit value of ang_spd
    if ang_spd < -MAX_ANG_SPD:
        ang_spd = -MAX_ANG_SPD
    if ang_spd > MAX_ANG_SPD:
        ang_spd = MAX_ANG_SPD
    
    # check if turn is complete
    if abs(gz) < 0.01 and abs(yaw_err) < ANGLE_TOL:
        ang_spd = 0
    # give an extra boost if needed to break past static friction
    elif abs(gz) < 0.01 and abs(yaw_err) < 3 * ANGLE_TOL:
        ang_spd *= 2
        print("needed a boost!")
    
    return ang_spd

if __name__ == "__main__":
    try:
        goal_angle = -pi/2  # turn to face -Y direction
        while True:
            gz, yaw = get_imu_data()
            ang_spd = turn(goal_angle, gz, yaw)
            motors.drive_motors(0, ang_spd)
            # when turn is complete
            if ang_spd == 0:
                break
            led.toggle()
            time.sleep(0.1)
        goal_angle = pi/2  # turn to face -Y direction
        while True:
            gz, yaw = get_imu_data()
            ang_spd = turn(goal_angle, gz, yaw)
            motors.drive_motors(0, ang_spd)
            # when turn is complete
            if ang_spd == 0:
                break
            led.toggle()
            time.sleep(0.1)
        goal_angle = 0  # turn to face -Y direction
        while True:
            gz, yaw = get_imu_data()
            ang_spd = turn(goal_angle, gz, yaw)
            motors.drive_motors(0, ang_spd)
            # when turn is complete
            if ang_spd == 0:
                break
            led.toggle()
            time.sleep(0.1)

    finally:
        motors.drive_motors(0, 0)