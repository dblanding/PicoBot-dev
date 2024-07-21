# steertest.py
"""
An exploration of how to optimize turning (in place) with less
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
from parameters import ANGLE_TOL
from bno08x_i2c import *
import time


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

def turn(goal_angle):
    """
    Starting at an initial yaw angle (zero, in this case)
    Turn in place to goal_angle (radians)
    Positive angles are to the left, negative to the right
    """
    done = False
    # get IMU data
    gz, yaw = get_imu_data()

    # steer to goal angle
    yaw_err = yaw - goal_angle
    kp = -(yaw_err * P_GAIN)  # proportional term
    kd = -(gz * D_GAIN)  # derivative term
    ang_spd = kp + kd
    if ang_spd < -MAX_ANG_SPD:
        ang_spd = -MAX_ANG_SPD
    if ang_spd > MAX_ANG_SPD:
        ang_spd = MAX_ANG_SPD
    if abs(gz) < 0.01 and abs(yaw_err) < ANGLE_TOL:
        ang_spd = 0
        done = True
    motors.drive_motors(0, ang_spd)
    ang_deg = yaw*180/pi
    print(ang_spd, gz, ang_deg)
    return done

if __name__ == "__main__":
    try:
        MAX_ANG_SPD = 0.7
        P_GAIN = 8.0  # Proportional Gain
        D_GAIN = 0.5  # Derivative Gain
        while True:
            done = turn(-pi/4)  # 45 deg right
            if done:
                break
            time.sleep(0.1)
        while True:
            done = turn(pi/4)  # 45 deg left
            if done:
                break
            time.sleep(0.1)
        while True:
            done = turn(0)  # back to 0
            if done:
                break
            time.sleep(0.1)

    finally:
        motors.drive_motors(0, 0)