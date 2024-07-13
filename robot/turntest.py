# turntest.py

import encoder_rp2 as encoder
from math import pi
from machine import I2C, Pin, UART
import motors
from odometer import Odometer
from parameters import ANGLE_TOL
from bno055 import BNO055
from time import sleep

# setup encoders
print("Setting up motor encoders")
enc_b = encoder.Encoder(0, Pin(8))
enc_a = encoder.Encoder(1, Pin(6))

# setup onboard LED
led = machine.Pin("LED", machine.Pin.OUT)

# instantiate odometer
odom = Odometer()

# set up IMU on i2c1
i2c1 = I2C(1, sda=Pin(14), scl=Pin(15))
print(i2c1.scan())
imu = BNO055(i2c1)

while True:

    # get IMU data
    *_, gz = imu.gyro()  # deg/sec (+ turning left)
    heading, *_ = imu.euler()  # deg
    heading = - heading  # match sense of pose angle
    yaw = heading * pi / 180  # convert to radians

    # match pose angle numerically
    if yaw < -pi:
        yaw += 2 * pi

    # initial 90 deg turn to right
    goal_angle = -pi/2
    lin_spd = 0
    ang_spd = -0.6
    motors.drive_motors(lin_spd, ang_spd)
    if yaw > (goal_angle + ANGLE_TOL):
        motors.drive_motors(0, -0.6)
    elif yaw < (goal_angle - ANGLE_TOL):
        motors.drive_motors(0, 0.6)
    else:
        motors.drive_motors(0, 0)

    sleep(0.1)
