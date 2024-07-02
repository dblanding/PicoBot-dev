# robot.py

"""
MicroPython code for Pico car project
* Raspberry Pi Pico mounted on differential drive car
* 56:1 gear motors with encoders
"""

import encoder_rp2 as encoder
import gc
import math
from machine import I2C, Pin, PWM, UART
from time import sleep
import motors
from odometer import Odometer
from parameters import TICKS_PER_METER, TARGET_TICK_RATE, TURN_SPD, ANGLE_TOL, SPD_GAIN
import struct
from bno055 import *
import VL53L0X

# setup encoders
print("Settint up motor encoders")
enc_b = encoder.Encoder(0, Pin(8))
enc_a = encoder.Encoder(1, Pin(6))

# setup onboard LED
led = machine.Pin("LED", machine.Pin.OUT)

# set up uart0 for communication with BLE UART friend
print("setting up uart0 for accepting tele-op joystick commands")
uart0 = UART(0, 9600)
uart0.init(tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, timeout=10)

# set up uart1 for communication with BLE UART friend
print("Setting up uart1 for sending pose data to laptop")
uart1 = UART(1, 9600)
uart1.init(tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1, timeout=10)


# set up left & right VCSEL TOF distance sensors
def setup_i2c_device(bus_id, sda_pin, scl_pin):
    """Setup a device on an I2C bus.
    There are two available busses: 0 & 1.
    Return I2C object."""
    sda = Pin(sda_pin)
    scl = Pin(scl_pin)

    print("Setting up i2c%s" % bus_id)
    i2c = I2C(id=bus_id, sda=sda, scl=scl)
    print("Device IDs %s on i2c%s" % (i2c.scan(), bus_id))

    return VL53L0X.VL53L0X(i2c)

print("Setting up Left Dist sensor")
tof0 = setup_i2c_device(0, 12, 13)  # Left
tof0.start()
print("Setting up Right Dist sensor")
tof1 = setup_i2c_device(1, 10, 11)  # Right
tof1.start()

# set up bno055 IMU
print("Device bno055 IMU set up")
i2c1 = I2C(1, sda=Pin(10), scl=Pin(11))
imu = BNO055(i2c1)
calibrated = False


yaw_prev = 0
lin_spd = 0
ang_spd = 0
odom = Odometer()
while True:

    # get IMU data
    try:
        heading, *rest = imu.euler()
        # convert from degrees to radians
        yaw = -heading  # * math.pi / 180 
        if yaw != yaw_prev:
            yaw_prev = yaw
    except Exception as e:
        print(e)
        yaw = yaw_prev

    # read distances from sensors
    dist0 = tof0.read()
    dist1 = tof1.read()

    if uart0.any() > 0:
        try:
            # get Bluetooth command
            bytestring = uart0.readline()
            data_type = bytestring[:2].decode()
            bin_value = bytestring[2:14]
            if data_type == '!A':  # accelerometer data
                x, y, z = struct.unpack('3f', bin_value)
                # print(x, y, z)
                lin_spd = y * SPD_GAIN
                ang_spd = -x * SPD_GAIN
        except Exception as e:
            lin_spd, ang_spd = 0, 0
            print(e)

        # send commands to motors
        motors.drive_motors(lin_spd, ang_spd)

        # get latest encoder values
        enc_a_val = enc_a.value()
        enc_b_val = enc_b.value()

        # get current pose
        pose = odom.update(enc_a_val, enc_b_val)

        # send pose data to laptop
        pose_data = list(pose)
        data = pose_data
        data += [yaw, dist0, dist1]
        str_data = ', '.join(str(n) for n in data)
        uart1.write(str_data + "\n")
        print(str_data + "\n")

        led.toggle()

    sleep(0.1)
