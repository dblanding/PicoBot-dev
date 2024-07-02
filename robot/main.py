"""
MicroPython code for Pico car project w/ 2 BLE UART Friend modules
* Tele-op driving commands come in on uart0
* Pose data sent out to laptop on uart1
* Raspberry Pi Pico mounted on differential drive car
* 56:1 gear motors with encoders
* 2 VL53L0X tof distance sensors
* BNO055 IMU
"""
import asyncio
import encoder_rp2 as encoder
import gc
import json
import math
from machine import I2C, Pin, UART
import motors
from odometer import Odometer
from parameters import SPD_GAIN
import struct
from bno055 import BNO055
import VL53L0X

import arena
# set up uart0 for communication with BLE UART friend
print("setting up uart0 for accepting tele-op joystick commands")
uart0 = UART(0, 9600)
uart0.init(bits=8, parity=None, stop=1, timeout=10)

# set up uart1 for communication with BLE UART friend
print("Setting up uart1 for sending pose data to laptop")
uart1 = UART(1, 9600)
uart1.init(tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1, timeout=10)

# setup encoders
print("Settint up motor encoders")
enc_b = encoder.Encoder(0, Pin(8))
enc_a = encoder.Encoder(1, Pin(6))

# setup onboard LED
led = machine.Pin("LED", machine.Pin.OUT)

# set up left VCSEL TOF distance sensor on I2C0
i2c0 = I2C(0, sda=Pin(12), scl=Pin(13))
tof0 = VL53L0X.VL53L0X(i2c0)
tof0.start()

# set up right VCSEL TOF distance sensor on I2C1
i2c1 = I2C(1, sda=Pin(10), scl=Pin(11))
tof1 = VL53L0X.VL53L0X(i2c1)
tof1.start()

# set up bno055 IMU on I2C1
imu = BNO055(i2c1)

# instantiate odometer
odom = Odometer()


def send_json(data):
    uart1.write((json.dumps(data) + "\n").encode())

def read_json():
    try:
        data = uart1.readline()
        decoded = data.decode()
        return json.loads(decoded)
    except (UnicodeError, ValueError):
        print("Invalid data")
        return None

def send_poses(pose_list):
    send_json({
        "poses": pose_list,
    })


class Robot():
    def __init__(self):

        # set up some starting values
        self.yaw_prev = 0
        self.lin_spd = 0
        self.ang_spd = 0
        self.prev_pose = (0, 0, 0)
        self.pose_list = []

    async def main(self):
        try:
            while True:
                # get IMU data
                heading, *rest = imu.euler()
                # convert from degrees to radians
                self.yaw = -heading  # * math.pi / 180 
                if self.yaw != self.yaw_prev:
                    self.yaw_prev = self.yaw

                # read distances from sensors
                dist0 = tof0.read()
                dist1 = tof1.read()

                # Check for tele-op commands
                if uart0.any() > 0:
                    try:
                        # get Bluetooth command
                        bytestring = uart0.readline()
                        data_type = bytestring[:2].decode()
                        bin_value = bytestring[2:14]
                        if data_type == '!A':  # accelerometer data
                            x, y, z = struct.unpack('3f', bin_value)
                            # print(x, y, z)
                            self.lin_spd = y * SPD_GAIN
                            self.ang_spd = -x * SPD_GAIN
                    except Exception as e:
                        self.lin_spd, self.ang_spd = 0, 0
                        print(e)

                    # send commands to motors
                    motors.drive_motors(self.lin_spd, self.ang_spd)

                # get current pose
                pose = odom.update(enc_a.value(), enc_b.value())
                print(pose)

                # send pose data to laptop
                if pose != (0, 0, 0):
                    self.pose_list.append(pose)
                    send_poses(self.pose_list)
                    if len(self.pose_list) >= 8:
                        _ = self.pose_list.pop(0)

                led.toggle()
                await asyncio.sleep(0.1)
        finally:
            motors.drive_motors(0, 0)



async def command_handler(robot):
    print("Starting handler")
    robot_task = asyncio.create_task(robot.main())
    while True:
        if uart1.any():
            request = read_json()
            if not request:
                continue
            print("Received: ", request)
            if request["command"] == "arena":
                send_json({"arena": arena.boundary_lines,})

        await asyncio.sleep(0.1)


robot = Robot()
asyncio.run(command_handler(robot))
