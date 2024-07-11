# main.py
"""
MicroPython code for PicoBot project w/ 2 BLE UART Friend modules
* Tele-op driving commands come in on uart0
* Robot data sent out to laptop on uart1
* Raspberry Pi Pico mounted on differential drive robot
* 56:1 gear motors with encoders
* 3 VL53L0X tof distance sensors
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
from parameters import JS_GAIN
import struct
from bno055 import BNO055
import VL53L0X
import arena

# set up uart0 for communication with BLE UART friend
print("setting up uart0 for accepting tele-op joystick commands")
uart0 = UART(0, 9600)
uart0.init(bits=8, parity=None, stop=1, timeout=10)

# set up uart1 for communication with BLE UART friend
print("Setting up uart1 for sending robot data to laptop")
uart1 = UART(1, 9600)
uart1.init(tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1, timeout=10)

# setup encoders
print("Setting up motor encoders")
enc_b = encoder.Encoder(0, Pin(8))
enc_a = encoder.Encoder(1, Pin(6))

# setup onboard LED
led = machine.Pin("LED", machine.Pin.OUT)

# instantiate odometer
odom = Odometer()

# set up multiplexer on i2c0
i2c0 = I2C(0, sda=Pin(12), scl=Pin(13))

# set up IMU on i2c1
i2c1 = I2C(1, sda=Pin(14), scl=Pin(15))
print(i2c1.scan())
imu = BNO055(i2c1)

# set up forward looking dist sensor on i2c1
tof1 = VL53L0X.VL53L0X(i2c1)
tof1.start()

def get_dist(channel):
    """
    return dist (mm) from tof sensor on mux
        channel = b'\x02'  # left sensor on ch 1
        channel = b'\x04'  # right sensor on ch 2
    see: https://github.com/mcauser/micropython-tca9548a
    """
    i2c0.writeto(0x70, channel)
    tof = VL53L0X.VL53L0X(i2c0)
    tof.start()
    dist = tof.read()
    tof.stop()
    return dist

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


class Robot():
    def __init__(self):

        # set up some starting values
        self.lin_spd = 0
        self.ang_spd = 0

    async def main(self):
        try:
            while True:

                # get IMU data
                heading, *rest = imu.euler()
                heading = - heading  # match sense of pose angle
                yaw = heading * math.pi / 180  # convert to radians

                # match pose angle numerically
                if yaw < -math.pi:
                    yaw += 2 * math.pi

                # read distances from VCSEL sensors
                dist_L = get_dist(b'\x02')
                dist_R = get_dist(b'\x04')
                dist_F = tof1.read()
    
                # Check for tele-op commands
                if uart0.any() > 0:
                    try:
                        # get Bluetooth command
                        bytestring = uart0.readline()
                        data_type = bytestring[:2].decode()
                        bin_value = bytestring[2:14]
                        if data_type == '!A':  # accelerometer data
                            x, y, z = struct.unpack('3f', bin_value)
                            self.lin_spd = y * JS_GAIN
                            self.ang_spd = -x * JS_GAIN
                    except Exception as e:
                        self.lin_spd, self.ang_spd = 0, 0
                        print(e)

                    # send commands to motors
                    motors.drive_motors(self.lin_spd, self.ang_spd)

                # get current pose
                pose = odom.update(enc_a.value(), enc_b.value())
                print(pose, yaw, dist_L, dist_R, dist_F)

                # send robot data to laptop
                if pose != (0, 0, 0):
                    send_json({
                        "pose": list(pose),
                        "yaw": yaw,
                        "dist_L": dist_L,
                        "dist_R": dist_R,
                        "dist_F": dist_F,
                        })

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
