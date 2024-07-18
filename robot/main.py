# main.py
"""
MicroPython code for PicoBot project w/ 2 BLE UART Friend modules
* Tele-op driving commands come in on uart0
* Robot data sent out to laptop on uart1
* Raspberry Pi Pico mounted on differential drive robot
* 56:1 gear motors with encoders
* 3 VL53L0X tof distance sensors
* BNO08x IMU

Notes on angular units:
Use radians: pose_angle (from odometer) and yaw (from IMU)
0 at the X axis (initially straight ahead), pos CCW / neg CW
"""
import asyncio
import encoder_rp2 as encoder
import gc
import json
from math import pi
from machine import I2C, Pin, UART
import motors
from odometer import Odometer
from parameters import JS_GAIN, ANGLE_TOL, SWATH_PITCH
import struct
from bno08x_i2c import *
import VL53L0X
import arena

D_GAIN = 0.286  # Gain of Derivative feedback term

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

# set up BNO08x IMU on i2c1
i2c1 = I2C(1, sda=Pin(14), scl=Pin(15), freq=100000, timeout=200000 )
print("I2C Device found at address : ",i2c1.scan(),"\n")
bno = BNO08X_I2C(i2c1, debug=False)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
#bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

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

def get_imu_data():
    """return yaw and gyro_z data from IMU"""
    *_, gz = bno.gyro  # rad/sec (+ turning left)
    *_, heading = bno.euler  # deg
    yaw = heading * pi / 180  # convert to radians
    # match pose angle numerically
    if yaw < -pi:
        yaw += 2 * pi
    return yaw, gz

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
        self.run = True
        self.mode = 0

    def stop(self):
        self.run = False
        motors.drive_motors(0.0, 0.0)

    async def main(self):
        try:
            while self.run:

                # get IMU data
                yaw, gz = get_imu_data()

                # read distances from VCSEL sensors
                dist_L = get_dist(b'\x02')
                dist_R = get_dist(b'\x04')
                dist_F = tof1.read()

                # get current pose
                pose = odom.update(enc_a.value(), enc_b.value())
                # pose_ang = pose[2]
                '''
                # Just turn right 90 deg
                goal_angle = -pi/2
                if yaw > (goal_angle + ANGLE_TOL):
                    motors.drive_motors(0, -0.6)
                else:
                    motors.drive_motors(0, 0)
                
                # Drive a single leg
                goal_angle = 0
                self.lin_spd = 0.4
                kp = -(yaw - goal_angle)  # proportional term
                kd = -(gz * D_GAIN)  # derivative term
                self.ang_spd = kp + kd
                motors.drive_motors(self.lin_spd, self.ang_spd)
                if dist_F < 500:
                    motors.drive_motors(0, 0)
                '''
                # Drive in a back & forth parallel line pattern
                if self.mode == 0:
                    # initial 90 deg turn to right
                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000
                    goal_angle = -pi/2
                    if yaw > (goal_angle + ANGLE_TOL):
                        motors.drive_motors(0, -0.6)
                    elif yaw < (goal_angle - ANGLE_TOL):
                        motors.drive_motors(0, 0.6)
                    else:
                        motors.drive_motors(0, 0)
                        
                        # sync pose angle to yaw value
                        await asyncio.sleep(1.0)
                        # get a fresh yaw value
                        yaw, gz = get_imu_data()
                        odom.set_angle(yaw)
                        
                        self.mode = 1
                elif self.mode == 1:
                    # drive -y direction, steering to goal angle
                    self.lin_spd = 0.4
                    kp = -(yaw - goal_angle)  # proportional term
                    kd = -(gz * D_GAIN)  # derivative term
                    self.ang_spd = kp + kd
                    motors.drive_motors(self.lin_spd, self.ang_spd)
                    if dist_F < 500:
                        motors.drive_motors(0, 0)
                        self.mode = 2
                elif self.mode == 2:
                    # turn 90 deg left
                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000
                    goal_angle = 0
                    if yaw > (goal_angle + ANGLE_TOL):
                        motors.drive_motors(0, -0.6)
                    elif yaw < (goal_angle - ANGLE_TOL):
                        motors.drive_motors(0, 0.6)
                    else:
                        motors.drive_motors(0, 0)
                        self.mode = 3
                        next_swath = pose[0] + SWATH_PITCH
                elif self.mode == 3:
                    # jog +x to next swath, steering to goal angle
                    self.lin_spd = 0.4
                    kp = -(yaw - goal_angle)  # proportional term
                    kd = -(gz * D_GAIN)  # derivative term
                    self.ang_spd = kp + kd
                    motors.drive_motors(self.lin_spd, self.ang_spd)
                    if pose[0] > next_swath:
                        self.mode = 4
                        motors.drive_motors(0, 0)
                elif self.mode == 4:
                    # turn 90 deg left
                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000
                    goal_angle = pi/2
                    if yaw > (goal_angle + ANGLE_TOL):
                        motors.drive_motors(0, -0.6)
                    elif yaw < (goal_angle - ANGLE_TOL):
                        motors.drive_motors(0, 0.6)
                    else:
                        motors.drive_motors(0, 0)
                        
                        # sync pose angle to yaw value
                        await asyncio.sleep(1.0)
                        # get a fresh yaw value
                        yaw, gz = get_imu_data()
                        odom.set_angle(yaw)
                        
                        self.mode = 5
                        goal_angle = pi/2
                elif self.mode == 5:
                    # drive +y direction, steering to goal angle
                    self.lin_spd = 0.4
                    kp = -(yaw - goal_angle)  # proportional term
                    kd = -(gz * D_GAIN)  # derivative term
                    self.ang_spd = kp + kd
                    motors.drive_motors(self.lin_spd, self.ang_spd)
                    if dist_F < 500:
                        motors.drive_motors(0, 0)
                        self.mode = 6
                elif self.mode == 6:
                    # turn right 90 deg
                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000
                    goal_angle = 0
                    if yaw > (goal_angle + ANGLE_TOL):
                        motors.drive_motors(0, -0.6)
                    elif yaw < (goal_angle - ANGLE_TOL):
                        motors.drive_motors(0, 0.6)
                    else:
                        motors.drive_motors(0, 0)
                        self.mode = 7
                        next_swath = pose[0] + SWATH_PITCH
                elif self.mode == 7:
                    # jog +x to next swath, steering to goal angle
                    self.lin_spd = 0.4
                    kp = -(yaw - goal_angle)  # proportional term
                    kd = -(gz * D_GAIN)  # derivative term
                    self.ang_spd = kp + kd
                    motors.drive_motors(self.lin_spd, self.ang_spd)
                    if pose[0] > next_swath:
                        motors.drive_motors(0, 0)
                        self.mode = 0

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
    robot_task = None
    # robot_task = asyncio.create_task(robot.main())
    while True:
        if uart1.any():
            request = read_json()
            if not request:
                continue
            print("Received: ", request)
            if request["command"] == "arena":
                send_json({"arena": arena.boundary_lines,})
            elif request["command"] == "start":
                if not robot_task:
                    robot_task = asyncio.create_task(robot.main())
            elif request["command"] == "stop":
                robot.stop()

        await asyncio.sleep(0.1)


robot = Robot()
asyncio.run(command_handler(robot))
