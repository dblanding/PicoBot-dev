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
from parameters import P_TURN_GAIN, D_TURN_GAIN, MAX_ANG_SPD
import struct
from bno08x_i2c import *
import VL53L0X
import arena
import time

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
print("I2C address of device(s) found: ",i2c1.scan(),"\n")
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
    return gz, yaw

def sync_pose_ang_to_yaw():
    gz, yaw = get_imu_data()
    # Wait for value of gz to settle to near zero
    while abs(gz) > .002: 
        time.sleep(0.05)
        gz, yaw = get_imu_data()
    # yaw value should now be unchanging
    odom.set_angle(yaw)

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
        self.lin_spd = 0.4  # nominal drive speed
        self.ang_spd = 0  # prev value ang_spd only when stuck
        self.run = True
        self.mode = 'T'  # 'T' for tele-op, 0 for S&R pattern
        self.errors = []

    def turn(self, goal_angle, gz, yaw):
        """
        Return ang_spd needed to drive motors when
        turning in place to goal_angle (radians).
        self.ang_spd is used to remember prev ang_spd when stuck
        """
        # calculate ang_spd to steer to goal_angle
        yaw_err = yaw - goal_angle
        p = -(yaw_err * P_TURN_GAIN)  # proportional term
        d = -(gz * D_TURN_GAIN)  # derivative term
        ang_spd = p + d

        # limit value of ang_spd
        if ang_spd < -MAX_ANG_SPD:
            ang_spd = -MAX_ANG_SPD
        if ang_spd > MAX_ANG_SPD:
            ang_spd = MAX_ANG_SPD

        # reset self.ang_spd to zero if not stuck
        if abs(gz) > 0.01:
            self.ang_spd = 0

        # check if turn is complete
        if abs(gz) < 0.01 and abs(yaw_err) < ANGLE_TOL:
            ang_spd = 0

        # give an extra boost if needed to overcome static friction
        elif abs(gz) < 0.01 and abs(yaw_err) < 3 * ANGLE_TOL:
            if not self.ang_spd:  # not previously boosted
                ang_spd *= 1.5
                self.ang_spd = ang_spd
            else:  # boost it further
                ang_spd = self.ang_spd * 1.5
                self.ang_spd = 0

        return ang_spd

    def stop(self):
        self.run = False
        motors.drive_motors(0, 0)

    async def main(self):
        try:
            while self.run:
                # read distances from VCSEL sensors
                dist_L = get_dist(b'\x02')
                dist_R = get_dist(b'\x04')
                dist_F = tof1.read()

                # get current pose
                pose = odom.update(enc_a.value(), enc_b.value())

                # get IMU data
                gz, yaw = get_imu_data()

                # Drive in tele-op mode
                if self.mode == 'T':
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
                            self.errors.append(e)

                        # send commands to motors
                        motors.drive_motors(self.lin_spd, self.ang_spd)

                # Drive in a back & forth "S & R" pattern
                elif self.mode == 0:
                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000

                    # turn in place to face -Y direction
                    goal_angle = -pi/2
                    ang_spd = self.turn(goal_angle, gz, yaw)
                    motors.drive_motors(0, ang_spd)

                    # until turn is complete
                    if ang_spd == 0:
                        sync_pose_ang_to_yaw()
                        pose = odom.update(enc_a.value(), enc_b.value())
                        self.mode = 1

                elif self.mode == 1:
                    # drive -y direction, steering to goal_angle
                    p = -(yaw - goal_angle)  # proportional term
                    d = -(gz * D_GAIN)  # derivative term
                    ang_spd = p + d
                    motors.drive_motors(self.lin_spd, ang_spd)

                    # stop
                    if pose[1] < -1.3:  # dist_F < 500:
                        motors.drive_motors(0, 0)
                        self.mode = 2

                elif self.mode == 2:
                    # turn to face +X direction
                    goal_angle = 0

                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000

                    # turn in place to goal angle
                    ang_spd = self.turn(goal_angle, gz, yaw)
                    motors.drive_motors(0, ang_spd)

                    # when turn is complete
                    if ang_spd == 0:
                        pose = odom.update(enc_a.value(), enc_b.value())
                        next_swath = pose[0] + SWATH_PITCH
                        self.mode = 3

                elif self.mode == 3:
                    # jog +X direction to next swath
                    p = -(yaw - goal_angle)  # proportional term
                    d = -(gz * D_GAIN)  # derivative term
                    ang_spd = p + d
                    motors.drive_motors(self.lin_spd, ang_spd)

                    # stop at next_swath
                    if pose[0] >= next_swath:
                        motors.drive_motors(0, 0)
                        self.mode = 4

                elif self.mode == 4:
                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000

                    # turn in place to face +Y direction
                    goal_angle = pi/2
                    ang_spd = self.turn(goal_angle, gz, yaw)
                    motors.drive_motors(0, ang_spd)

                    # until turn is complete
                    if ang_spd == 0:
                        sync_pose_ang_to_yaw()
                        pose = odom.update(enc_a.value(), enc_b.value())
                        self.mode = 5

                elif self.mode == 5:
                    # drive +y direction, steering to goal angle
                    p = -(yaw - goal_angle)  # proportional term
                    d = -(gz * D_GAIN)  # derivative term
                    ang_spd = p + d
                    motors.drive_motors(self.lin_spd, ang_spd)

                    # stop
                    if dist_F < 500:
                        motors.drive_motors(0, 0)
                        self.mode = 6

                elif self.mode == 6:
                    # turn right 90 deg
                    goal_angle = 0

                    # suppress distance values while turning
                    dist_L = 2000
                    dist_R = 2000
                    dist_F = 2000

                    # turn in place to goal angle
                    ang_spd = self.turn(goal_angle, gz, yaw)
                    motors.drive_motors(0, ang_spd)

                    # when turn is complete
                    if ang_spd == 0:
                        pose = odom.update(enc_a.value(), enc_b.value())
                        next_swath = pose[0] + SWATH_PITCH
                        self.mode = 7

                elif self.mode == 7:
                    # jog +x to next swath
                    p = -(yaw - goal_angle)  # proportional term
                    d = -(gz * D_GAIN)  # derivative term
                    ang_spd = p + d
                    motors.drive_motors(self.lin_spd, ang_spd)

                    # stop at next_swath
                    if pose[0] >= next_swath:
                        motors.drive_motors(0, 0)
                        self.mode = 0

                # send robot data to laptop
                if pose != (0, 0, 0):
                    send_json({
                        "pose": list(pose),
                        "gz": gz,
                        "yaw": yaw,
                        "dist_L": dist_L,
                        "dist_R": dist_R,
                        "dist_F": dist_F,
                        "errors": self.errors,
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
