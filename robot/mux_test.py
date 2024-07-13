# https://github.com/mcauser/micropython-tca9548a
from machine import I2C, Pin
import struct
from bno055 import BNO055
import VL53L0X


i2c0 = I2C(0, sda=Pin(12), scl=Pin(13))

# disable all 8 channels
i2c0.writeto(0x70, b'\x00')
print(i2c0.scan())

def get_dist(channel):
    i2c0.writeto(0x70, channel)
    tof = VL53L0X.VL53L0X(i2c0)
    tof.start()
    dist = tof.read()
    tof.stop()
    return dist

channel = b'\x02'  # left
print(get_dist(channel))

channel = b'\x04'  # right
print(get_dist(channel))
