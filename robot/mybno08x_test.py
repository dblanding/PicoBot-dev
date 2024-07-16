from machine import I2C, Pin
import time
import math
from bno08x_i2c import *

I2C1_SDA = Pin(14)
I2C1_SCL = Pin(15)

i2c1 = I2C(1, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000 )
print("I2C Device found at address : ",i2c1.scan(),"\n")

bno = BNO08X_I2C(i2c1, debug=False)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
#bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

while True:
    time.sleep(0.5)
    *_, heading = bno.euler  # degrees
    *_, gyro_z = bno.gyro  # radians/sec
    print(heading)
    
    print("")