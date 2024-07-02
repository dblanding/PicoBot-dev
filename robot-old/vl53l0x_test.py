import time
import VL53L0X
from machine import I2C, Pin

# set up left & right VCSEL TOF distance sensors
def setup_tof_sensor(bus_id, sda_pin, scl_pin):
    """Setup a Vcsel sensor on an I2C bus.
    There are two available busses: 0 & 1.
    Return VL53L0X object."""
    sda = Pin(sda_pin)
    scl = Pin(scl_pin)

    print("setting up i2c%s" % bus_id)
    i2c = I2C(id=bus_id, sda=sda, scl=scl)
    print("Set up device %s on i2c%s" % (i2c.scan(), bus_id))

    return VL53L0X.VL53L0X(i2c)


tof0 = setup_tof_sensor(0, 12, 13)  # Left
tof0.start()
tof1 = setup_tof_sensor(1, 10, 11)  # Right
tof1.start()

print(dir(tof0))

while True:
    l_dist = tof0.read()
    r_dist = tof1.read()
    print("left: %d cm, right: %d cm" % (l_dist / 10, r_dist / 10))
    time.sleep(0.1)

tof0.stop()
tof1.stop()