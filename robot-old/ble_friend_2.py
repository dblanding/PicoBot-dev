# Read & decode data from Bluefruit LE controller phone app
from machine import Pin, UART
import struct
import time

uart = UART(0, 9600)
uart.init(tx=0, rx=1, bits=8, parity=None, stop=1, timeout=10)

def decode(bs):
    return struct.unpack('3f', bs)

print("Waiting for incoming bytes on UART...:")
while True:
    if uart.any():
        try:
            bytestring = uart.readline()
            substring = bytestring[2:14]
            print(decode(substring))
            print('')
        except Exception as e:
            print(e)
    time.sleep(.1)
    