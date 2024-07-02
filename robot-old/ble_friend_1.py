# Send data to Bluefruit LE phone app (UART mode)
from machine import Pin, UART
import time

uart = UART(0, 9600)                   # init with given baudrate
uart.init(bits=8, parity=None, stop=1) # init with given parameters
x = 0
y = 0
while True:
    x += 1
    y += 1
    uart.write(f"{x}, {y}\n".encode())
    time.sleep(1)