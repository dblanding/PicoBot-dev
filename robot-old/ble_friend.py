from machine import Pin, UART

uart = UART(0, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

while True:
    if uart.read(32) is not None:
        uart.write("Hello, Bluetooth World!\n".encode())