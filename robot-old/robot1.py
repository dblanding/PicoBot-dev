from machine import UART

uart = UART(0, 9600)
uart.init(bits=8, parity=None, stop=1, timeout=10)
