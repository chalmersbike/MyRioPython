#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
# from serial import Serial
import time


baudrate = 38400
port = "/dev/ttyO1"  # GPS serial port
UART.setup("UART1")

# baudrate = 115200
# port = "/dev/ttyO5"
# UART.setup("UART5")

write_str = "ident\n"

ser1 = serial.Serial(port, baudrate)
ser1.close()
ser1.open()
# print("Serial is open")
# ser1.write(write_str)
# print("Command Sent!")

if ser1.isOpen():
    while 1:
        print("Serial is open")
        # ser1.write(write_str)
        # time.sleep(1)
        # print "data has been sent!"
        print (ser1.readline())

ser1.close()
print "Serial is closed"
