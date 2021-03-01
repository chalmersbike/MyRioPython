#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
# from serial import Serial
import time


baudrate = 38400
# baudrate = 9600
port = "/dev/ttyO1"  # GPS serial port
UART.setup("UART1")

# baudrate = 38400
# port = "/dev/ttyO5"
# # UART.setup("UART5")

write_str = "ident\r\n"


ser1 = serial.Serial(port, baudrate)
ser1.close()
ser1.open()
# print("Serial is open")
# ser1.write(write_str)
# print("Command Sent!")

try:
    if ser1.isOpen():
        while 1:
            # print("Serial is open")
            # # ser1.write(write_str)
            # print "Data has been sent!"
            # time.sleep(1)
            print "Reading ..."
            print (ser1.readline())

    ser1.close()
    print "Serial is closed"
except:
    ser1.close()
    print "Serial is closed"