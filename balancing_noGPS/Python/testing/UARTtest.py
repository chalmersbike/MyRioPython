#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
# from serial import Serial
import time


baudrate = 460800
# baudrate = 9600
port = "/dev/ttyO1"  # GPS serial port
port = "/dev/ttyO4"  # GPS serial port
# UART.setup("UART1")

# baudrate = 38400
# port = "/dev/ttyO5"
# # UART.setup("UART5")

write_str = "$GNRMC,122837.30,A,5741.28526,N,01158.77734,E,0.026,,130421,,,A,V*1A\r\n"
# write_str = "ident\r\n"


ser1 = serial.Serial(port, baudrate)
ser1.close()
ser1.open()
# print("Serial is open")
# ser1.write(write_str)
# print("Command Sent!")

try:
    if ser1.isOpen():
        while 1:
            print("Serial is open")
            ser1.write(write_str)
            print "Data has been sent!"
            # time.sleep(1)
            print "Reading ..."
            t_start_read = time.time()
            print (ser1.readline())
            print('time read : %f' % (time.time() - t_start_read))
            time.sleep(0.1)

    ser1.close()
    print "Serial is closed"
except:
    ser1.close()
    print "Serial is closed"