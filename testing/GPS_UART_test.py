#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
# from serial import Serial
import time


baudrate = 38400
# baudrate = 460800
# baudrate = 921600
port = "/dev/ttyO1"  # GPS serial port
UART.setup("UART1")

write_str = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"

ser1 = serial.Serial(port, baudrate)
ser1.close()
ser1.open()
print("Serial is open")

time_start_write = time.time()
ser1.write(write_str)
time_end_write = time.time()
time_start_read = time_end_write
readall = ser1.readline()
time_end_read = time.time()
print('read : ' + readall)
print('time write : %f ; time read : %f' % (time_end_write-time_start_write,time_end_read-time_start_read))

ser1.close()
print "Serial is closed"