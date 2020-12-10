#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
# from serial import Serial
import time


baudrate = 9600
port = "/dev/ttyO1"  # GPS serial port
UART.setup("UART1")

# baudrate = 38400
# port = "/dev/ttyO5"
# # UART.setup("UART5")

# write_str = "ident\r\n" # Run identification of the motor parameters in the motor controller
# write_str = "set -p #m_imax -v 30 -max -sudo" # Set the maximum permissible current #m_imax to 30A
                                              # Must be sent before setting the value of the maximum current
                                              # Motor driver can handle up to 50A
write_str = "set -p #m_imax -v 20" # Sets the maximum current to 20A


ser1 = serial.Serial(port, baudrate)
ser1.close()
ser1.open()
print("Serial is open")

try:
    if ser1.isOpen():
        ser1.write(write_str)
        print("Command sent : \"" + write_str + "\"")
        # while 1:
        #     # print("Serial is open")
        #     # # ser1.write(write_str)
        #     # print "Data has been sent!"
        #     # time.sleep(1)
        #     print "Reading ..."
        #     print (ser1.readline())

    ser1.close()
    print "Serial is closed"
except:
    ser1.close()
    print "Serial is closed"
