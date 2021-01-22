#!/usr/bin/python
import sys
sys.path.append(sys.path[0]+'/../')
from param import *
import Adafruit_BBIO.UART as UART
import serial
import time
from sensors import HallSensor

UART.setup(driveMotor_UARTPort)

write_str = "ident\r\n" # Run identification of the motor parameters in the motor controller

# write_str = "set -p #m_imax -v 30 -max -sudo\r\n" # Set the maximum permissible current #m_imax to 30A
                                                        # Must be sent before setting the value of the maximum current
                                                        # Motor driver can handle up to 50A
# write_str = "set -p #m_imax -v 20\r\n"            # Sets the maximum current to 20A
# write_str = "set -p #ibus_limp -v 20\r\n"         # Set maximum bus current to 20A

ser1 = serial.Serial(port=driveMotor_port, baudrate=driveMotor_CommunicationFrequency)
ser1.close()
ser1.open()
ser1.reset_input_buffer()
print("Serial is open")


# Setup Hall sensor
hall_sensor = HallSensor()


try:
    if ser1.isOpen():
        # Set motor speed
        # ser1.write("run -s 1 -f 9999\n")

        # Send command to set parameters or identify motor parameters
        # ser1.write(write_str)
        # print("Command sent : \"" + write_str + "\"")

        # Get motor controller parameters
        # print("\nGetting motor controller parameters ...")
        # ser1.write("get\r\n")
        # time.sleep(0.1)
        # while ser1.in_waiting: # Read until UART input buffer is empty, which means we got all controller parameters
        #     print(ser1.readline()[0:-1]) # Print line without new line at the end
        #     # print(ser1.readline())

        # Log data with motor running
        print("\nStarting motor controller logging ...")
        # ser1.write('run -s 0.6 -f 9999\r\n')
        ser1.write("log -p 0.1 -k\n")
        ser1.reset_input_buffer()
        while 1:#ser1.in_waiting:
            # print(ser1.readline()[0:-1]) # Print line without new line at the end
            print(ser1.readline())

            # Get speed from Hall sensor
            print("Hall sensor: %f m/s\n" % (hall_sensor.get_velocity()))

    ser1.close()
    print "\nSerial is closed"
except:
    ser1.close()
    print "Serial is closed"
