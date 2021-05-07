from param import *
import serial, time, numpy
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.ADC as ADC
from pyvesc import VESC
import pysnooper

# @pysnooper.snoop()
class DriveMotor(object):
    serial = None

    def __init__(self):
        UART.setup(driveMotor_UARTPort)
        # ADC.setup()
        self.serial = VESC(serial_port=driveMotor_port,baudrate=driveMotor_CommunicationFrequency,start_heartbeat=True)
        # self.serial.stop_heartbeat()
        # self.serial.start_heartbeat()
        if debug:
            print('Drive Motor : Serial port opened')
        self.Time = -1 # Initialize time to -1 as a way to check that we correctly read from the controller after starting the logging

    def rear_set_rpm(self, rpm):
        self.serial.set_rpm(int(rpm))

    def set_velocity(self, input_velocity):
        # self.rear_set_pwm(self._m_per_second_to_pwm(input_velocity))
        self.rear_set_rpm(input_velocity*800)
        print('VESC : Set speed')
        # self.serial.start_heartbeat()
        # for GEAR 6Th the coef vel -> pwm = 0.31
        # self.rear_set_rpm(input_velocity * 0.31)

    def stop(self):
        print('VESC : Stop')
        self.serial.set_rpm(0)
        self.serial.stop_heartbeat()