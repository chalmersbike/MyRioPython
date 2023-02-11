from bike_zeroSpeed import Bike
from controller import Controller
import Adafruit_BBIO.GPIO as GPIO
from actuators import DriveMotor, SteeringMotor
import sys
import getopt
import pysnooper

try:
    print 'Starting Bike with controller ACTIVE\n'


    @pysnooper.snoop()
    bike = Bike()
except (ValueError, KeyboardInterrupt):
    rearmotor = DriveMotor()
    rearmotor.stop()
    steeringmotor = SteeringMotor()
    steeringmotor.stop()
    GPIO.cleanup()
    # print 'sensor reading time is %g' % bike.controller.sensor_reading_time
    # print '\n Error detected, all the control signals terminated...'
    exc_msg = '\n Error detected by easystart program, all the control signals terminated...'
    print(exc_msg)
    print ValueError
    # bike.exception_log(-2, exc_msg)