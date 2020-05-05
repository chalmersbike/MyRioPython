print 'Starting Bike with controller ACTIVE'

from bike import Bike
from controller import Controller
import Adafruit_BBIO.GPIO as GPIO
from actuators import DriveMotor, SteeringMotor
# import pysnooper

try:
    bike = Bike(debug=False)
except (ValueError, KeyboardInterrupt):
    rearmotor = DriveMotor()
    rearmotor.stop()
    steeringmotor = SteeringMotor()
    steeringmotor.stop()
    GPIO.cleanup()
    # print 'sensor reading time is %g' % bike.controller.sensor_reading_time
    print '\n Error detected, all the control signals terminated...'
    print ValueError