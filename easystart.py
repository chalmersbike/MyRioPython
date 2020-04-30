print 'Starting Bike with controller ACTIVE'

from bike import Bike
from controller import Controller
import Adafruit_BBIO.GPIO as GPIO
from actuators import RearMotorDrive, Steering
# import pysnooper

try:
    bike = Bike(debug=False)
except (ValueError, KeyboardInterrupt):
    rearmotor = RearMotorDrive()
    rearmotor.stop()
    steeringmotor = Steering()
    steeringmotor.stop()
    GPIO.cleanup()
    # print 'sensor reading time is %g' % bike.controller.sensor_reading_time
    print '\n Error detected, all the control signals terminated...'
    print ValueError