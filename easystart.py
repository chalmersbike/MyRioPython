print 'Starting Bike with controller ACTIVE'
from bike import Bike
from controller import Controller
import Adafruit_BBIO.GPIO as GPIO
from actuators import RearMotorDrive, Steering

try:
    bike = Bike(debug=False)
except (ValueError, KeyboardInterrupt):
    rearmotor = RearMotorDrive()
    rearmotor.stop()
    steeringmotor = SteeringMotor()
    steeringmotor.set_angular_velocity(0)
    GPIO.cleanup()
    print '\n Error detected, all the control signals terminated...'