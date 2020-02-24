from steering import SteeringMotor
import Adafruit_BBIO.GPIO as GPIO
import time

Pin_enable = 'P9_22'

#GPIO.setup(Pin_enable, GPIO.OUT)
#GPIO.output(Pin_enable, GPIO.LOW)
#GPIO.cleanup()

mot = SteeringMotor()
mot.set_angular_velocity(0)
mot.stop()