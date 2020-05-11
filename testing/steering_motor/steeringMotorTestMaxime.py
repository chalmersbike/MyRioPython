import sys
sys.path.append(sys.path[0]+'/../../')
import param
from sensors import Encoder
import Adafruit_BBIO.GPIO as GPIO

from actuators import SteeringMotor
import time
import csv


try:
    steering_motor_drive = SteeringMotor()

    encoder = Encoder()
    encoder_angle = 57.29577 * encoder.get_angle()  # angle is negative in clockwise direction
    print('t = 0 ; delta = %f' % (encoder_angle))

    steering_motor_drive.set_angular_velocity(0)
    #time.sleep(30)
    steering_motor_drive.enable()
    encoder_angle = 57.29577 * encoder.get_angle()  # angle is negative in clockwise direction
    time_start = time.time()
    print('t = %f ; delta = %f' % (time.time() - time_start, encoder_angle))
    while encoder_angle < 20.0/57.29577:
        encoder_angle = 57.29577 * encoder.get_angle()  # angle is negative in clockwise direction
        steering_motor_drive.set_angular_velocity(10.0/57.29577)
        print('t = %f ; delta = %f' %(time.time()-time_start,encoder_angle))
    # input1 = raw_input('Input a velocity between 0 and 5 deg/s ')
    sleep(0.01)
except:
    steering_motor_drive.set_angular_velocity(0)
    steering_motor_drive.stop()

steering_motor_drive.set_angular_velocity(0)
steering_motor_drive.stop()