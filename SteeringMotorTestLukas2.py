from sensors import Encoder
import Adafruit_BBIO.GPIO as GPIO

from actuators import SteeringMotor, Steering
import time
import csv
import sys


class Test(object):
    def __init__(self):
        self.steering_motor_drive = Steering()
        self.steering_motor_drive.enable()
        self.encoder = Encoder()
        self.encoder_angle = 57.29577 * self.encoder.get_angle()  # angle is negative in clockwise direction
        print 'tdelta = %f' % (self.encoder_angle)
        angular_velocity = 10/57.29577
        angle = 20
        start_time = time.time()
        self.steering_motor_drive.set_angular_velocity(-angular_velocity)
        while angle >= 57.29577 * self.encoder.get_angle():
            print 'Time=%f,\t tdelta = %f,\t Angular Velocity = %f\t' % (
            time.time() - start_time, 57.29577 * self.encoder.get_angle(), 57.29577*angular_velocity)
        self.steering_motor_drive.stop()

test = Test()
