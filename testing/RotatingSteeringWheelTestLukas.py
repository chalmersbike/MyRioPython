import sys
sys.path.append(sys.path[0] + '/../')
from param import *
from sensors import Encoder
import Adafruit_BBIO.GPIO as GPIO
from actuators import SteeringMotor
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")

class Test(object):
    def __init__(self):
        self.steering_motor_drive = SteeringMotor()
        self.steering_motor_drive.set_angular_velocity(0)
        self.steering_motor_drive.enable()
        self.encoder = Encoder()
        self.encoder_angle = -57.29577 * self.encoder.get_angle()  # with the "-" the angle becomes positive in clockwise direction
        input1 = raw_input('Input a velocity between 1 and 100 deg/s ')
        if not input1:
            print 'Press enter to exit'
            exit()
        elif 0 <= float(input1) <= 100:
            print 'tdelta = %f' % (self.encoder_angle)
            raw_input('The test will start now and last 30 seconds! ')
            angular_velocity = float(input1)*deg2rad
            # Setup CSV file
            results_steering_motor = open('Tests_Lukas/%s-SensorTest_Lukas_RotatingSteeringWheel.csv' % timestr, 'wb')
            writer_steering_motor = csv.writer(results_steering_motor)
            writer_steering_motor.writerow(('Time (s)\t', 'tdelta (deg)\t' , 'Angular velocity (deg/s)\t'))
            start_time = time.time()
            time_now = time.time()
            while time_now - start_time < 30:
                time_now = time.time()
                print 'Time=%f\t tdelta = %f\t Angular Velocity = %f\t' % (
                    time_now - start_time, -57.29577 * self.encoder.get_angle(), angular_velocity)
                # Write to CSV file
                writer_steering_motor.writerow((time.time() - start_time, self.encoder.get_angle(), angular_velocity))
                time.sleep(0.2)
                self.steering_motor_drive.set_angular_velocity(angular_velocity)  #will cause the motor to rotate the wheel to the right
                if (57.29577 * self.encoder.get_angle()) < -20:     #will cause the wheel to rotate back left because it is too far right
                    angular_velocity = -angular_velocity
                    print 'Too far on the right!'
                if (-57.29577 * self.encoder.get_angle()) < -20:    #will cause the wheel to rotate back right because it is too far left
                    angular_velocity = -angular_velocity
                    print 'Too far on the left!'
            angular_velocity = 0
            self.steering_motor_drive.set_angular_velocity(angular_velocity)

test = Test()

