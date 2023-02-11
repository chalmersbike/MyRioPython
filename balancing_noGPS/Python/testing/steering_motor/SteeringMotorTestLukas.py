import sys
sys.path.append(sys.path[0]+'/../../')
from param import *
from sensors import Encoder
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO

from actuators import SteeringMotor
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")

class Test(object):
    def __init__(self):
        try:
            self.steering_motor_drive = SteeringMotor()
            self.steering_motor_drive.set_angular_velocity(0)
            PWM.start(steeringMotor_Channel, steeringMotor_IdleDuty, steeringMotor_Frequency)

            self.encoder = Encoder()
            self.steeringAngle = self.encoder.get_angle()
            self.encoder_angle = 57.29577 * self.steeringAngle  # self.angle is negative in clockwise direction
            input1 = raw_input('Input a velocity between 0 and 10 deg/s ')

            # Setup CSV file
            results_steering = open('Tests_Lukas/%s-SensorTest_Lukas_SteeringMotor.csv' % timestr, 'wb')
            self.writer_steering = csv.writer(results_steering)
            self.writer_steering.writerow(('Time (s)', 'Steering angle (rad)', 'Current (A)'))

            # if 0 <= abs(float(input1)) <= 10:
            if 1:
                print 'tdelta = %f' % (self.encoder_angle)
                self.angular_velocity = float(input1)/57.296
                input2 = raw_input('Enter the desired self.angle! The test will start afterwards! ')
                self.angle = float(input2)
                # Setup CSV file
                # results_steering_motor = open('Tests_Lukas/%s-SensorTest_Lukas_SteeringMotor.csv' % timestr, 'wb')
                # self.writer_steering_motor = csv.writer(results_steering_motor)
                # self.writer_steering_motor.writerow(('Time (s)\t', 'tdelta (deg)\t' , 'Angular velocity (deg/s)\t'))

                self.start_time = time.time()

                # Enable steering motor
                self.steering_motor_drive.enable()
                if self.angle > 57.29577 * self.steeringAngle:
                    while self.angle >= 57.29577 * self.steeringAngle:
                        self.time_now = time.time()

                        self.steeringAngle = self.encoder.get_angle()
                        self.steeringCurrent = self.steering_motor_drive.read_steer_current()

                        print 'Time=%f,\t delta = %f,\t I = %f\t' % (self.time_now - self.start_time, 57.29577 * self.steeringAngle,self.steeringCurrent)
                        # Write to CSV file
                        # self.writer_steering_motor.writerow((time.time() - self.start_time, 57.29577 *self.steeringAngle, 57.29577*self.angular_velocity))
                        self.steering_motor_drive.set_angular_velocity(self.angular_velocity)  #will cause the motor to rotate the wheel to the right
                        # self.steering_motor_drive.set_PWM_duty_cycle(10)

                        # self.writer_steering.writerow((time.time() - self.start_time, self.steeringAngle, self.steeringCurrent))
                        self.log_str = [
                                "{0:.8f}".format(time.time() - self.start_time),
                                "{0:.8f}".format(self.steeringAngle),
                                "{0:.8f}".format(self.steeringCurrent)
                            ]
                        self.writer_steering.writerow(self.log_str)
                else:
                    while self.angle <= 57.29577 * self.steeringAngle:
                        self.time_now = time.time()

                        self.steeringAngle = self.encoder.get_angle()
                        self.steeringCurrent = self.steering_motor_drive.read_steer_current()

                        print 'Time=%f,\t tdelta = %f,\t Angular Velocity = %f\t' % (
                        self.time_now - self.start_time, 57.29577 * self.steeringAngle, 57.29577*self.angular_velocity)
                        # Write to CSV file
                        # self.writer_steering_motor.writerow((time.time() - self.start_time, 57.29577 *self.steeringAngle, 57.29577*self.angular_velocity))
                        self.steering_motor_drive.set_angular_velocity(-self.angular_velocity)  #will cause the motor to rotate the wheel to the right
                        # self.steering_motor_drive.set_PWM_duty_cycle(55)

                        # self.writer_steering.writerow((time.time() - self.start_time, self.steeringAngle, self.steeringCurrent))
                        self.log_str = [
                                "{0:.8f}".format(time.time() - self.start_time),
                                "{0:.8f}".format(self.steeringAngle),
                                "{0:.8f}".format(self.steeringCurrent)
                            ]
                        self.writer_steering.writerow(self.log_str)

            self.steering_motor_drive.stop()
            self.steering_motor_drive.set_angular_velocity(0)
            self.steering_motor_drive.disable()
        except:
            self.steering_motor_drive.stop()
            self.steering_motor_drive.set_angular_velocity(0)
            self.steering_motor_drive.disable()


test = Test()