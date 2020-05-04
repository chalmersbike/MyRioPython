from sensors import Encoder
import Adafruit_BBIO.GPIO as GPIO

from actuators import Steering
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")

class Test(object):
    def __init__(self):
        self.steering_motor_drive = Steering()
        self.steering_motor_drive.set_angular_velocity(0)
        self.steering_motor_drive.enable()
        self.encoder = Encoder()
        self.encoder_angle = 57.29577 * self.encoder.get_angle()  # angle is negative in clockwise direction
        input1 = raw_input('Input a velocity between 0 and 5 deg/s ')
        if 0 <= abs(float(input1)) <= 10:
            print 'tdelta = %f' % (self.encoder_angle)
            angular_velocity = float(input1)/57.296
            input2 = raw_input('Enter the desired angle! The test will start afterwards! ')
            angle = float(input2)
            # Setup CSV file
            results_steering_motor = open('Tests_Lukas/%s-SensorTest_Lukas_SteeringMotor.csv' % timestr, 'wb')
            writer_steering_motor = csv.writer(results_steering_motor)
            writer_steering_motor.writerow(('Time (s)\t', 'tdelta (deg)\t' , 'Angular velocity (deg/s)\t'))
            start_time = time.time()
            if angle > 57.29577 * self.encoder.get_angle():
                while angle >= 57.29577 * self.encoder.get_angle():
                    time_now = time.time()
                    print 'Time=%f,\t tdelta = %f,\t Angular Velocity = %f\t' % (time_now - start_time, 57.29577 * self.encoder.get_angle(),57.29577 * angular_velocity)
                    # Write to CSV file
                    writer_steering_motor.writerow((time.time() - start_time, 57.29577 *self.encoder.get_angle(), 57.29577*angular_velocity))
                    self.steering_motor_drive.set_angular_velocity(angular_velocity)  #will cause the motor to rotate the wheel to the right
                angular_velocity = 0
                self.steering_motor_drive.set_angular_velocity(angular_velocity)
            else:
                while angle <= 57.29577 * self.encoder.get_angle():
                    time_now = time.time()
                    print 'Time=%f,\t tdelta = %f,\t Angular Velocity = %f\t' % (
                    time_now - start_time, 57.29577 * self.encoder.get_angle(), 57.29577*angular_velocity)
                    # Write to CSV file
                    writer_steering_motor.writerow((time.time() - start_time, 57.29577 *self.encoder.get_angle(), 57.29577*angular_velocity))
                    self.steering_motor_drive.set_angular_velocity(-angular_velocity)  #will cause the motor to rotate the wheel to the right
                angular_velocity = 0
                self.steering_motor_drive.set_angular_velocity(-angular_velocity)
test = Test()