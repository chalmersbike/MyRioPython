from sensors import HallSensor
import Adafruit_BBIO.GPIO as GPIO

from actuators import RearMotorDrive, SteeringMotor, RearMotor
from controller import Controller
from time import sleep
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")


class Test(object):
    def __init__(self):
        # Drive motor
        input1 = raw_input(
            'If all the tests are passed, then you may check the rear motor and steering motor. HOWEVER THEY ARE RISKY! PRESS ENTER TO CONTINUE')
        if not input1:
            input1 = raw_input('Input a velocity between 1 and 5 m/s, the test will last 20 secs ')
            self.rear_motor_velocity = RearMotorDrive()
            # Setup CSV file
            results_velocity = open('Tests_Lukas/%s-SensorTest_Lukas_Velocity_Motor.csv' % timestr, 'wb')
            writer_velocity = csv.writer(results_velocity)
            writer_velocity.writerow(('Time (s)', 'Set velocity (m/s)', 'Actual Velocity (m/s)', 'Current (A)', 'RPM (1/s)'))
            if not input1:
                print 'Press enter to exit'
                exit()
            elif float(input1) >= 1 and float(input1) <= 5:
                input_velocity = float(input1)
                raw_input(
                    'You need to be careful on the velocity reading, if they deviate too much, maybe there is problem in gear setting, see rearmotordrive.py!')
                start_time = time.time()
                self.rear_motor_velocity.set_velocity(input_velocity)
                time_now = start_time
                self.rear_motor = RearMotor()
                self.hall_sensor = HallSensor()

                while time_now - start_time < 8:
                    time_now = time.time()
                    self.rear_motor_velocity.set_velocity(input_velocity)
                    print 'Time=%f\t Vel = %f\t' % (time_now-start_time, self.hall_sensor.get_velocity())
                    # Write to CSV file
                    writer_velocity.writerow((time_now - start_time, input_velocity, self.hall_sensor.get_velocity(), self.rear_motor.read_motor_current(), self.rear_motor.read_motor_rpm()))
                    time.sleep(0.1)
                if time_now - start_time > 8:
                    self.rear_motor_velocity.stop()

            else:
                print 'Test Finished'
        else:
            print 'It is done, the process will be killed, GOOD LUCK!'


test = Test()
