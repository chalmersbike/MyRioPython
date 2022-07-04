import sys
sys.path.append(sys.path[0]+'/../../')
import param
from sensors import HallSensor
import Adafruit_BBIO.GPIO as GPIO
from actuators import DriveMotor
import time
import csv
import pysnooper

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")


# @pysnooper.snoop()
class Test(object):
    def __init__(self):
        # Drive motor
        input1 = input('Input a velocity between 1 and 5 m/s, the test will last 20 secs ')
        self.rear_motor_velocity = DriveMotor()
        # Setup CSV file
        # results_velocity = open('Tests_Lukas/%s-SensorTest_Lukas_Velocity_Motor.csv' % timestr, 'wb')
        # writer_velocity = csv.writer(results_velocity)
        # writer_velocity.writerow(('Time (s)', 'Set velocity (m/s)', 'Measured Velocity (m/s)'))
        if not input1:
            print('Press enter to exit')
            exit()
        elif float(input1) >= 1 and float(input1) <= 5:
            input_velocity = float(input1)
            try:
                input('You need to be careful on the velocity reading, if they deviate too much, maybe there is problem in gear setting, see rearmotordrive.py!')
            except:
                pass
            start_time = time.time()
            self.rear_motor_velocity.set_velocity(int(input_velocity))
            time_now = start_time
            # self.rear_motor = DriveMotor()
            self.hall_sensor = HallSensor()

            try:
                while time_now - start_time < 20000:
                    time_now = time.time()
                    # self.rear_motor_velocity.set_velocity(input_velocity)
                    print('Time=%f\t Vel = %f\t' % (time_now-start_time, self.hall_sensor.get_velocity()))
                    time.sleep(0.5)

                    # Write to CSV file
                    # writer_velocity.writerow((time_now - start_time, input_velocity, self.hall_sensor.get_velocity()))

                self.rear_motor_velocity.stop()
            except:
                self.rear_motor_velocity.stop()

test = Test()