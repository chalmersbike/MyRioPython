from sensors import Encoder, HallSensor, IMU, SafetyStop, GPS
import Adafruit_BBIO.GPIO as GPIO

from actuators import RearMotorDrive, SteeringMotor
from controller import Controller
from time import sleep
import time
import csv


# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):

        # Hall sensor
        number_samples_HallSensor = raw_input('Input the number of samples or press ENTER for 50 samples for the Hall sensor test, rotate the rear wheel for the reading! ')
        self.hall_sensor = HallSensor()
        start_time = time.time()
        #def get_timebetween(self):
         #   self.last_time_measured = time()
          #  time_measured = time()
           # self.elapse = time() - self.last_time_measured
        if not number_samples_HallSensor:
            number_samples_HallSensor = 50
        if number_samples_HallSensor is not 0:
            # Setup CSV file
            results_hallsensor = open('Tests_Lukas/%s-SensorTest_Lukas_HallSensor.csv' % timestr, 'wb')
            writer_hallsensor = csv.writer(results_hallsensor)
            writer_hallsensor.writerow(('Time (s)', 'Speed (m/s)'))
        for x in range(1,int(number_samples_HallSensor)+1):
            hall_sensor_velocity = self.hall_sensor.get_velocity_kmh()
            print 'Time=%f\t Vel = %f  km/h\n' % (time.time()-start_time, hall_sensor_velocity)
            # Write to CSV file
            writer_hallsensor.writerow((time.time() - start_time, hall_sensor_velocity))
            time.sleep(0.5)
test = Test()
