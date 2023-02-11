import sys
sys.path.append(sys.path[0] + '/../../')
from sensors import HallSensor
import Adafruit_BBIO.GPIO as GPIO

import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):

        # Hall sensor
        number_samples_HallSensor = raw_input('Input the number of samples for the Hall sensor test, rotate the rear wheel for the reading! ')
        self.hall_sensor = HallSensor()
        start_time = time.time()
        if number_samples_HallSensor is not 0:
            # Setup CSV file
            results_hallsensor = open('Tests_Lukas/%s-SensorTest_Lukas_HallSensor.csv' % timestr, 'wb')
            writer_hallsensor = csv.writer(results_hallsensor)
            writer_hallsensor.writerow(('Time (s)', 'Speed (m/s)'))

        hall_sensor_velocity_previous = 0.0

        for x in range(1,int(number_samples_HallSensor)+1):
            hall_sensor_velocity = self.hall_sensor.get_velocity()

            if hall_sensor_velocity - hall_sensor_velocity_previous > 1.5:
                print('WARNING : [%f] Measured speed change between two samples too large')
                hall_sensor_velocity = hall_sensor_velocity_previous
            hall_sensor_velocity_previous = hall_sensor_velocity

            print 'Time=%f\t Vel = %f  m/s\n' % (time.time()-start_time, hall_sensor_velocity)
            # Write to CSV file
            writer_hallsensor.writerow((time.time() - start_time, hall_sensor_velocity))
            time.sleep(0.1)
test = Test()
