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
         # Encoder
        number_samples_Encoder = raw_input('Input the number of samples or press ENTER for 100 samples for the encoder test, rotate the handle bar for the reading! ')
        self.encoder = Encoder()
        start_time = time.time()
        print 'ENCODER FREQENCY = %f \n' % (self.encoder.encoder.frequency)
        if not number_samples_Encoder:
            number_samples_Encoder = 100
        if number_samples_Encoder is not 0:
            # Setup CSV file
            results_encoder = open('Tests_Lukas/%s-SensorTest_Lukas_Encoder.csv' % timestr, 'wb')
            writer_encoder = csv.writer(results_encoder)
            writer_encoder.writerow(('Time (s)', 'Steering angle (deg)'))
            for x in range(1,int(number_samples_Encoder)+1):
                print 'Time=%f\t delta = %f\n' % (time.time() - start_time, 57.29577 * self.encoder.get_angle())
                # Write to CSV file
                writer_encoder.writerow((time.time() - start_time, self.encoder.get_angle()))
                #time.sleep(0.5)
test = Test()
