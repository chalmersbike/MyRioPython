import sys
from sensors import Encoder, HallSensor, SafetyStop, GPS
import Adafruit_BBIO.GPIO as GPIO
from IMU import IMU as imu_bb
from actuators import RearMotorDrive, SteeringMotor
from controller import Controller
from time import sleep
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):
        # Test GPS
        number_samples_GPS = raw_input('Input the number of samples of press ENTER for 50 samples for the GPS test, move the GPS for the reading! ')
        #imu = imu_bb(0)
        gps = GPS()
        start_time = time.time()
        if not number_samples_GPS:
            number_samples_GPS = 50
        if number_samples_GPS is not 0:
        # Setup CSV file
            results_gps = open('Tests_Lukas/%s-SensorTest_Lukas_GPS.csv' % timestr, 'wb')
            writer_gps = csv.writer(results_gps)
            writer_gps.writerow(('Time (s)', 'x (m)', 'y (m)', 'latitude', 'longitude'))#,'Temp', 'ax (mg)','ay (mg)', 'az (mg)', 'gx (deg/s)', 'gy (deg/s)', 'gz (deg/s)', 'Mx', 'My', 'Mz', 'MDefective'))
        for x in range(1,int(number_samples_GPS)+1):
            gpspos = gps.get_position()
            print 'Time=%f\t' % (time.time() - start_time)
            print 'Temp=%g\tAx = %g\t' % (gpspos[0], gpspos[1])
            #imudata = imu.IMU_Read()
            # Write to CSV file
            elapsed_time = time.time() - start_time
            writer_gps.writerow((elapsed_time, gpspos[0], gpspos[1], gps.latitude, gps.longitude))#, imudata[4], imudata[1], imudata[2], imudata[3], imudata[5],imudata[6], imudata[7], imudata[8], imudata[9], imudata[10], int(imudata[11])))
            #time.sleep(0.005)
test = Test()
