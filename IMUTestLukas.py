from sensors import Encoder, HallSensor, IMU, SafetyStop, GPS
import Adafruit_BBIO.GPIO as GPIO

#from actuators import RearMotorDrive, SteeringMotor
#from controller import Controller
from time import sleep
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):
         # IMU
        number_samples_IMU = raw_input('Input the number of samples or press ENTER for 50 samples for the IMU test, move the bike body for the reading! ')
        self.a_imu = IMU()
        start_time = time.time()
        if not number_samples_IMU:
            number_samples_IMU = 50
        if number_samples_IMU is not 0:
             # Setup CSV file
             results_a_imu = open('Tests_Lukas/%s-SensorTest_Lukas_IMU.csv' % timestr, 'wb')
             writer_a_imu = csv.writer(results_a_imu)
             writer_a_imu.writerow(
                 ('Time (s)', 'Phi', 'Gx (deg/s)', 'Ax (mg)', 'Ay (mg)', 'Az (mg)', 'Phi_gyro'))
        for x in range(1,int(number_samples_IMU)+1):
            #self.imureading = self.a_imu.get_reading()
            safe_time = start_time
            #print 'Time = %g\t Time_diff = %f\t Temp = %g\tAx = %g\tAy = %g\tAz = %g\tGx = %g\tGy = %g\tGz = %g\t' % (time.time() - start_time,start_time-safe_time, self.imureading[0], self.imureading[1], self.imureading[2], self.imureading[3],self.imureading[4], self.imureading[5], self.imureading[6])

            self.imudata = self.a_imu.get_imu_data()
            print 'Time = %g\tPhi = %g\tGx = %g\tAx = %g\tAy = %g\tAz = %g\tPhi_gyro = %g\n' % (time.time() - start_time, self.imudata[0], self.imudata[2], self.imudata[3], self.imudata[4],self.imudata[6], self.imudata[7])

            # Write to CSV file
            #writer_a_imu.writerow((time.time() - start_time, self.imudata[0], self.imudata[2], self.imudata[3], self.imudata[4],self.imudata[6], self.imudata[7]))

test = Test()
