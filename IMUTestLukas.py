from sensors import IMU
import Adafruit_BBIO.GPIO as GPIO

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
        if number_samples_IMU is not 0:
             # Setup CSV file
             results_a_imu = open('Tests_Lukas/%s-SensorTest_Lukas_IMU.csv' % timestr, 'wb')
             writer_a_imu = csv.writer(results_a_imu)
             writer_a_imu.writerow(('Time (s)', 'Phi', 'Gx (deg/s)', 'Ax (mg)', 'Ay (mg)', 'Az (mg)', 'Phi_gyro'))
        for x in range(1,int(number_samples_IMU)+1):
             self.imudata = self.a_imu.get_imu_data()
             print 'Time = %g\tPhi = %g\tGx = %g\tAx = %g\tAy = %g\tAz = %g\tPhi_gyro = %g\n' % (time.time() - start_time, self.imudata[0], self.imudata[2], self.imudata[3], self.imudata[4],self.imudata[6], self.imudata[7])

             # Write to CSV file
             writer_a_imu.writerow((time.time() - start_time, self.imudata[0], self.imudata[2], self.imudata[3], self.imudata[4],self.imudata[6], self.imudata[7]))
             time.sleep(0.1)
test = Test()
