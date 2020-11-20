import sys
import os
sys.path.append(sys.path[0]+'/../../')
# os.chdir("../../")
# os.chdir("./")
pwd = os.system("pwd")
print (pwd)

from sensors import IMU
import Adafruit_BBIO.GPIO as GPIO

import time
import csv

import param

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
# RESULTS = open('imu_data_test-%s.csv' % timestr, 'wb')
class Test(object):
    def __init__(self):
         # IMU
        number_samples_IMU = raw_input('Input the number of samples or press ENTER for 50 samples for the IMU test, move the bike body for the reading! ')
         # writer = csv.writer(RESULTS)
         # writer.writerow(('time', 'phi_d', 'phi'))  # headers
        self.a_imu = IMU()
        start_time = time.time()
        #if number_samples_IMU is not 0:
             # Setup CSV file
        results_a_imu = open(sys.path[0]+'/%s-SensorTest_Lukas_IMU.csv' % timestr, 'wb')
        writer_a_imu = csv.writer(results_a_imu)
        # writer_a_imu.writerow(('Time (s)', 'Phi', 'Gx (rad/s)', 'Ax (mg)', 'Ay (mg)', 'Az (mg)', 'Phi_gyro'))
        writer_a_imu.writerow(('Time (s)', 'Phi', 'Gx (rad/s)', 'Gy (rad/s)', 'Gz (rad/s)', 'Ax (mg)', 'Ay (mg)', 'Az (mg)'))
        for x in range(1,int(number_samples_IMU)+1):
            loop_start_time = time.time()
            # [phi_comp, phi_gyro, gx (phidot), gy, gz, ax, ay, az]
            self.imudata = self.a_imu.get_imu_data(0,0,0)

            print 'Time = %g\tPhi = %g\tGx = %g\tGy = %g\tGz = %g\tAx = %g\tAy = %g\tAz = %g\n' % (time.time() - start_time, self.imudata[0], self.imudata[2], self.imudata[3], self.imudata[4], self.imudata[5], self.imudata[6], self.imudata[7])

            # Write to CSV file
            # writer_a_imu.writerow((time.time() - start_time, self.imudata[0], self.imudata[2], self.imudata[5], self.imudata[6], self.imudata[7], self.imudata[1]))
            writer_a_imu.writerow((time.time() - start_time, self.imudata[0], self.imudata[2], self.imudata[3], self.imudata[4], self.imudata[5], self.imudata[6], self.imudata[7]))
            time.sleep(0.01)
            # if (0.01-time.time()+loop_start_time) > 0:
            #     time.sleep(0.01-time.time()+loop_start_time)
test = Test()
