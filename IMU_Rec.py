from IMU import IMU
import csv
import time
imu = IMU(0)
timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('IMUData-%s.csv' % timestr, 'wb')
writer = csv.writer(RESULTS)
writer.writerow(('Time', 'ax', 'ay', 'az', 'Temp', 'gx', 'gy', 'gz'))
start_time = time.time()
Count = 1
while((time.time() - start_time < 86400) or (Count < 200000)):
    imu_data = imu.IMU_Read()
    t = ("%.4f" % (imu_data[0]-start_time))
    ax = ("%.4f" % imu_data[1])
    ay = ("%.4f" %imu_data[2])
    az = ("%.4f" %imu_data[3])
    Temp = ("%.4f" %imu_data[4])
    gx = ("%.4f" %imu_data[5])
    gy = ("%.4f" %imu_data[6])
    gz = ("%.4f" %imu_data[7])
    writer.writerow((t,ax, ay, az, Temp, gx, gy, gz))
    Count += 1


