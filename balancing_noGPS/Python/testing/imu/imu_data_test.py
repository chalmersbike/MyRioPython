import csv
import sys
import time
import os
# sys.path.append('../../')
# os.chdir("../../")
# sys.path.append(sys.path[0]+'/../../')
pwd = os.system("pwd")
print (pwd)
from bike import Bike

bike = Bike()

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('imu_data_test-%s.csv' % timestr, 'wb')

try:
    writer = csv.writer(RESULTS)
    writer.writerow(('time', 'phi_d', 'phi'))  # headers
    time_count = 0.0

    raw_input('press ENter to start')
    while True:

        imu_data = bike.get_imu_data()
        print(imu_data)
        # phi_state = imu_data[0]
        # phi_d_state = imu_data[1]
        # print ('%f\t%f\n') % (phi_d_state, phi_state)
        # writer.writerow((time_count, phi_d_state, phi_state))  # writer.writerow((time_count, phi_d, phi)

        #        print phi_d_state
        time.sleep(0.01)
        time_count += 0.01
finally:
    print ("done")
    RESULTS.close()
