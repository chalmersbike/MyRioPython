import csv
import sys
import time

sys.path.append('../')

from bike import Bike

TIME_CONSTANT = 0.1

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('function_timer-%s.csv' % timestr, 'wb')

try:
    bike = Bike(debug=True)

    writer = csv.writer(RESULTS)
    writer.writerow(('time', 'exec_time', 'velocity_time', 'imu_time', 'handlebar_angle_time'))  # headers
    time_count = 0.0

    raw_input('Press ENTER to start')

    while True:  # seconds
        tick1 = time.time()
        bike.get_velocity()
        tock1 = time.time() - tick1

        tick2 = time.time()
        bike.get_imu_data()
        # bike.get_roll_angle()
        # bike.get_roll_angular_velocity()
        tock2 = time.time() - tick2

        tick3 = time.time()
        bike.get_handlebar_angle()
        tock3 = time.time() - tick3

        exec_time = tock1 + tock2 + tock3

        if exec_time < TIME_CONSTANT:
           time.sleep(TIME_CONSTANT - exec_time)

        time_count += time.time() - tick1
        print time_count
        writer.writerow((time_count, exec_time, tock1, tock2, tock3))  # writer.writerow((time_count, phi_d, phi)

except KeyboardInterrupt, SystemExit:
    print 'BREAK'
    RESULTS.close()

finally:
    RESULTS.close()
