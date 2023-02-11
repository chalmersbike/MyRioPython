import sys
sys.path.append(sys.path[0]+'/../../')
import param
from sensors import DualLaserRanger
import time
import csv
roller_width = 380  # milimeter

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")

class Test(object):
    def __init__(self):
        number_samples_Laser_ranger = raw_input('Input the number of samples for the Laser Ranger test, move the bike body for the reading! ')
        self.laser_ranger = DualLaserRanger()
        start_time = time.time()
        time_last_read = 0
        if number_samples_Laser_ranger is not 0:
            # Setup CSV file
            results_laser_ranger = open('./%s-SensorTest_Lukas_Laser.csv' % timestr, 'wb')
            #writer_laser_ranger = csv.writer(results_laser_ranger)
            #writer_laser_ranger.writerow(('Time (s)', 'Laser reading 1', 'Laser reading 2', 'Position'))
        for x in range(1, int(number_samples_Laser_ranger) + 1):
            reading1 = self.laser_ranger.tof1.get_distance()
            reading2 = self.laser_ranger.tof2.get_distance()
            if reading1 < 1000 and reading2 < 1000:
                bike_y = (roller_width / 2 - reading1) / 1000.0 if reading1 >= reading2 else (reading2 - roller_width / 2) / 1000.0
            else:
                print
                "Warning: The laser reading exceeds the roller, neglected"
            print 'Time = %g\t Time loop = %g\t Laser reading 1 = %g\t Laser reading 2 = %g\t Position = %g\t ' % (time.time() - start_time, time.time()-time_last_read, reading1, reading2, bike_y)
            # Write to CSV file
            #writer_laser_ranger.writerow((time.time() - start_time, reading1 , reading2, bike_y))
            time.sleep(0.05)
            time_last_read = time.time()
test = Test()

