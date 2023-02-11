import sys
sys.path.append(sys.path[0] + '/../../')
import param
from sensors import GPS
# import Adafruit_BBIO.GPIO as GPIO
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):
        # Test GPS
        number_samples_GPS = 6000
        #raw_input('Input the number of samples of press ENTER for 50 samples for the GPS test, move the GPS for the reading! ')
        gps = GPS('')
        start_time = time.time()
        if number_samples_GPS is not 0:
        # Setup CSV file
            results_gps = open(sys.path[0]+'/%s-SensorTest_Lukas_GPS.csv' % timestr, 'w')
            writer_gps = csv.writer(results_gps)
            # writer_gps.writerow(('Time (s)', 'x (m)', 'y (m)', 'latitude', 'longitude', 'status', 'timestamp'))
            writer_gps.writerow(['Time (s)', 'x (m)', 'y (m)', 'latitude', 'longitude', 'status', 'timestamp'])
        for x in range(1,int(number_samples_GPS)+1):
            time_read_gps = time.time()
            gpspos = gps.get_position()
            time_read_gps = time.time() - time_read_gps

            writer_gps.writerow((time.time() - start_time, gpspos[0], gpspos[1], gpspos[2], gpspos[3], gpspos[4], gpspos[5]))

            time.sleep(0.1)

            # print(gps.ser_gps.inWaiting())
            # gps.ser_gps.flushInput()
            # gps.ser_gps.flushOutput()

            print('Time=%f\tTimeReadGPS=%f\tx=%g\ty = %g\tlat = %g\tlon = %g' % (time.time() - start_time, time_read_gps, gpspos[0], gpspos[1], gpspos[2], gpspos[3]))
            # Write to CSV file

test = Test()
