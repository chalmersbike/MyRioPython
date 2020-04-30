from sensors import GPS
import Adafruit_BBIO.GPIO as GPIO
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):
        # Test GPS
        number_samples_GPS = raw_input('Input the number of samples of press ENTER for 50 samples for the GPS test, move the GPS for the reading! ')
        gps = GPS()
        start_time = time.time()
        if number_samples_GPS is not 0:
        # Setup CSV file
            results_gps = open('Tests_Lukas/%s-SensorTest_Lukas_GPS.csv' % timestr, 'wb')
            writer_gps = csv.writer(results_gps)
            writer_gps.writerow(('Time (s)', 'x (m)', 'y (m)', 'latitude', 'longitude'))
        for x in range(1,int(number_samples_GPS)+1):
            gpspos = gps.get_position()
            print 'Time=%f\t Temp=%g\tAx = %g\t' % (time.time() - start_time, gpspos[0], gpspos[1])
            # Write to CSV file
            writer_gps.writerow((time.time() - start_time, gpspos[0], gpspos[1], gps.latitude, gps.longitude))

test = Test()
