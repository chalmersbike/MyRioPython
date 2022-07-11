import sys
sys.path.append(sys.path[0] + '/../')
import param
from sensors import GPS
import Adafruit_BBIO.GPIO as GPIO
import time
import csv
import pysnooper
# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
INPUT_PORT = 'P9_15'
GPIO.setup(INPUT_PORT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# @pysnooper.snoop()
class Test(object):
    def __init__(self):
        # Test GPS
        number_samples_GPS = 288000
        #raw_input('Input the number of samples of press ENTER for 50 samples for the GPS test, move the GPS for the reading! ')
        gps = GPS()
        start_time = time.time()

        if number_samples_GPS is not 0:
        # Setup CSV file
            results_gps = open(sys.path[0]+'/ExpData/%s-RTKGPS-RECORD.csv' % timestr, 'wb')
            writer_gps = csv.writer(results_gps)
            writer_gps.writerow(('Time (s)', 'x (m)', 'y (m)', 'latitude', 'longitude', 'status', 'timestamp',
                                 'Speed', 'Course', 'Date', 'ModeRMC', 'NAVStatus', 'NSIndicator', 'EWIndicator',
                                 'MagneticVariation', 'MagneticVariationEW', 'StrMotorSwitch'))
        exp_start = time.time()
        for x in range(1,int(number_samples_GPS)+1):
            steer_Switch = GPIO.input(INPUT_PORT)
            read_start = time.time()
            gpspos = gps.get_position()
            while abs(gpspos[2]) < 50 or abs(gpspos[2]) > 60:
                gpspos = gps.get_position()

            print(gpspos)
            writer_gps.writerow((time.time() - start_time, gpspos[0], gpspos[1], gpspos[2], gpspos[3], gpspos[4], gpspos[5],
                                 gpspos[6], gpspos[7], gpspos[8], gpspos[9], gpspos[10], gpspos[11], gpspos[12],
                                 gpspos[13], gpspos[14], steer_Switch))
            # print time.time()- read_start
            timeleft = (time.time() - exp_start) % 0.1
            if timeleft < 0.1:
                print(steer_Switch)
                print(0.1 - timeleft)
                time.sleep(0.1 - timeleft)



            # print(gps.ser_gps.inWaiting())
            # gps.ser_gps.flushInput()
            # gps.ser_gps.flushOutput()

            # print 'Time=%f\tx=%g\ty = %g\t' % (time.time() - start_time, gpspos[0], gpspos[1])
            # Write to CSV file

test = Test()
