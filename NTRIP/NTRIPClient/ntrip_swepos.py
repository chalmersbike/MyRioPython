import sys
sys.path.append(sys.path[0] + '/../../')
import param
from sensors import GPS
import Adafruit_BBIO.GPIO as GPIO
import time
import csv
# from NtripClient import NtripClient
from NtripClient2 import NtripClient

ntripclient = NtripClient(user="ChalmersE2RTK:885511",caster="192.71.190.141",port=80,mountpoint="MSM_GNSS",verbose=True)
# ntripclient = NtripClient(user="ChalmersE2RTK:885511",caster="192.71.190.141",port=80,mountpoint="RTCM3_GNSS",verbose=True)

ntripclient.readData()

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):
        gps = GPS()
        start_time = time.time()
        while 1:
            time_read_gps = time.time()
            gpspos = gps.get_position()
            time_read_gps = time.time() - time_read_gps

            time.sleep(0.1)

            print('Time=%f\tTimeReadGPS=%f\tx=%g\ty = %g\tlat = %g\tlon = %g' % (time.time() - start_time, time_read_gps, gpspos[0], gpspos[1], gpspos[2], gpspos[3]))

test = Test()
