from NtripClient import NtripClient
import sys
sys.path.append(sys.path[0] + '/../')
import param
from sensors import GPS
import Adafruit_BBIO.GPIO as GPIO
import time
import csv

# ntripclient = NtripClient(user="ChalmersE2RTK:885511",caster="192.71.190.141",port=80,mountpoint="MSM_GNSS",verbose=True)
# print(ntripclient.readData())


# Initialize GPS
print("GPS : Initializing GPS...")
gps = GPS()
lat_ini = 0.0
lon_ini = 0.0
while not (55.0<lat_ini<59.0 and 10.0<lon_ini<14.0):
    gpspos_ini = gps.get_position()
    lat_ini = gpspos_ini[2]
    lon_ini = gpspos_ini[3]
print("GPS : GPS Initialized.")


# Initialize NTRIP connection
ntrip_correction = True
if ntrip_correction:
    print("GPS : Initializing NTRIP to improve GPS accuracy...")

    # # Choose the DGPS data source mode
    # if gps_DPGSDataSourceMode == 'RTCM':
    #     self.ser_gps.write(PMTK_API_SET_DGPS_MODE_RTCM + "\r\n")  # turn on RTCM DGPS data source mode
    # elif gps_typeOutputData == 'WAAS':
    #     self.ser_gps.write(PMTK_API_SET_DGPS_MODE_WAAS + "\r\n")  # turn on WAAS DGPS data source mode
    # else:
    #     print("GPS : Chosen DGPS data source mode is not valid : %s. Choosing RTCM instead" % (gps_typeOutputData))
    #     self.ser_gps.write(PMTK_API_SET_DGPS_MODE_RTCM + "\r\n")  # turn on RTCM DGPS data source mode

    # self.ntripclient = NtripClient(user=ntrip_username + ':' + ntrip_password, caster=ntrip_caster_address,
    #                                port=ntrip_port,
    #                                mountpoint=ntrip_mountpoint, verbose=True, lat=lat_ini, lon=lon_ini,
    #                                height=12)  # Average elevation in Gothenburg is 12m, some NMEA sentences do not carry elevation so it is hard coded here

    ntripclient = NtripClient(user="ChalmersE2RTK:885511", caster="192.71.190.141", port=80, mountpoint="MSM_GNSS",lat=lat_ini,lon=lon_ini,verbose=True)
# else:
#     self.ser_gps.write((PMTK_API_SET_DGPS_MODE_OFF + "\r\n").encode('utf-8'))  # turn on


start_time = time.time()
while 1:
    if ntrip_correction:
        print('NTRIP Data:')
        time_read_NTRIP = time.time()
        ntrip_correction_data = ntripclient.readData()
        time_read_NTRIP = time.time() - time_read_gps
        print(ntrip_correction_data)
        time_write_NTRIP = time.time()
        gps.write_ntrip(ntrip_correction_data)
        time_write_NTRIP = time.time() - time_write_NTRIP
        print('time_read_NTRIP=%f ; time_write_NTRIP = %f' % (time_read_NTRIP,time_write_NTRIP))

    time_read_gps = time.time()
    gpspos = gps.get_position()
    time_read_gps = time.time() - time_read_gps

    print('Time=%f\tTimeReadGPS=%f\tx=%g\ty = %.6g\tlat = %.6g\tlon = %g' % (time.time() - start_time, time_read_gps, gpspos[0], gpspos[1], gpspos[2], gpspos[3]))
    print()
    print()

    time.sleep(0.1)