from param import *
from sensors import GPS
import Adafruit_BBIO.GPIO as GPIO
import time
import csv
import math
import traceback

# GPS sample time
sample_time = 1.0/gps_dataUpdateRate

# Create GPS object
gps = GPS()

# Setup CSV file
file_name = raw_input('Input the name of the file where the path will be recorded: ')
timestr = time.strftime("%Y%m%d-%H%M%S")

csv_path = './paths/%s-%s.csv' % (file_name,timestr)
results_gps = open(csv_path, 'wb')
writer_gps = csv.writer(results_gps)
# writer_gps.writerow(('Time (s)', 'latitude', 'longitude'))

# Wait before starting experiment
print("")
for i in range(0, int(math.ceil(start_up_interval))):
    time.sleep(1)
    print("Please start walking the bike in %is" % (int(math.ceil(start_up_interval)) - i))
print("")

# Initialize lat/lon
lat_measured_GPS_raw = 0
lon_measured_GPS_raw = 0

while lat_measured_GPS_raw  == 0 or lat_measured_GPS_raw == 0:
    # Get GPS position
    gpspos = gps.get_position()
    lat_measured_GPS_raw = gpspos[2]
    lon_measured_GPS_raw = gpspos[3]

    if ((lat_measured_GPS_raw >= 53) and (lat_measured_GPS_raw <= 70) and (lon_measured_GPS_raw >= 8) and (lon_measured_GPS_raw <= 26)):  # The location should be in SWEDEN
        window_movingAverage = 10 # Samples
        lat_movingAverage = np.full(window_movingAverage,lat_measured_GPS_raw)
        lon_movingAverage = np.full(window_movingAverage,lon_measured_GPS_raw)
        lat_LP = np.average(lat_movingAverage)
        lon_LP = np.average(lon_movingAverage)

        # Write position to CSV file
        writer_gps.writerow((0.0, lat_LP, lon_LP, lat_measured_GPS_raw, lon_measured_GPS_raw))

# Save start time of the recording
start_time = time.time()

while 1:
    try:
        # Save time of the start of the current loop
        time_start_current_loop = time.time()

        # Get GPS position
        gpspos = gps.get_position()
        lat_measured_GPS_raw = gpspos[2]
        lon_measured_GPS_raw = gpspos[3]

        # Check that we are in Sweden
        if ((lat_measured_GPS_raw >= 53) and (lat_measured_GPS_raw <= 70) and (lon_measured_GPS_raw >= 8) and (lon_measured_GPS_raw <= 26)): # The location should be in SWEDEN
            lat_movingAverage = np.append(lat_movingAverage[1:], lat_measured_GPS_raw)
            lon_movingAverage = np.append(lon_movingAverage[1:], lon_measured_GPS_raw)
            lat_LP = np.average(lat_movingAverage)
            lon_LP = np.average(lon_movingAverage)

            # Write position to CSV file
            writer_gps.writerow((time.time() - start_time, lat_LP, lon_LP, lat_measured_GPS_raw, lon_measured_GPS_raw))

        # Compute total time for current loop
        loop_time = time.time() - time_start_current_loop
        # Sleep to match sampling time
        if loop_time < sample_time:
            time.sleep(sample_time - loop_time)
    except Exception as e:
        print('Path file saved : %s' % (csv_path))

        # e = sys.exc_info()[0]
        print("Detected error :")
        print(e)
        print(traceback.print_exc())

        exc_msg = 'Error or keyboard interrupt, aborting the experiment'
        print(exc_msg)