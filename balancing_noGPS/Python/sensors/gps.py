from param import *
import Adafruit_BBIO.UART as UART
import serial
import math
import time
import warnings
from NtripClient import NtripClient


########################################################################################################################
########################################################################################################################
# GPS predefined messages
# DO NOT MODIFY

# Define the PMTK configurations strings to send to the GPS according to the NMEA PMTK protocol
# PMTK Packet User Manual       : https://cdn.sparkfun.com/assets/parts/1/2/2/8/0/PMTK_Packet_User_Manual.pdf
# NMEA sentences                : http://aprs.gids.nl/nmea/
# PMTK/NMEA checksum calculator : http://www.hhhh.org/wiml/proj/nmeaxor.html

# Data update rates
PMTK_SET_NMEA_UPDATE_1HZ  = "$PMTK220,1000*1F" # 1 Hz
PMTK_SET_NMEA_UPDATE_2HZ  = "$PMTK220,500*2B"  # 2 Hz
PMTK_SET_NMEA_UPDATE_5HZ  = "$PMTK220,200*2C"  # 5 Hz
PMTK_SET_NMEA_UPDATE_10HZ = "$PMTK220,100*2F"  # 10 Hz

# Baud rate
PMTK_SET_BAUD_115200 = "$PMTK251,115200*1F" # 115200 bps
PMTK_SET_BAUD_57600  = "$PMTK251,57600*2C"  # 57600 bps
PMTK_SET_BAUD_9600   = "$PMTK251,9600*17"   # 9600 bps

# NMEA sentences to output
PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  # turn on GPRMC and GPGGA
PMTK_SET_NMEA_OUTPUT_RMCONLY = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" # turn on only the GPRMC sentence
PMTK_SET_NMEA_OUTPUT_ALLDATA = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28" # turn on ALL THE DATA
PMTK_SET_NMEA_OUTPUT_OFF     = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28" # turn off output
PMTK_SET_NMEA_OUTPUT_GLLONLY = "$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" # turn on only the GPGLL sentence
PMTK_SET_NMEA_OUTPUT_VTGONLY = "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" # turn on only the GPVTG
PMTK_SET_NMEA_OUTPUT_GGAONLY = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" # turn on just the GPGGA
PMTK_SET_NMEA_OUTPUT_GSAONLY = "$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" # turn on just the GPGSA
PMTK_SET_NMEA_OUTPUT_GSVONLY = "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29" # turn on just the GPGSV

# GPS status
PMTK_STANDBY         = "$PMTK161,0*28"     # standby command & boot successful message
PMTK_AWAKE           = "$PMTK010,002*2D"   # Wake up

# Antenna status
PGCMD_ANTENNA   = "$PGCMD,33,1*6C" # request for updates on antenna status
PGCMD_NOANTENNA = "$PGCMD,33,0*6D" # don't show antenna status messages

# DGPS Mode
PMTK_API_SET_DGPS_MODE_OFF = "$PMTK301,0*2C"  # turn off DGPS mode
PMTK_API_SET_DGPS_MODE_RTCM = "$PMTK301,1*2D"  # turn on RTCM DGPS data source mode
PMTK_API_SET_DGPS_MODE_WAAS = "$PMTK301,2*2E"  # turn on WAAS DGPS data source mode

########################################################################################################################
########################################################################################################################



class GPS(object):
    def __init__(self):
        # Open the serial connection at the default baudrate of 9600
        self.ser_gps = serial.Serial(gps_port, 9600, timeout=1)

        if gps_baudrate == 115200:
            # Change the baud rate to 115200 bps
            self.ser_gps.write(PMTK_SET_BAUD_115200 + "\r\n")
        elif gps_baudrate == 57600:
            # Change the baud rate to 57600 bps
            self.ser_gps.write(PMTK_SET_BAUD_57600 + "\r\n")
        elif gps_baudrate == 9600:
            # Change the baud rate to 9600 bps
            self.ser_gps.write(PMTK_SET_BAUD_9600 + "\r\n")
        else:
            print("GPS : Chosen GPS baudrate is not valid : %i. Choosing 115200 instead" %(gps_baudrate))
            # Change the baud rate to 115200 bps
            self.ser_gps.write(PMTK_SET_BAUD_115200 + "\r\n")

        # Restart the serial connection
        self.ser_gps.close()
        self.ser_gps = serial.Serial(gps_port, gps_baudrate, timeout=1000)

        # Choose the type of the GPS data output
        if gps_typeOutputData == 'RMCGGA':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_RMCGGA + "\r\n")    # turn on GPRMC and GPGGA
        elif gps_typeOutputData == 'RMCONLY':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_RMCONLY + "\r\n")   # turn on only the GPRMC sentence
        elif gps_typeOutputData == 'ALLDATA':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_ALLDATA + "\r\n")   # turn on ALL THE DATA
        elif gps_typeOutputData == 'OFF':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_OFF + "\r\n")       # turn off output
        elif gps_typeOutputData == 'GLLONLY':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_GLLONLY + "\r\n")   # turn on only the GPGLL sentence
        elif gps_typeOutputData == 'VTGONLY':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_VTGONLY + "\r\n")   # turn on only the GPVTG
        elif gps_typeOutputData == 'GGAONLY':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_GGAONLY + "\r\n")   # turn on just the GPGGA
        elif gps_typeOutputData == 'GSAONLY':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_GSAONLY + "\r\n")   # turn on just the GPGSA
        elif gps_typeOutputData == 'GSVONLY':
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_GSVONLY + "\r\n")   # turn on just the GPGSV
        else:
            print("GPS : Chosen GPS output data type is not valid : %s. Choosing RMCONLY instead" %(gps_typeOutputData))
            self.ser_gps.write(PMTK_SET_NMEA_OUTPUT_RMCONLY + "\r\n")  # turn on only the GPRMC sentence

        # Choose the frequency of the GPS data output
        if gps_dataUpdateRate == 1:
            self.ser_gps.write(PMTK_SET_NMEA_UPDATE_1HZ + "\r\n")  # select a 1Hz output rate
        elif gps_dataUpdateRate == 2:
            self.ser_gps.write(PMTK_SET_NMEA_UPDATE_2HZ + "\r\n")  # select a 2Hz output rate
        elif gps_dataUpdateRate == 5:
            self.ser_gps.write(PMTK_SET_NMEA_UPDATE_5HZ + "\r\n")  # select a 5Hz output rate
        elif gps_dataUpdateRate == 10:
            self.ser_gps.write(PMTK_SET_NMEA_UPDATE_10HZ + "\r\n")  # select a 10Hz output rate
        else:
            print("GPS : Chosen GPS data update rate is not valid : %iHz. Choosing 10Hz instead" %(gps_dataUpdateRate))
            self.ser_gps.write(PMTK_SET_NMEA_UPDATE_10HZ + "\r\n")  # select a 10Hz output rate

        self.ser_gps.flush()

        # Initialize variables
        self.latitude = 0
        self.longitude = 0
        self.status = 'No status'
        self.utc = 0
        self.found_satellite = 1
        self.dx = 0
        self.dy = 0
        self.x = 0
        self.y = 0
        self.x0 = 0
        self.y0 = 0
        self.lat_ini = 0.0
        self.lon_ini = 0.0
        while ((self.latitude <= 53) or (self.latitude >= 70) or (self.longitude <= 8) or (self.longitude >= 26)): # The location should be in SWEDEN
            self.lat_ini, self.lon_ini = self.get_latlon()
        self.x0 = R * self.lon_ini * deg2rad * math.cos(self.lat_ini * deg2rad)
        self.y0 = R * self.lat_ini * deg2rad
        # print("lat_ini = %f ; lon_ini = %f ; x0 = %f ; y0 = %f" % (lat_ini,lon_ini,x0,y0))
        if debug:
            print('GPS : GPS initialized, obtained initial latitude and longitude')
        print('GPS : GPS initialized, obtained initial latitude and longitude')

        # Initialize NTRIP connection
        if ntrip_correction:
            print("GPS : Using NTRIP to improve GPS accuracy")

            # Choose the DGPS data source mode
            if gps_DPGSDataSourceMode == 'RTCM':
                self.ser_gps.write(PMTK_API_SET_DGPS_MODE_RTCM + "\r\n")  # turn on RTCM DGPS data source mode
            elif gps_typeOutputData == 'WAAS':
                self.ser_gps.write(PMTK_API_SET_DGPS_MODE_WAAS + "\r\n")  # turn on WAAS DGPS data source mode
            else:
                print("GPS : Chosen DGPS data source mode is not valid : %s. Choosing RTCM instead" % (gps_typeOutputData))
                self.ser_gps.write(PMTK_API_SET_DGPS_MODE_RTCM + "\r\n")  # turn on RTCM DGPS data source mode

            self.ntripclient = NtripClient(user=ntrip_username+':'+ntrip_password, caster=ntrip_caster_address, port=ntrip_port,

                                      mountpoint=ntrip_mountpoint, verbose=True, lat=lat_ini, lon=lon_ini, height=12) # Average elevation in Gothenburg is 12m, some NMEA sentences do not carry elevation so it is hard coded here
        else:
            self.ser_gps.write(PMTK_API_SET_DGPS_MODE_OFF + "\r\n")  # turn on RTCM DGPS data source mode

    def get_position(self):
        lat, lon = self.get_latlon()
        if self.found_satellite == 1:
            self.x = R * lon * deg2rad * math.cos(self.lat_ini * deg2rad)
            self.y = R * lat * deg2rad
            self.dx = self.x - self.x0
            self.dy = self.y - self.y0
        else:
            print(warnings.warn("GPS : No Satelite found !"))
        return self.dx, self.dy, lat, lon,self.status, self.utc

    def get_latlon(self):
        if ntrip_correction:
            self.ntrip_data = self.ntripclient.readData()

        # readall = self.ser_gps.readline().split('\r\n')  # Read data from the GPS

        # Read through all received lines until we find the last (most recent) one
        readall = ''
        while self.ser_gps.inWaiting() > 0:
            buffer_string = self.ser_gps.readline().split('\r\n')  # Read data from the GPS
            # print('buffer: %s' % (buffer_string))
            readall = buffer_string

        # print('readall: %s' % (readall))

        # Process data
        for i in range(0, len(readall) - 1):
            line = readall[i]      # Extract one NMEA sentence from the received data
            line = line.split("*") # Remove "*", the final character of a NMEA sentence
            checksum = line[1]     # Get the checksum

            line = line[0].split(",", 19) # Split comma-separated values
            self.process_data(line) # Process data
        # print line
        # print self.latitude
        if self.latitude is not '':
            self.latitude = float(self.latitude)
            self.longitude =  float(self.longitude)
        # Process the latitude and longitude
            self.latitude = int(self.latitude / 100) + (self.latitude % 100) / 60
            self.longitude = int(self.longitude / 100) + (self.longitude % 100) / 60
        else:
            print('GPS : No available Satellites, automatically set longitude and latitude to be ZERO ; Wait for a while or CHANGE your POSITION')
            self.latitude = 0
            self.longitude = 0
            self.found_satellite = 0

        # print line

        return self.latitude, self.longitude

    def get_raw_data(self):
        readall = self.ser_gps.readline().split('\r\n')  # Read data from the GPS
        return readall
        # # Process data
        # for i in range(0, len(readall) - 1):
        #     line = readall[i]      # Extract one NMEA sentence from the received data
        #     line = line.split("*") # Remove "*", the final character of a NMEA sentence
        #     checksum = line[1]     # Get the checksum
        #
        #     line = line[0].split(",", 19) # Split comma-separated values
        # return line

    def process_data(self,line):
        if line[0] == '$GPGGA':
            gga = line
            self.utc = gga[1]
            self.latitude = gga[2]
            self.ns_Indicator = gga[3]
            self.longitude = gga[4]
            self.ew_Indicator = gga[5]
            self.position_Indicator = gga[6]
            self.satelite_Used = gga[7]
            self.hdop = gga[8]
            self.msl_altitude = gga[9]
            self.altitude_units = gga[10]
            self.geoidal_Separation = gga[11]
            self.separation_unit = gga[12]
            self.age_of_Diff_Corr = gga[13]  # null when DGPS not used
        elif line[0] == '$GPGSA':
            gsa = line
            self.mode1 = gsa[1]
            self.mode2 = gsa[2]
            self.satelite_used_channel1 = gsa[3]
            self.satelite_used_channel2 = gsa[4]
            self.satelite_used_channel3 = gsa[5]
            self.satelite_used_channel4 = gsa[6]
            self.satelite_used_channel5 = gsa[7]
            self.satelite_used_channel6 = gsa[8]
            self.satelite_used_channel7 = gsa[9]
            self.satelite_used_channel8 = gsa[10]
            self.satelite_used_channel9 = gsa[11]
            self.satelite_used_channel10 = gsa[12]
            self.satelite_used_channel11 = gsa[13]
            self.satelite_used_channel12 = gsa[14]
            self.pdop = gsa[15]
            self.hdop = gsa[16]
            self.vdop = gsa[17]
        elif line[0] == '$GPGSV':
            gsv = line
            self.number_of_message = gsv[1]
            self.msg1 = gsv[2]
            self.satelite_in_view = gsv[3]
            if len(gsv) > 7:
                self.satelite_id1 = gsv[4]
                self.elevation1 = gsv[5]
                self.azimuth1 = gsv[6]
                self.SNR1 = gsv[7]
            if len(gsv) > 11:
                self.satelite_id2 = gsv[8]
                self.elevation2 = gsv[9]
                self.azimuth2 = gsv[10]
                self.SNR2 = gsv[11]
            if len(gsv) > 15:
                self.satelite_id3 = gsv[12]
                self.elevation3 = gsv[13]
                self.azimuth3 = gsv[14]
                self.SNR3 = gsv[15]
            if len(gsv) > 19:
                self.satelite_id4 = gsv[16]
                self.elevation4 = gsv[17]
                self.azimuth4 = gsv[18]
                self.SNR4 = gsv[19]
        elif line[0] == '$GPRMC':
            rmc = line
            self.utc = rmc[1]
            self.status = rmc[2]
            self.latitude = rmc[3]
            self.NS_indicator = rmc[4]
            self.longitude = rmc[5]
            self.EW_indicator = rmc[6]
            self.Speed_over_Ground = rmc[7]
            self.Course_over_Ground = rmc[8]
            self.Date = rmc[9]
            self.Magnetic_Variation = rmc[10]
            self.Mode_rmc = rmc[11]
        elif line[0] == '$GPVTG':
            vtg = line
            self.Course = vtg[1]
            self.Reference = vtg[2]
            self.Course2 = vtg[3]
            self.Reference2 = vtg[4]
            self.Speed1 = vtg[5]
            self.Units1 = vtg[6]
            self.Speed2 = vtg[7]
            self.Units2 = vtg[8]
            self.Mode = vtg[9]
        else:
            if debug:
                print("GPS : Bad GPS data")
                #print("GPS : Exception happens")
                #print(line)