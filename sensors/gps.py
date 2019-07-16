import Adafruit_BBIO.UART as UART
import serial
import math
import time

# Select the port
port = "/dev/ttyO5"

# Select the sampling time
samplingTime = 0.2  # secs

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


class GPS(object):

    def __init__(self):
        # Open the serial connection at the default baudrate of 9600
        self.ser1 = serial.Serial(port, 9600, timeout=1000)

        # Change the baud rate to 115200 bps
        self.ser1.write(PMTK_SET_BAUD_115200 + "\r\n")

        # Restart the serial connection
        self.ser1.close()
        self.ser1 = serial.Serial(port, 115200, timeout=1000)

        # Choose the type of the GPS data output
        # self.ser1.write(PMTK_SET_NMEA_OUTPUT_RMCGGA+ "\r\n")   # turn on GPRMC and GPGGA
        self.ser1.write(PMTK_SET_NMEA_OUTPUT_RMCONLY + "\r\n") # turn on only the GPRMC sentence

        # Choose the frequency of the GPS data output
        self.ser1.write(PMTK_SET_NMEA_UPDATE_5HZ + "\r\n") # select a 5Hz output rate
        self.ser1.flush()

        self.latitude = 0
        self.longitude = 0
        self.found_satelite = 1
        while ((self.latitude == 0) and (self.longitude == 0)):
            self.lat_ini, self.lon_ini = self.get_latlon()


    def get_position(self):
        lat, lon = self.get_latlon()
        if self.found_satelite == 1:
            self.dx = 1000 * (lon - self.lon_ini) * 40000 * math.cos((lat + self.lat_ini) * math.pi / 360) / 360
            self.dy = 1000 * (lat - self.lat_ini) * 40000 / 360

        return self.dx, self.dy


    def get_latlon(self):
        readall = self.ser1.readline().split('\r\n')  # Read data from the GPS
        # print 'Communication costs ' + str(ReadFintime-starttime) # print communication time

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
            print 'No available Satelites, automatically set longitude and latitude to be ZERO \n Wait for a while or CHANGE your POSITION'
            self.latitude = 0
            self.longitude = 0
            self.found_satelite = 0



        return self.latitude, self.longitude


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
            self.UTC = rmc[1]
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
            print "Bad GPS data"
            #print "Exception happens"
            #print line