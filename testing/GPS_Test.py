#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
import time
# import adafruit_gps
# import busio
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F"  ///< Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"   ///< Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"              ///<  1 Hz
#define PMTK_SET_NMEA_UPDATE_2HZ  "$PMTK220,500*2B"               ///<  2 Hz
PMTK_SET_NMEA_UPDATE_5HZ = "$PMTK220,200*2C"               #<  5 Hz
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"               ///< 10 Hz
# Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C"  ///< Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"   ///< Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"              ///< 1 Hz
PMTK_API_SET_FIX_CTL_5HZ  = "$PMTK300,200,0,0,0,0*2F"               # < 5 Hz Can't fix position faster than 5 times a second!

PMTK_SET_BAUD_115200 = "$PMTK251,115200*1F"  #< 115200 bps
#define PMTK_SET_BAUD_57600  "$PMTK251,57600*2C"   ///<  57600 bps
#define PMTK_SET_BAUD_9600   "$PMTK251,9600*17"    ///<   9600 bps

#define PMTK_SET_NMEA_OUTPUT_GLLONLY "$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  ///< turn on only the GPGLL sentence
PMTK_SET_NMEA_OUTPUT_RMCONLY = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  #///< turn on only the GPRMC sentence
#define PMTK_SET_NMEA_OUTPUT_VTGONLY "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  ///< turn on only the GPVTG
#define PMTK_SET_NMEA_OUTPUT_GGAONLY "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  ///< turn on just the GPGGA
#define PMTK_SET_NMEA_OUTPUT_GSAONLY "$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  ///< turn on just the GPGSA
#define PMTK_SET_NMEA_OUTPUT_GSVONLY "$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"  ///< turn on just the GPGSV
# PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  #< turn on GPRMC and GPGGA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_OFF     "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"  ///< turn off output


# to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
# such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"          ///< Start logging data
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"            ///< Stop logging data
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"   ///< Acknowledge the start or stop command
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"         ///< Query the logging status
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"        ///< Erase the log flash data
#define LOCUS_OVERLAP 0                               ///< If flash is full, log will overwrite old data with new logs
#define LOCUS_FULLSTOP 1                              ///< If flash is full, logging will stop

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"              ///< Enable search for SBAS satellite (only works with 1Hz output rate)
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"              ///< Use WAAS for DGPS correction data

#define PMTK_STANDBY "$PMTK161,0*28"              ///< standby command & boot successful message
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  ///< Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"              ///< Wake up

#define PMTK_Q_RELEASE "$PMTK605*31"              ///< ask for the release and version


#define PGCMD_ANTENNA "$PGCMD,33,1*6C"            ///< request for updates on antenna status
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"          ///< don't show antenna status messages

#define MAXWAITSENTENCE 10   ///< how long to wait when we're looking for a response
port = "/dev/ttyO5"
baudrate = 9600
# UART.setup("UART1")
write_str = "Hello World\n"
INPUT_PORT = 'P9_24'
OUTPUT_PORT = 'P9_26'
samplingTime = 0.2 # secs
# UART.setup(INPUT_PORT, UART.IN)
# UART.setup(OUTPUT_PORT, UART.OUT)
ser1 = serial.Serial(port, baudrate, timeout=1000)


# ser1.write(PMTK_SET_NMEA_OUTPUT_RMCGGA+ "\r\n")
ser1.write(PMTK_SET_BAUD_115200 + "\r\n")
ser1.flush()
ser1.close()
ser1 = serial.Serial(port, 115200, timeout=1000)
ser1.write(PMTK_SET_NMEA_OUTPUT_RMCONLY + "\r\n")
ser1.write(PMTK_SET_NMEA_UPDATE_5HZ + "\r\n")
# ser1 = busio.UART(OUTPUT_PORT,INPUT_PORT, baudrate = 9600, timeout = 3000)

# gps = adafruit_gps.GPS(ser1, debug=False)
# gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# gps.send_command(b'PMTK220,1000')
# gps.send_command(b'PMTK220,1000')

# ser1.read_until("$GPVTG")+ ser1.readline()

while ser1.isOpen():
    starttime = time.time()

    # print("Serial is open")
    # ser1.write(write_str)
    # print "data has been sent!"
    while ser1.inWaiting():
    #     time.sleep(0.1)
    #     readall = (ser1.read_until("$GPRMC")+ ser1.readline()).split('\r\n')
        readall = ser1.readline().split('\r\n')
        # readall = ((ser1.readline() + ser1.readline())).split('\r\n')
        # line = ser1.readline()
        ReadFintime = time.time()
        print 'Communication costs ' + str(ReadFintime-starttime)
        # print readall
        for i in range(0,len(readall)-1):
            line = readall[i]
            line = line.split("*")
            if len(line) <2:
                print line
            Checksum = line[1]

            line = line[0].split(",", 19)
            # line2 = line2[0].split(",", 17)
            # line3 = line3[0].split(",", 19)
            # line4 = line4[0].split(",", 11)
            # line5 = line5[0].split(",", 9)
            if line[0] == '$GPGGA': # The reading is quite time consuming, not knowing why
                gga = line
                print gga
                utc = gga[1]
                latitude = gga[2]
                ns_Indicator = gga[3]
                longitude = gga[4]
                ew_Indicator = gga[5]
                position_Indicator = gga[6]
                satelite_Used = gga[7]
                hdop = gga[8]
                msl_altitude = gga[9]
                altitude_units = gga[10]
                geoidal_Separation = gga[11]
                separation_unit = gga[12]
                age_of_Diff_Corr = gga[13]  # null when DGPS not used
            elif line[0] == '$GPGSA':
                gsa = line
                print gsa
                mode1 = gsa[1]
                mode2 = gsa[2]
                satelite_used_channel1 = gsa[3]
                satelite_used_channel2 = gsa[4]
                satelite_used_channel3 = gsa[5]
                satelite_used_channel4 = gsa[6]
                satelite_used_channel5 = gsa[7]
                satelite_used_channel6 = gsa[8]
                satelite_used_channel7 = gsa[9]
                satelite_used_channel8 = gsa[10]
                satelite_used_channel9 = gsa[11]
                satelite_used_channel10 = gsa[12]
                satelite_used_channel11 = gsa[13]
                satelite_used_channel12 = gsa[14]
                pdop = gsa[15]
                hdop = gsa[16]
                vdop = gsa[17]
            elif line[0] == '$GPGSV':
                gsv = line
                print gsv
                number_of_message = gsv[1]
                msg1 = gsv[2]
                satelite_in_view = gsv[3]
                if len(gsv) > 7:
                    satelite_id1 = gsv[4]
                    elevation1 = gsv[5]
                    azimuth1 = gsv[6]
                    SNR1 = gsv[7]
                if len(gsv) > 11:
                    satelite_id2 = gsv[8]
                    elevation2 = gsv[9]
                    azimuth2 = gsv[10]
                    SNR2 = gsv[11]
                if len(gsv) > 15:
                    satelite_id3 = gsv[12]
                    elevation3 = gsv[13]
                    azimuth3 = gsv[14]
                    SNR3 = gsv[15]
                if len(gsv) > 19:
                    satelite_id4 = gsv[16]
                    elevation4 = gsv[17]
                    azimuth4 = gsv[18]
                    SNR4 = gsv[19]
            elif line[0] == '$GPRMC':
                rmc = line
                print rmc
                UTC = rmc[1]
                status = rmc[2]
                Latitude = rmc[3]
                NS_indicator = rmc[4]
                Longitude = rmc[5]
                EW_indicator = rmc[6]
                Speed_over_Ground = rmc[7]
                Course_over_Ground = rmc[8]
                Date = rmc[9]
                Magnetic_Variation = rmc[10]
                Mode_rmc = rmc[11]
            elif line[0] == '$GPVTG':
                vtg = line
                print vtg
                Course = vtg[1]
                Reference = vtg[2]
                Course2 = vtg[3]
                Reference2 = vtg[4]
                Speed1 = vtg[5]
                Units1 = vtg[6]
                Speed2 = vtg[7]
                Units2 = vtg[8]
                Mode = vtg[9]
            else:
                print "Exception happens"
                print line
        Total_Time_Cost = time.time()-starttime
        Process_Time = time.time()-ReadFintime
        print Total_Time_Cost - Process_Time
        if line[0] == '$GPVTG':
            time_remain = samplingTime-Total_Time_Cost
            if time_remain > 0:
                ser1.flush()
                time.sleep(time_remain)
ser1.close()
print "Serial is closed"
