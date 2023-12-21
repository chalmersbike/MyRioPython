from param import *
import serial, time, numpy
# from pyvesc import VESC
import pyvesc
import pysnooper
import pyvisa
import signal
import threading
import sys
import multiprocessing as mul
# import pysnooper
import math
import warnings
from sensors.NtripClient import NtripClient
import threading
import pickle
# from queue import Queue
########################################################################################################################
########################################################################################################################
# TO BE CHANGED - THESE DO NOT WORK WITH A UBLOX GPS
#
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
# get_position_result = [0.0, 0.0, 0.0, 0.0, 'A', '000000', 0.0, 0.0]


# @pysnooper.snoop()
class VESC_GPS(object):
    serial = None

    def __init__(self, session):
        self.rm = pyvisa.ResourceManager()
        self.instr_VESC = self.rm.open_resource('ASRL2::INSTR')  # change back
        self.instr_VESC.baud_rate = driveMotor_CommunicationFrequency

        if gps_use:
            # Open the serial connection
            self.instr_GPS = self.rm.open_resource('ASRL1::INSTR', read_termination = '\r\n')
            self.instr_GPS.baud_rate = 460800
            self.instr_GPS.flush(pyvisa.constants.VI_READ_BUF_DISCARD)

            # Initialize variables
            self.latitude = 0
            self.longitude = 0
            self.Speed_over_Ground = 0
            self.Course_over_Ground = 0
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
            self.last_get_latlon = time.time()
            self.lat = 0.0
            self.lon = 0.0
            self.speed_m_s = 0.0
            self.course = 0.0
            self.x0 = 0.0
            self.y0 = 0.0
            self.speed = 0.0
            self.prev_gps_reading = [0.0, 0.0, 0.0, 0.0, 'A', '000000', 0.0, 0.0]
            self.latest_gps_reading = self.prev_gps_reading
            self.getposition_loop_parent, self.getposition_loop_child = mul.Pipe()
            self.readall = ''

        if currentControlVESC:
            self.lookup_table, params_lut = self.load_lookup_table('./actuators/lookup_table_current.pkl')
            self.lut_start, self.lut_stop, self.lut_step = params_lut
            print(self.lut_start, self.lut_stop, self.lut_step)
        else:
            self.lookup_table, params_lut = self.load_lookup_table('./actuators/lookup_table_speed.pkl')
            self.lut_start, self.lut_stop, self.lut_step = params_lut
            print(self.lut_start, self.lut_stop, self.lut_step)

        ## Test Multi-Process
        self.last_vesc_data = [0, 0, 0, 0]
        self.heart_pipe_parent, self.heart_pipe_child = mul.Pipe()
        self.vescdata_pipe_parent, self.vescdata_pipe_child = mul.Pipe()
        if read_vesc_data:
            self.read_sensor_msg = pyvesc.protocol.interface.encode_request(pyvesc.messages.getters.GetValues())
        if gps_use:
            self.instr_GPS.flush(pyvisa.constants.VI_READ_BUF_DISCARD)
            self.heart_beat_process = mul.Process(target=self._heartbeat_vsec_gps_read_dual_thread,
                                                  args=(self.heart_pipe_child, self.vescdata_pipe_child,
                                                        self.getposition_loop_child,),
                                                  daemon=True)
        else:
            self.heart_beat_process = mul.Process(target=self._heartbeat_vsec_gps_read_dual_thread,
                                                  args=(self.heart_pipe_child, self.vescdata_pipe_child,),
                                                  daemon=True)
        self._stop_heartbeat = mul.Event()
        self.heart_beat_process.start()


        if debug:
            print('Drive Motor : Serial port opened')
        self.Time = -1 # Initialize time to -1 as a way to check that we correctly read from the controller after starting the logging


    def stop_heartbeat(self):
        self.heart_pipe_parent.send('stop_heart_beat')
        self._stop_heartbeat.set()
        # self.heart_beat_thread.join()
        self.heart_beat_process.join(3)

    def load_lookup_table(self, filename):
        with open(filename, 'rb') as f:
            data = pickle.load(f)
        return data['lookup_table'], data['params']

    def interpolate(self, lookup_table, start, step, x):
        index = int((x - start) / step)
        return lookup_table[index]

    def rear_set_rpm(self, rpm):
        # self.VESCmotor.set_rpm(int(rpm))
        self.heart_beat_msg = pyvesc.protocol.interface.encode(pyvesc.messages.setters.SetRPM(int(rpm)))
        self.heart_pipe_parent.send(self.heart_beat_msg)


    def set_velocity(self, input_velocity):
        self.rear_set_rpm(input_velocity*600)

    def set_current_binary(self, ref_current):
        current_msg_binary = pyvesc.protocol.interface.encode(pyvesc.messages.setters.SetCurrent(ref_current))
        read_sensor_msg = pyvesc.protocol.interface.encode_request(pyvesc.messages.getters.GetValues())
        return read_sensor_msg + current_msg_binary

    def set_velocity_binary(self, ref_velocity):
        current_msg_binary = pyvesc.protocol.interface.encode(pyvesc.messages.setters.SetRPM(int(ref_velocity * 608.0)))
        read_sensor_msg = pyvesc.protocol.interface.encode_request(pyvesc.messages.getters.GetValues())
        return read_sensor_msg + current_msg_binary

    def stop(self):
        print('VESC : Stop')
        self.set_velocity(0)
        self.stop_heartbeat()

    def retrieve_vesc_data(self):
        while self.vescdata_pipe_parent.poll():
            self.last_vesc_data = self.vescdata_pipe_parent.recv()
        return self.last_vesc_data

    def get_position(self):
        while self.getposition_loop_parent.poll():
            self.latest_gps_reading = self.getposition_loop_parent.recv()
            # self.dx, self.dy, self.lat, self.lon, self.status, self.utc, self.speed_m_s, self.course = self.getposition_loop_parent.recv()
        # print('SENT GPS from get_position() in VESC LIB')
        # print(self.latest_gps_reading)
        return self.latest_gps_reading



    def get_latlon(self):

        while self.instr_GPS.bytes_in_buffer > 0:
            # # buffer_string = self.ser_gps.readline().split('\r\n')  # Read data from the GPS
            buffer_string = self.instr_GPS.read_raw().decode('utf-8',errors='ignore').split('\r\n')  # Read data from the GPS

            self.readall = buffer_string
            # print(buffer_string)
            self.last_get_latlon = time.time()

            # Process data
            for i in range(0, len(self.readall) - 1):
                line = self.readall[i]      # Extract one NMEA sentence from the received data

                # print('GHPS : line: ' + line)

                line = line.split("*") # Remove "*", the final character of a NMEA sentence
                checksum = line[1]     # Get the checksum

                line = line[0].split(",", 19) # Split comma-separated values
                self.process_data(line) # Process data
            # print(line)
            # print(self.latitude)
            if self.latitude is not '':
                self.latitude = float(self.latitude)
                self.longitude =  float(self.longitude)
                self.speed_m_s = 0.514 * float(self.Speed_over_Ground)
                if self.Course_over_Ground is not '':
                    self.course = float(self.Course_over_Ground)
                else:
                    self.course = float('nan')
            # Process the latitude and longitude
                self.latitude = int(self.latitude / 100) + (self.latitude % 100) / 60
                self.longitude = int(self.longitude / 100) + (self.longitude % 100) / 60
            else:
                print('GPS : No available Satellites, automatically set longitude and latitude to be ZERO ; Wait for a while or CHANGE your POSITION')
                self.latitude = 0
                self.longitude = 0
                self.found_satellite = 0

            self.x = R * self.longitude * deg2rad * math.cos(self.lat_ini * deg2rad)
            self.y = R * self.latitude * deg2rad
            self.dx = self.x - self.x0
            if abs(self.dx) > 1000:
                print(self.dx)
                print(self.readall)
        # print('GPS : lat : %f ; lon : %f' % (self.latitude,self.longitude))
        return self.latitude, self.longitude, self.speed_m_s, self.course

    # @pysnooper.snoop()
    def get_position_loop(self, pipe_gps):
        # print("lat_ini = %f ; lon_ini = %f ;" % (self.lat_ini, self.lon_ini))
        self.x0 = R * self.lon_ini * deg2rad * math.cos(self.lat_ini * deg2rad)
        self.y0 = R * self.lat_ini * deg2rad
        # print("lat_ini = %f ; lon_ini = %f ; x0 = %f ; y0 = %f" % (lat_ini,lon_ini,x0,y0))

        # if debug:
        #     print('GPS : GPS initialized, obtained initial latitude and longitude')
        print('GPS : GPS initialized, obtained initial latitude and longitude')

        start_loop = time.time()
        gps_read_quotient = 0
        gps_read_quotient_old = -1
        gps_sample_time = (1.0 / gps_dataUpdateRate)
        # global get_position_result
        while not self.stop_GPS_loop.is_set():
            gps_read_quotient = ((time.time() - start_loop)) // gps_sample_time
            if gps_read_quotient > gps_read_quotient_old:
                # currentReadLoopTime = time.time()
                lat, lon, speed_m_s, course = self.get_latlon()
                gps_read_quotient_old = gps_read_quotient
                # print('READ GPS ONCE!')

                if self.found_satellite == 1:
                    self.x = R * lon * deg2rad * math.cos(self.lat_ini * deg2rad)
                    self.y = R * lat * deg2rad
                    self.dx = self.x - self.x0
                    self.dy = self.y - self.y0
                    # if course is not float('nan'):
                    #     course = ((450 - course) / 360.0 + np.pi) % (2 * np.pi) - np.pi
                    # else:
                    #     course = np.aRXtan2(self.dy, self.dx)
                    if course is float('nan'):
                        course = np.aRXtan2(self.dy, self.dx)
                else:
                    print(warnings.warn("GPS : No Satelite found !"))
                if self.utc is None:
                    get_position_result = self.prev_gps_reading
                else:
                    get_position_result = [self.dx, self.dy, lat, lon, self.status, self.utc, speed_m_s, course]
                    self.prev_gps_reading = get_position_result
                pipe_gps.send(get_position_result)
                # get_position_result = [self.dx, self.dy, lat, lon, self.status, self.utc, speed_m_s, course]
                # print(get_position_result)
                sleep_proposal = gps_sample_time - (time.time() - start_loop - gps_sample_time*gps_read_quotient)
                # print('Read costs:' + str(time.time() - currentReadLoopTime))
                if sleep_proposal > 0.005:
                    time.sleep(sleep_proposal - 0.005)
            else:
                time.sleep(0.01)

    def get_raw_data(self):
        self.readall = self.instr_GPS.read_raw().split('\r\n')  # Read data from the GPS
        return self.readall
    def write_ntrip(self,gpspos):
        # Read NTRIP corrections once every second
        if ntrip_correction:
            ntrip_correction_data = self.ntripclient.readData(gpspos[2], gpspos[3])
            # print(ntrip_correction_data)
            try:
                if len(ntrip_correction_data) > 0:
                    print('Received NTRIP data : %i bits' % (len(ntrip_correction_data)))
                    self.instr_GPS.write_raw(ntrip_correction_data)
                else:
                    print('Received empty NTRIP data, possible data starvation or socket disconnection.')
            except Exception as e:
                print('NTRIP error : ' + str(e))
                print('Received empty NTRIP data, possible data starvation or socket disconnection.')

    def process_data(self,line):
        # print(line)
        if line[0] == '$GPGGA' or line[0] == '$GNGGA':
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
        elif line[0] == '$GPGSA' or line[0] == '$GNGSA':
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
        elif line[0] == '$GPGSV'or line[0] == '$GNGSV':
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
        elif line[0] == '$GPRMC' or line[0] == '$GNRMC':
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
            self.Magnetic_VariationEW = rmc[11]
            self.Mode_rmc = rmc[12]
            self.Nav_status = rmc[13]
        elif line[0] == '$GPVTG' or line[0] == '$GNVTG':
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


### NewThread
    def _heartbeat_vsec_gps_read_dual_thread(self, pipe_heart, pipe_data, pipe_gps):
        """
        Continuous function calling that keeps the motor alive
        """
        # start_heart_beat = False
        # start_vesc_data_query = False
        start_heart_beat = False
        start_vesc_data_query = False
        nr_of_empty_loop = 0
        t_last_vesc_query = time.time()
        t_last_heart = time.time()
        gps_update_current_loop = False
        vesc_query_sent = False
        # global get_position_result
        if gps_use:
            while ((self.latitude <= 53) or (self.latitude >= 70) or (self.longitude <= 8) or (
                    self.longitude >= 26)):  # The location should be in SWEDEN
                self.lat_ini, self.lon_ini, self.speed, self.course = self.get_latlon()

            # print("lat_ini = %f ; lon_ini = %f ;" % (self.lat_ini, self.lon_ini))
            self.x0 = R * self.lon_ini * deg2rad * math.cos(self.lat_ini * deg2rad)
            self.y0 = R * self.lat_ini * deg2rad
            # print("lat_ini = %f ; lon_ini = %f ; x0 = %f ; y0 = %f" % (lat_ini,lon_ini,x0,y0))

            # if debug:
            #     print('GPS : GPS initialized, obtained initial latitude and longitude')
            print('GPS : GPS initialized, obtained initial latitude and longitude')

            start_loop = time.time()
            gps_read_quotient = 0
            gps_read_quotient_old = -1

            gps_sample_time = (1.0 / gps_dataUpdateRate)
            vesc_data_sleep_proposal = 1.0
            t_left_next_beat = 1.0

            # qGPS = Queue()

            self.gps_thread = threading.Thread(target=self.get_position_loop, args=(pipe_gps, ), daemon=True)
            self.stop_GPS_loop = threading.Event()
            self.gps_thread.start()

        pipe_heart_cmd_last_read_t = time.time()
        pipe_data_cmd_last_read_t = time.time()
        self.heart_beat_msg = pyvesc.messages.setters.alive_msg
        while not self._stop_heartbeat.is_set():
            if time.time() - pipe_heart_cmd_last_read_t > 1:
                while pipe_heart.poll():
                    cmd_heart = pipe_heart.recv()
                    type_msg = type(cmd_heart)
                    if type_msg is bytes:
                        # self.instr_VESC.write_raw(cmd_heart)
                        self.heart_beat_msg = cmd_heart
                    elif type_msg is float:
                        self.heart_beat_msg = self.interpolate(self.lookup_table, self.lut_start, self.lut_step, cmd_heart)
                    elif type_msg is str:
                        if cmd_heart == 'start_heart_beat':
                            start_heart_beat = True
                            self.heart_beat_msg = self.interpolate(self.lookup_table, self.lut_start, self.lut_step, 0)
                            print('HEARTBEAT  START!')
                        elif cmd_heart == 'stop_heart_beat':
                            start_heart_beat = False
                            self.heart_beat_msg = self.interpolate(self.lookup_table, self.lut_start, self.lut_step, 0)
                            self.heart_beat_msg = self.read_sensor_msg

                pipe_heart_cmd_last_read_t = time.time()

            t_since_last_beat = time.time() - t_last_heart
            if (t_since_last_beat > 0.01):
                self.instr_VESC.write_raw(self.heart_beat_msg)
                # print('MSG_SENT!')
                t_last_heart = time.time()
                # time.sleep(0.003)
                buffer_size = self.instr_VESC.bytes_in_buffer
                # print(buffer_size)
                if buffer_size < 79:
                    pass
                elif buffer_size < 158: # Only one complete message
                    readraw = self.instr_VESC.read_bytes(79)
                    (self.vesc_sensor_response, consumed) = pyvesc.protocol.interface.decode(readraw)
                    pipe_data.send([self.vesc_sensor_response.rpm,
                                    self.vesc_sensor_response.avg_motor_current,
                                    self.vesc_sensor_response.v_in,
                                    self.vesc_sensor_response.avg_input_current])
                    # print('DATA Received!!!!')
                    if nr_of_empty_loop is not 0:
                        nr_of_empty_loop = 0
                elif buffer_size == 158: # Two complete message
                    readraw = self.instr_VESC.read_bytes(158)  # Bytes type
                    warnings.warn('DOUBLE DATA Received!!!!')
                    readraw = readraw[79:158]
                    t_last_vesc_query = time.time()
                    (self.vesc_sensor_response, consumed) = pyvesc.protocol.interface.decode(readraw)
                    if self.vesc_sensor_response is not None:
                        pipe_data.send([self.vesc_sensor_response.rpm,
                                        self.vesc_sensor_response.avg_motor_current,
                                        self.vesc_sensor_response.v_in,
                                        self.vesc_sensor_response.avg_input_current])
                elif nr_of_empty_loop < 5:
                    nr_of_empty_loop = nr_of_empty_loop + 1
                else:
                    self.instr_VESC.flush(pyvisa.constants.VI_READ_BUF_DISCARD)
                    nr_of_empty_loop = 0
            t_left_next_beat = 0.01 - (time.time() - t_last_heart)
            # print(t_left_next_beat)
            if t_left_next_beat > 0:
                time.sleep(t_left_next_beat)

        self.stop_GPS_loop.set()

