from param import *
from constants import *
from utils import *
from PID import PID
import csv
import math
import time
import numpy as np
import pysnooper
# from scipy import signal
import bisect
import traceback
from datetime import datetime
import glob
import os
import dubins
import codecs
import pyvisa
import warnings

# Raise Numpy errors
np.seterr(all = "raise")

# @pysnooper.snoop()
class Controller(object):
    # @pysnooper.snoop()
    def __init__(self, bike, recordPath=False, reverse=False, straight=False,path_file_arg='',rollref_file_arg='',steeringdist_file_arg='',simulate_file=''):
        self.bike = bike

        self.recordPath = recordPath
        self.reverse = reverse
        self.straight = straight
        self.path_file_arg = path_file_arg
        self.rollref_file_arg = rollref_file_arg
        self.steeringdist_file_arg = steeringdist_file_arg
        self.simulate_file = simulate_file

        # Load data from previous experiment if simulating to debug the code
        if self.simulate_file != '':
            print("Loading data from experiment %s ..." % (self.simulate_file))
            # self.simulate_data = np.genfromtxt(self.simulate_file,
            #     names=True,  # If `names` is True, the field names are read from the first valid line
            #     delimiter=",",  # tab separated values
            #     skip_header = 1, # Skip first line of experiment description
            #     dtype=None,  # guess the dtype of each column
            #     )

            self.simulate_data = np.genfromtxt(("\t".join(i).encode('ascii') for i in csv.reader(open(self.simulate_file))),names=True,delimiter="\t",skip_header=1,dtype=None)

            self.idx_simulate_data = 0
            self.simulate_data_RealTime = self.simulate_data['RealTime'].astype('U13')
            self.simulate_data_Time = self.simulate_data['Time']
            self.simulate_data_CalculationTime = self.simulate_data['CalculationTime']
            self.simulate_data_MeasuredVelocity = self.simulate_data['MeasuredVelocity']
            self.simulate_data_FilteredVelocity = self.simulate_data['FilteredVelocity']
            self.simulate_data_BalancingGainsInner = self.simulate_data['BalancingGainsInner']
            self.simulate_data_BalancingGainsOuter = self.simulate_data['BalancingGainsOuter']
            self.simulate_data_Roll = self.simulate_data['Roll']
            self.simulate_data_SteeringAngle = self.simulate_data['SteeringAngle']
            self.simulate_data_RollRate = self.simulate_data['RollRate']
            self.simulate_data_ControlInput = self.simulate_data['ControlInput']
            self.simulate_data_BalancingSetpoint = self.simulate_data['BalancingSetpoint']
            self.simulate_data_gy = self.simulate_data['gy']
            self.simulate_data_gz = self.simulate_data['gz']
            self.simulate_data_ax = self.simulate_data['ax']
            self.simulate_data_ay = self.simulate_data['ay']
            self.simulate_data_az = self.simulate_data['az']
            self.simulate_data_imu_read_timing = self.simulate_data['imu_read_timing']
            self.simulate_data_SteerMotorCurrent = self.simulate_data['SteerMotorCurrent']
            self.simulate_data_v_estimated = self.simulate_data['v_estimated']
            self.simulate_data_yaw_estimated = self.simulate_data['yaw_estimated']
            self.simulate_data_heading_estimated = self.simulate_data['heading_estimated']
            self.simulate_data_x_estimated = self.simulate_data['x_estimated']
            self.simulate_data_y_estimated = self.simulate_data['y_estimated']
            self.simulate_data_GPS_timestamp = self.simulate_data['GPS_timestamp']
            self.simulate_data_x_GPS = self.simulate_data['x_GPS']
            self.simulate_data_y_GPS = self.simulate_data['y_GPS']
            self.simulate_data_theta_GPS = self.simulate_data['theta_GPS']
            self.simulate_data_theta_GPS_noUnwrap = self.simulate_data['theta_GPS_noUnwrap']
            self.simulate_data_latitude = self.simulate_data['latitude']
            self.simulate_data_longitude = self.simulate_data['longitude']
            self.simulate_data_lat_estimated = self.simulate_data['lat_estimated']
            self.simulate_data_lon_estimated = self.simulate_data['lon_estimated']
            self.simulate_data_status = self.simulate_data['status']
            self.simulate_data_NMEA_timestamp = self.simulate_data['NMEA_timestamp']
            try:
                self.simulate_data_DirectionGains = self.simulate_data['DirectionGains']
                self.simulate_data_LateralPositionGains = self.simulate_data['LateralPositionGains']
                self.simulate_data_distance_travelled = self.simulate_data['distance_travelled']
                self.simulate_data_x_ref = self.simulate_data['x_ref']
                self.simulate_data_y_ref = self.simulate_data['y_ref']
                self.simulate_data_heading_ref = self.simulate_data['heading_ref']
                self.simulate_data_lateral_error = self.simulate_data['lateral_error']
                self.simulate_data_heading_error = self.simulate_data['heading_error']
            except:
                pass

        self.variable_init()

        # Check Estop before starting the experiment
        if not self.recordPath:
            self.initial_Estop_Check()  # Check if the Estop Engaged

        # Load path
        if self.path_file_arg == '':
            if path_file != 'newest':
                self.path_file_arg = 'paths/' + path_file
            else:
                self.path_file_arg = 'newest'
        if path_tracking and self.path_file_arg != 'pot' and not self.recordPath:
            try:
                if self.path_file_arg == 'newest':
                    list_of_files = glob.glob('./paths/*.csv')
                    self.latest_file = max(list_of_files, key=os.path.getctime)
                    print("Loading newest path %s ..." % (self.latest_file))
                    self.path_data = np.genfromtxt(self.latest_file, delimiter=",", skip_header=1)
                else:
                    print("Loading path %s ..." % (self.path_file_arg))
                    # self.path_data = np.genfromtxt('paths/' + self.path_file_arg, delimiter=",", skip_header=1)
                    self.path_data = np.genfromtxt(self.path_file_arg, delimiter=",")
                self.path_time = self.path_data[:,0]
                # if self.path_data[0, 3] == 0:
                if self.path_data.shape[1] <= 3:
                    # Case if the path in defined in (x,y,psi)
                    self.path_x = self.path_data[:,1]
                    self.path_y = self.path_data[:,2]
                    self.path_heading = self.path_data[:,3]
                else:
                    # Case if the path in defined in (lat,lon)

                    # Extract unique timestamps
                    _, indices = np.unique(self.path_data[:, -1], return_index=True)
                    indices = indices[:-1]
                    self.path_data = self.path_data[indices,:]

                    # Compute XY path
                    self.path_lat = self.path_data[:,3]
                    self.path_lon = self.path_data[:,4]
                    print(self.path_lat)
                    # Reverse path is needed
                    if self.reverse:
                        print('Reading path in reverse ...')
                        self.path_lon = self.path_lon[::-1]
                        self.path_lat = self.path_lat[::-1]

                    self.path_x = R * self.path_lon * deg2rad * np.cos(self.path_lat[0] * deg2rad)
                    self.path_y = R * self.path_lat * deg2rad
                    print(self.path_lon)
                    print(self.path_lat)
                    print(self.path_x)
                    print(self.path_y)
                    print(self.path_heading)
                    self.path_x = self.path_x - self.path_x[0]
                    self.path_y = self.path_y - self.path_y[0]
                    # print(self.path_x)
                    # print(self.path_y)
                    # print(self.path_heading)
                    # self.path_x = self.path_x - self.bike.drive_gps_joint.x0
                    # self.path_y = self.path_y - self.bike.drive_gps_joint.y0

                    # Downsample from 10Hz to 1Hz
                    self.path_x = self.path_x[0::10]
                    self.path_y = self.path_y[0::10]

                    if self.straight:
                        print('Using only first and last points of path to get a straight line ...')
                        self.path_x = np.array([self.path_x[0],self.path_x[-1]])
                        self.path_y = np.array([self.path_y[0],self.path_y[-1]])

                    # Compute heading
                    self.path_heading = np.arctan2(self.path_y[1:] - self.path_y[0:-1], (self.path_x[1:] - self.path_x[0:-1]))

                    self.path_heading = np.append(self.path_heading, self.path_heading[-1])
                    self.path_heading = np.unwrap(self.path_heading)

                print("Path loaded, loading roll reference if needed, otherwise starting experiment ...")
            except Exception as e:
                print(e)
                print("Path file not found, setting all path references to 0 as default")
                # self.path_data = np.array([[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]) # Using two rows with zeros for np.interp to work
                self.path_data = np.array([[0.0,0.0,0.0,0.0],[1000000.0,0.0,0.0,0.0]]) # Using two rows with zeros for np.interp to work
                self.path_time = self.path_data[:,0]
                self.path_x = self.path_data[:,1]
                self.path_y = self.path_data[:,2]
                self.path_heading = self.path_data[:,3]

            self.path_distanceTravelled = np.cumsum(np.sqrt((self.path_x[1:] - self.path_x[0:-1])**2 + (self.path_y[1:] - self.path_y[0:-1])**2))
            self.path_distanceTravelled = np.append(0,self.path_distanceTravelled)

            # print(self.path_x)
            # print(self.path_y)
            # print(self.path_heading)

            if path_end == 'uturn':
                print("Adding a U-turn at the end of the reference path to bring the bike back to its start point")
                # if path_end_uturn_left:
                #     q0_dubins = (self.path_x[-1], self.path_y[-1], self.path_heading[-1] + 0.1)
                # else:
                #     q0_dubins = (self.path_x[-1], self.path_y[-1], self.path_heading[-1] - 0.1)
                q0_dubins = (self.path_x[-1], self.path_y[-1], self.path_heading[-1])

                idx_path_currentDistanceTravelled = bisect.bisect_right(self.path_distanceTravelled,self.path_distanceTravelled[-1] - path_end_uturn_distanceEndPath ) + np.array([-1, 0])
                x1_dubins = np.interp(self.path_distanceTravelled[-1] - path_end_uturn_distanceEndPath ,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_x[idx_path_currentDistanceTravelled])
                y1_dubins = np.interp(self.path_distanceTravelled[-1] - path_end_uturn_distanceEndPath ,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_y[idx_path_currentDistanceTravelled])
                theta1_dubins = np.interp(self.path_distanceTravelled[-1] - path_end_uturn_distanceEndPath ,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_heading[idx_path_currentDistanceTravelled]) - np.pi
                q1_dubins = (x1_dubins,y1_dubins,theta1_dubins)

                q2_dubins = (self.path_x[0],self.path_y[0],self.path_heading[0] - np.pi)

                self.path_dubins01 = dubins.shortest_path(q0_dubins, q1_dubins, path_end_uturn_radius)
                if path_end_uturn_left:
                    self.path_dubins01 = dubins.path(q0_dubins, q1_dubins, path_end_uturn_radius,dubins.LSR)
                else:
                    self.path_dubins01 = dubins.path(q0_dubins, q1_dubins, path_end_uturn_radius,dubins.RSL)
                self.configurations_dubins01, _ = self.path_dubins01.sample_many(path_end_uturn_stepSize)
                self.configurations_dubins01 = zip(*self.configurations_dubins01)

                self.path_dubins12 = dubins.shortest_path(q1_dubins, q2_dubins, path_end_uturn_radius)
                self.configurations_dubins12, _ = self.path_dubins12.sample_many(path_end_uturn_stepSize)
                self.configurations_dubins12 = zip(*self.configurations_dubins12)

                self.path_x = np.append(self.path_x,self.configurations_dubins01[0] + self.configurations_dubins12[0])
                self.path_y = np.append(self.path_y,self.configurations_dubins01[1] + self.configurations_dubins12[1])
                self.path_heading = np.append(self.path_heading,self.configurations_dubins01[2] + self.configurations_dubins12[2])
                self.path_heading = np.unwrap(self.path_heading)
                # self.path_heading = np.unwrap(self.path_heading)


                self.path_distanceTravelled = np.cumsum(np.sqrt((self.path_x[1:] - self.path_x[0:-1]) ** 2 + (self.path_y[1:] - self.path_y[0:-1]) ** 2))
                self.path_distanceTravelled = np.append(0, self.path_distanceTravelled)

            print(self.path_x)
            print(self.path_y)
            print(self.path_heading)

        # Load roll reference
        if self.rollref_file_arg == '':
            self.rollref_file_arg = 'rollref/' + rollref_file
        if rollref_file != 'nofile':
            print("Loading roll reference %s ..." % (self.rollref_file_arg))
            try:
                # self.rollref_data = np.genfromtxt('rollref/' + self.rollref_file_arg, delimiter=",", skip_header=1)
                self.rollref_data = np.genfromtxt(self.rollref_file_arg, delimiter=",", skip_header=1)
                self.rollref_time = self.rollref_data[:, 0]
                self.rollref_roll = self.rollref_data[:, 1] * rollref_multiplier + rollref_offset
                print(self.rollref_roll)
                print("Roll reference loaded, starting experiment.")
            except:
                print([self.rollref_file_arg, "Path file not found, setting roll reference to 0 as default"])
                # self.rollref_data = np.array([[0.0, 0.0], [0.0, 0.0]]) # Using two rows with zeros for np.interp to work
                self.rollref_data = np.array([[0.0, 0.0], [1000000.0, 0.0],]) # Using three rows with zeros for np.interp to work
                self.rollref_time = self.rollref_data[:, 0]
                self.rollref_roll = self.rollref_data[:, 1] * rollref_multiplier + rollref_offset

        # Load steering disturbance
        if self.steeringdist_file_arg == '':
            self.steeringdist_file_arg = 'strratedistbref/' + strdistbref_file
        if strdistbref_file != 'nofile':
            print("Loading steering rate disturbance reference %s ..." % (self.steeringdist_file_arg))
            try:
                self.strdistbref_data = np.genfromtxt(self.steeringdist_file_arg, delimiter=",", skip_header=1)
                self.strdistbref_time = self.strdistbref_data[:, 0]
                self.strdistbref_str = self.strdistbref_data[:, 1] * strdistbref_multiplier
                print(self.strdistbref_str)
                print("Steering rate disturbance reference loaded, starting experiment.")
            except:
                print([self.steeringdist_file_arg, "Steering rate disturbance file not found, setting roll reference to 0 as default"])
                # self.strdistbref_data = np.array([[0.0, 0.0], [0.0, 0.0]]) # Using two rows with zeros for np.interp to work
                self.strdistbref_data = np.array([[0.0, 0.0], [1000000.0, 0.0],]) # Using three rows with zeros for np.interp to work
                self.strdistbref_time = self.strdistbref_data[:, 0]
                self.strdistbref_str = self.strdistbref_data[:, 1] * strdistbref_multiplier

        # Read the IMU complementary filter Phi as the initial phi estimation
        # self.roll = self.bike.get_imu_data(0, self.steeringAngle, self.roll)[0]
        # self.roll = self.bike.get_imu_data(self.velocity, self.steeringAngle, self.roll)[0]



        # Get description (if any) of the experiment
        if not self.recordPath:
            self.descr = input('\nType a description for the experiment if necessary. Press ENTER to start the experiment. ')
        else:
            self.descr = ''
            input('Press enter to start recording a path by walking the bike.')
            
        # Create log file and add header line
        if not self.recordPath:
            self.log_headerline()
        else:
            self.log_headerline_recordPath()

        self.bike.drive_gps_joint.heart_pipe_parent.send('start_heart_beat')
        self.heartbeat_on_flag = True
        print('Heart Beat STARTS!!!')
        self.time_run_start = time.time() # Reset the starting time, BECAUSE Used in gps_read()
        # Verify that NTRIP correction is not too old
        if gps_use:
            self.gps_read()
            if ntrip_correction:
                if self.bike.drive_gps_joint.Mode_rmc != 'R' and self.bike.drive_gps_joint.Mode_rmc != 'F':
                    print(self.bike.drive_gps_joint.Mode_rmc)
                    print('\nGPS: Waited too long before starting the experiment, NTRIP corrections have to be sent again to the GPS.')
                    print('GPS : Reconnecting NTRIP socket...')
                    # self.bike.drive_gps_joint.ntripclient.socket.connect_ex((self.bike.drive_gps_joint.ntripclient.caster, self.bike.drive_gps_joint.ntripclient.port))
                    self.bike.drive_gps_joint.ntripclient.reconnectNTRIPSocket()
                    print('GPS : Waiting for NTRIP corrections to be received and processed by GPS...')
                    self.gps_read()
                    self.bike.drive_gps_joint.write_ntrip(self.gpspos)
                    while self.bike.drive_gps_joint.Mode_rmc != 'R' and self.bike.drive_gps_joint.Mode_rmc != 'F':
                        try:
                            self.gps_read()
                            self.bike.drive_gps_joint.write_ntrip(self.gpspos)
                            time.sleep(1)
                        except KeyboardInterrupt:
                            break
                        except:
                            pass
                    time.sleep(3)
                    print('GPS : GPS ready.')
                # print(self.gps_timestamp)
        # Wait before starting experiment
        print("")
        for i in range(0,int(math.ceil(start_up_interval))):
            time.sleep(1)
            print("Experiment starting in %is" % (int(math.ceil(start_up_interval))-i))
            # if i <= 2:
            #     self.bike.set_velocity(initial_speed) # Gain speed 2 secs in advance
        print("")
        # print("\nExperiment starting in %is\n" % start_up_interval)
        # time.sleep(start_up_interval)

    ####################################################################################################################
    ####################################################################################################################
    # Run bike

    def run(self):
        # Clear PID controller memory
        self.pid_velocity.clear()
        self.pid_balance.clear()
        self.pid_balance_outerloop.clear()
        self.pid_steeringangle.clear()
        self.pid_lateral_position.clear()
        self.pid_direction.clear()

        self.bike.steering_motor.enable()


        # Flush GPS data buffer
        if gps_use and self.simulate_file == '':
            while self.bike.drive_gps_joint.instr_GPS.bytes_in_buffer > 100:
                self.bike.drive_gps_joint.instr_GPS.flush(pyvisa.constants.VI_READ_BUF_DISCARD)
                # Wait 0.1s to receive new GPS data
            time.sleep(0.1)

        if self.simulate_file == '':
            self.time_run_start = time.time()
            # warnings.warn('RESET TIME RUN POINT!!!!')
            # print(self.time_run_start)
        else:
            self.time_run_start = 0.0

        if read_vesc_data:
            self.bike.drive_gps_joint.vescdata_pipe_parent.send('start_vesc_query')
            
        while self.controller_active or not self.gainingSpeedOver_flag:
            if self.simulate_file == '':
                self.time_start_current_loop = time.time()
            else:
                self.time_start_current_loop = self.simulate_data_Time[self.idx_simulate_data]

            try:
                # Get velocity
                self.velocity_previous = self.velocity
                if self.simulate_file == '':
                    self.velocity = self.bike.get_velocity(initial_speed)
                else:
                    self.velocity = self.simulate_data_MeasuredVelocity[self.idx_simulate_data]
                self.velocity_rec = self.velocity
                if abs(self.velocity) > 1.25*abs(initial_speed):
                    print('[%f] WARNING : Measured speed larger than 1.25 times reference speed' % (
                                time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                if self.velocity-self.velocity_previous > 1.5:
                    print('[%f] WARNING : Measured speed change between two samples too large' % (
                            time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                    self.velocity = 1.5 + self.velocity
                    self.velocity = self.velocity_previous
                if (self.time_count >= speed_up_time + walk_time) and self.velocity < 0.8:
                    print('[%f] WARNING : Measured velocity lower than 0.8m/s after speed-up, setting the measured speed to the reference' % (
                            time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                    self.velocity = initial_speed
                if self.broken_speed_flag:
                    self.velocity = initial_speed

                # Get states
                self.time_get_states = time.time()
                self.get_states()
                self.time_get_states = time.time() - self.time_get_states


                self.sensor_reading_time = time.time()
                # Get position from GPS
                if gps_use:
                    # print(time.time(), self.time_run_start,  self.gps_timestamp)
                    # if (((time.time() - self.time_run_start) - self.gps_timestamp) > 1.0 / gps_dataUpdateRate and self.simulate_file=='') or self.gps_timestamp <= 0.01 or (((self.simulate_data_Time[self.idx_simulate_data] - self.time_run_start) - self.gps_timestamp) > 1.0 / gps_dataUpdateRate and self.simulate_file!=''):
                    # if (((time.time() - self.time_run_start) - self.gps_timestamp) > 1.0 / gps_dataUpdateRate and self.simulate_file=='') or self.gps_timestamp <= 0.01:
                    self.gps_read_quotient = ((time.time() - self.time_run_start) ) // (gps_sampling_time)   # The Quotient in GPS time
                    # print('GPS quotient: \n')
                    # print(self.gps_read_quotient, self.gps_read_quotient_old)
                    if (self.gps_read_quotient > self.gps_read_quotient_old and self.simulate_file=='') or self.gps_timestamp <= 0.01:
                        # print(time.time(),self.time_ntrip)
                        if ntrip_correction and time.time() - self.time_ntrip > 5:
                            self.time_ntrip = time.time()
                            self.bike.drive_gps_joint.write_ntrip(self.gpspos)
                            print("NTRIP Requested!!!")
                        self.gps_timestamp_previous = self.gps_timestamp
                        self.gps_read_quotient_old = self.gps_read_quotient

                        # self.bike.get_gps_data()
                        self.time_gps = time.time()
                        self.gps_read()

                        # self.compute_steering_offset_flag = True

                        if self.gps_nmea_timestamp_ini == '000000':
                            self.gps_nmea_timestamp_ini = self.gps_nmea_timestamp
                            # warnings.warn('RESET NEMA INI!!!!!')

                        # # Set flag to compute estimated states at next time step
                        # self.compute_estimators_flag = False

                        # Check if GPS NMEA timestamp is more than 1s away from BeagleBone timestamp
                        if self.simulate_file == '':
                            # print(self.gps_nmea_timestamp)
                            if self.gps_nmea_timestamp is not None and self.gps_nmea_timestamp_ini is not None:
                                # print(self.gps_nmea_timestamp)
                                # print(self.gps_nmea_timestamp_ini)
                                # print(type(self.gps_nmea_timestamp))
                                est_gps_lag = (datetime.strptime(self.gps_nmea_timestamp, '%H%M%S.%f') - datetime.strptime(self.gps_nmea_timestamp_ini, '%H%M%S.%f')).total_seconds() - self.time_count
                                # print(est_gps_lag)
                                if abs(est_gps_lag) > 1 and (est_gps_lag < 4.9 or est_gps_lag > 5.4):
                                    print(est_gps_lag)

                                    print("[%f] WARNING : the GPS NMEA timestamp is more than 1s away from BeagleBone timestamp. Check GPS data, it might be compromised." % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                            else:
                                print(self.gps_nmea_timestamp)
                                print(type(self.gps_nmea_timestamp))
                                print(self.gps_nmea_timestamp_ini)
                                print(
                                    "[%f] WARNING : timestamp received is NONE!!!!" % (
                                        time.time() - self.time_run_start if self.simulate_file == '' else
                                        self.simulate_data_Time[self.idx_simulate_data]))

                        if self.lat_estimated == 0.0:# and self.recordPath:
                            # Set initial conditions of estimators
                            self.lat_estimated = self.lat_measured_GPS
                            self.lon_estimated = self.lon_measured_GPS
                            self.beta = np.arctan((LENGTH_A / LENGTH_B) * np.tan(self.steeringAngle))
                            self.beta_previous = self.beta
                            self.v_estimated = self.velocity
                            self.v_estimated_previous = self.v_estimated
                            self.pos_estimated = self.pos_GPS
                            self.pos_estimated_previous = self.pos_estimated
                            self.x_estimated = np.asscalar(self.pos_estimated[0])
                            self.y_estimated = np.asscalar(self.pos_estimated[1])
                            self.x_estimated_previous = self.x_estimated
                            self.y_estimated_previous = self.y_estimated
                            self.heading_estimated = np.arctan2(self.y_measured_GPS,self.x_measured_GPS)  # Heading computed between initial (0,0) position and position at end of walk
                            self.heading_estimated_previous = self.heading_estimated
                            self.yaw_estimated = self.heading_estimated - self.beta
                            self.yaw_estimated_previous = self.yaw_estimated

                        # Estimate steering angle offset from GPS position
                        if not self.steering_angle_offset_computed_flag:
                            try:
                                # Rotate path to get small heading angle
                                self.y_GPS_rot_previous = self.y_GPS_rot
                                rotMatrix = np.array([[np.cos(-self.theta_measured_GPS), -np.sin(-self.theta_measured_GPS)], [np.sin(-self.theta_measured_GPS), np.cos(-self.theta_measured_GPS)]])
                                self.pos_GPS_rot = np.dot(rotMatrix,self.pos_GPS)
                                self.y_GPS_rot = self.pos_GPS_rot[1,:]

                                # self.velocity_previous = 1.5
                                # self.velocity = 1.5
                                # # self.steeringAngle_previous = 0.02
                                # # self.steeringAngle = 0.02
                                # # self.y_measured_GPS_previous = self.y_measured_GPS - 0.03
                                # # self.gps_timestamp_previous = self.gps_timestamp - 0.1

                                self.steering_angle_offset += (1 / ((LENGTH_A + np.sin(lambda_bike) * self.cumsum_v_dt) / LENGTH_B)) * (((self.y_GPS_rot - self.y_GPS_rot_previous) / (self.velocity_previous * (self.gps_timestamp - self.gps_timestamp_previous))) - (LENGTH_A * self.steeringAngle_previous / LENGTH_B) - ((np.sin(lambda_bike) / LENGTH_B) * self.cumsum_delta_v_dt))
                                self.cumsum_v_dt += self.velocity_previous * (self.gps_timestamp - self.gps_timestamp_previous)
                                self.cumsum_delta_v_dt += self.steeringAngle_previous * self.velocity_previous * (self.gps_timestamp - self.gps_timestamp_previous)

                                self.steering_angle_offset_count = self.steering_angle_offset_count + 1

                                # print(self.velocity_previous)
                                # print(self.steeringAngle_previous)
                                # print((self.gps_timestamp - self.gps_timestamp_previous))
                                # print(self.cumsum_v_dt)
                                # print(self.cumsum_delta_v_dt)
                                # print(self.y_GPS_rot)
                                # print((self.y_GPS_rot - self.y_GPS_rot_previous))
                                # print((1 / ((LENGTH_A + np.sin(lambda_bike) * self.cumsum_v_dt) / LENGTH_B)) * (((self.y_measured_GPS - self.y_measured_GPS_previous) / (self.velocity_previous * (self.gps_timestamp - self.gps_timestamp_previous))) - (LENGTH_A * self.steeringAngle_previous / LENGTH_B) - ((np.sin(lambda_bike) / LENGTH_B) * self.cumsum_delta_v_dt))))
                                # print(self.steering_angle_offset_count)
                                # print(self.steering_angle_offset)

                                self.compute_steering_offset_flag = False
                            except:
                                # self.steering_angle_offset += 0.0
                                # self.steering_angle_offset_count = self.steering_angle_offset_count #+ 1

                                self.compute_steering_offset_flag = False
                else:
                    self.x_measured_GPS = 0.0
                    self.y_measured_GPS = 0.0
                    self.lat_measured_GPS = 0.0
                    self.lon_measured_GPS = 0.0
                    self.gps_timestamp = 0.0


                # Estimate states (v, yaw, heading, x, y)
                # if self.compute_estimators_flag:# and (self.time_count >= walk_time):
                    # self.estimate_states()
                    # self.compute_estimators_flag = False

                # Get laser ranger position (y position on the roller)
                if laserRanger_use:
                    #if (time.time() - self.time_laserranger) > 1.1 * self.bike.laser_ranger.timing:
                    if (time.time() - self.time_laserranger) > 10 * sample_time:
                        self.time_laserranger = time.time()
                        self.y_laser_ranger = self.bike.get_laserRanger_data()
                        if abs(self.y_laser_ranger)>0.25:
                            print('[%f] WARNING : Laser ranger position outside of roller' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                else:
                    self.y_laser_ranger = 0


                # Read the VESC data
                if read_vesc_data:
                    # print(self.bike.get_vesc_data())
                    # self.rpm, self.v_in, self.avg_motor_current, self.avg_input_current = self.bike.get_vesc_data()
                    # print(self.bike.drive_gps_joint.send_out_readings())
                    # self.rpm, self.v_in, self.avg_motor_current, self.avg_input_current = self.bike.drive_gps_joint.send_out_readings()
                    # print(self.rpm)
                    self.rpm, self.v_in, self.avg_motor_current, self.avg_input_current = self.bike.drive_gps_joint.retrieve_vesc_data()

                
                # Compute time needed to read from all sensors
                self.sensor_reading_time = time.time() - self.sensor_reading_time + self.time_get_states

                # if laserRanger_use:
                #     print("Laser ranger reading time : %f" %(time.time()-self.time_laserranger))
                # if gps_use:
                #     print("GPS reading time : %f" %(time.time()-self.time_gps))
                if self.time_count > drive_motor_timeout and self.heartbeat_on_flag and self.velocity > initial_speed-0.1:
                    self.bike.drive_gps_joint.heart_pipe_parent.send('stop_heart_beat')
                    self.heartbeat_on_flag = False
                    print('Stop Heart Beat!')
                elif self.time_count > drive_motor_timeout and self.heartbeat_on_flag and self.velocity < drive_motor_restart_threshold:
                    self.bike.drive_gps_joint.heart_pipe_parent.send('start_heart_beat')
                    self.heartbeat_on_flag = True
                    print('Start Heart Beat!')


                # PID Velocity Control
                if pid_velocity_active:
                    self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                    self.bike.set_velocity(self.pid_velocity_control_signal)

                # Let bike get up to speed for a few seconds before starting controllers
                if (self.time_count < walk_time):
                    if not self.walk_message_printed_flag:
                        self.walk_message_printed_flag = True
                        print("[%f] Please walk the bike as straight as possible and let go of the handle bar as soon as it gets stiff" % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))

                    if self.simulate_file == '':
                        self.bike.steering_motor.disable()

                    # [OLD METHOD - CHANGED TO USE GPS POSITION OVER 5s] Estimate steering angle offset from mean of steering angle during walk
                    # self.steering_angle_offset = self.steering_angle_offset + self.steeringAngle
                    # self.steering_angle_offset_count = self.steering_angle_offset_count + 1

                elif (self.time_count < speed_up_time+walk_time and self.time_count >= walk_time) and not self.gainingSpeedOver_flag:
                    # Do not start controllers until bike ran for enough time to get up to speed
                    # self.bike.steering_motor.enable()
                    if self.simulate_file == '':
                        self.bike.steering_motor.disable()

                    if not self.speed_up_message_printed_flag:
                        self.speed_up_message_printed_flag = True
                        print('[%f] Gaining speed ...' % (time.time() - self.time_run_start) if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data])

                    if not self.recordPath and self.simulate_file == '':
                        self.bike.set_velocity(initial_speed)
                elif (self.time_count >= speed_up_time+walk_time) and not self.gainingSpeedOver_flag:
                    # Compute mean of steering angle during first 5s
                    if not self.steering_angle_offset_computed_flag:
                        self.steering_angle_offset_computed_flag = True
                        try:
                            self.steering_angle_offset = float(self.steering_angle_offset) / self.steering_angle_offset_count
                        except:
                            print('[%f] WARNING : Cannot compute steering angle offset, setting it to 0.0' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                            self.steering_angle_offset = 0.0
                        print('[%f] Steering angle offset : %.2f deg' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data],self.steering_angle_offset))

                        # Set initial conditions of estimators
                        self.beta = np.arctan((LENGTH_A / LENGTH_B) * np.tan(self.steeringAngle))
                        self.beta_previous = self.beta
                        self.v_estimated = self.velocity
                        self.v_estimated_previous = self.v_estimated
                        self.pos_estimated = self.pos_GPS
                        self.pos_estimated_previous = self.pos_estimated
                        self.x_estimated = np.asscalar(self.pos_estimated[0])
                        self.y_estimated = np.asscalar(self.pos_estimated[1])
                        self.x_estimated_previous = self.x_estimated
                        self.y_estimated_previous = self.y_estimated
                        self.heading_estimated = np.arctan2(self.y_measured_GPS,self.x_measured_GPS)  # Heading computed between initial (0,0) position and position at end of walk
                        self.heading_estimated_previous = self.heading_estimated
                        self.yaw_estimated = self.heading_estimated - self.beta
                        self.yaw_estimated_previous = self.yaw_estimated

                        # Set GPS heading
                        # self.theta_measured_GPS = np.arctan2(self.y_measured_GPS,self.x_measured_GPS)
                    # Once enough time has passed, start controller
                    self.gainingSpeedOver_flag = True
                    print('[%f] Gaining speed phase over' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))

                    # Check that speed if high enough
                    if self.velocity < 0.3*initial_speed:
                        print("[%f] WARNING : speed is lower than one third the reference speed. Hall sensors or drive motor might be faulty. Will use reference speed instead of measured speed in calculations." % (time.time() - self.time_run_start) if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data])
                        self.broken_speed_flag = True

                    self.controller_active = True

                    # Clear PID controllers memory
                    self.pid_velocity.clear()
                    self.pid_balance.clear()
                    self.pid_balance_outerloop.clear()
                    self.pid_steeringangle.clear()
                    self.pid_lateral_position.clear()
                    self.pid_direction.clear()

                    # Enable steering motor
                    if not self.recordPath and self.simulate_file == '':
                        self.bike.steering_motor.enable()

                    # Time at which the controller starts running
                    self.time_start_controller = time.time()

                    # Abort experiment if bike is in unsafe conditions at this point
                    # if (abs(self.roll)>20*deg2rad or abs(self.rollRate)>20*deg2rad or abs(self.steeringAngle)>20*deg2rad):
                    if (abs(self.roll) > 20*deg2rad or abs(self.steeringAngle) > 20*deg2rad):
                        exc_msg = ('[%f] Bike is in unsafe conditions, aborting the experiment' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                        print(exc_msg)
                        self.exception_log(-1, exc_msg)
                        break

                    # Reset estimated roll to zero
                    self.roll = 0
                    if self.simulate_file == '':
                        self.bike.imu.phi = 0
                elif self.controller_active:
                    # Check steering angle
                    self.keep_handlebar_angle_within_safety_margins(self.steeringAngle)

                    # Balancing and path tracking control
                    # self.bike.steering_motor.enable()
                    # self.controller_set_handlebar_angular_velocity(0)
                    # self.update_controller_gains()
                    if not self.recordPath:
                        self.get_balancing_setpoint()
                        self.keep_the_bike_stable()


                # Compute time needed to run controllers
                self.control_cal_time = time.time() - self.time_start_current_loop - self.sensor_reading_time
            #except (ValueError, KeyboardInterrupt):
            except Exception as e:
                print("[%f] Number of times sampling time was exceeded : %d" % (self.exceedscount,time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))

                # e = sys.exc_info()[0]
                print("[%f] Detected error :" % (time.time() - self.time_run_start) if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data])
                print(e)
                print(traceback.print_exc())

                self.safe_stop()
                exc_msg = ('[%f] Error or keyboard interrupt, aborting the experiment' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                print(exc_msg)
                self.exception_log(-2, exc_msg)


            # Control Frequency
            self.loop_time = time.time() - self.time_start_current_loop
            if self.simulate_file == '':
                self.time_count = time.time() - self.time_run_start
            else:
                # self.time_count = self.simulate_data_Time[self.idx_simulate_data] - self.simulate_data_Time[self.idx_simulate_data-1]
                self.time_count = self.simulate_data_Time[self.idx_simulate_data]


            if not self.recordPath:
                # Check for ESTOP
                if self.simulate_file == '':
                    self.ESTOP = self.bike.emergency_stop_check()
                    if self.ESTOP:
                        exc_msg = '[%f] Emergency stop pressed, aborting the experiment' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data])
                        print(exc_msg)
                        self.exception_log(0,exc_msg)
                        self.safe_stop()
                        break

                # Check for extreme PHI
                if self.roll > MAX_LEAN_ANGLE or self.roll < MIN_LEAN_ANGLE:
                    self.safe_stop()
                    exc_msg = ('[%f] Exceeded min/max lean (roll) angle abs %3f > abs %3f, aborting the experiment' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data],self.roll, MAX_LEAN_ANGLE))
                    print(exc_msg)
                    self.exception_log(2, exc_msg)


                # End test time condition
                if self.time_count > test_duration:
                    # Print number of times sampling time was exceeded if experiment is aborted early
                    self.safe_stop()
                    exc_msg = ('[%f] Exceeded test duration, aborting the experiment' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                    print(exc_msg)
                    self.exception_log(-1, exc_msg)
                    break

            # Log data
            if not self.recordPath:
                self.log_regular()
            # else:
            else:
                self.log_regular_recordPath()

            # Compute total time for current loop
            self.loop_time = time.time() - self.time_start_current_loop
            # Sleep to match sampling time
            if self.loop_time < sample_time and self.simulate_file == '':
                time.sleep((sample_time - self.loop_time))
                if debug:
                    print('Sleeping...')

            self.time_start_previous_loop = self.time_start_current_loop

            # Distance travelled
            if self.simulate_file == '':
                self.distance_travelled += self.velocity * (time.time() - self.time_start_current_loop)
            else:
                self.distance_travelled += self.velocity * (self.simulate_data_Time[self.idx_simulate_data] - self.simulate_data_Time[max(0,self.idx_simulate_data-1)])

            # Increase index for simulation using prerecorded data
            if self.simulate_file != '':
                self.idx_simulate_data += 1

        # Print number of times sampling time was exceeded after experiment is over
        print("[%f] Number of times sampling time was exceeded : %d" % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data],self.exceedscount))

    ####################################################################################################################
    ####################################################################################################################
    # Stop bike
    def stop(self):
        self.controller_active = False
        if self.simulate_file == '':
            self.bike.stop_all()


    ####################################################################################################################
    ####################################################################################################################
    # Initialize bike variables
    def variable_init(self):
        # Log File Description
        self.descr = 'NoComments'

        # Gaining Speed Phase
        self.walk_message_printed_flag = False
        self.speed_up_message_printed_flag = False
        self.time_run_start = 0.0
        self.gainingSpeedOver_flag = False
        self.broken_speed_flag = False

        self.roll_ref_imp_doneflag1 = False
        self.roll_ref_imp_doneflag2 = False

        # Potentiometer
        self.pot = 0.0

        # Laser Ranger
        self.time_laserranger = 0.0
        self.y_laser_ranger = 0.0

        # GPS
        self.time_gps = 0.0
        self.time_ntrip = 0.0
        self.gps_nmea_timestamp_ini = '000000'
        self.gps_nmea_timestamp = '000000'
        self.gpspos = [0.0,0.0,0.0,0.0,0.0]
        self.x_measured_GPS = 0.0
        self.y_measured_GPS = 0.0
        self.pos_GPS = np.matrix([[self.x_measured_GPS],[self.y_measured_GPS]])
        self.lat_measured_GPS = 0.0
        self.lon_measured_GPS = 0.0
        self.lat_measured_GPS_ini = 'none'
        self.gps_status = 'No status'
        self.gps_timestamp = 0.0
        self.gps_read_quotient = 0
        self.gps_read_quotient_old = 0
        self.theta_measured_GPS = 0.0
        self.theta_measured_GPS_noUnwrap = 0.0
        self.x_measured_GPS_previous = 0.0
        self.y_measured_GPS_previous = 0.0
        self.pos_GPS_previous = np.matrix([[self.x_measured_GPS_previous],[self.y_measured_GPS_previous]])
        # self.x_estimated_Odo = 0.0
        # self.y_estimated_Odo = 0.0
        # self.psi_estimated_Odo = 0.0
        self.speed_m_s = 0.0
        self.course = 0.0
        self.steering_offset = 0.0
        self.steeringAngle_est = 0.0
        self.rollRate_est = 0.0

        # Emergency Stop
        self.ESTOP = False

        # Global Angles and Coordinates
        self.distance_travelled = 0.0
        self.psi_estimated = 0.0
        self.nu_estimated = 0.0
        if PATH_TYPE == 'CIRCLE':
            self.x_estimated = PATH_RADIUS
        else:
            self.x_estimated = 0.0
        self.y_estimated = 0.0

        # Time Measurement and Timing of Loops
        self.time_count = 0.0
        self.loop_time = 0.0
        self.sensor_reading_time = 0.0
        self.control_cal_time = 0.0
        self.exceedscount = 0.0
        self.time_start_controller = 0.0
        # self.time_pathtracking = 0.0
        # self.roll_ref_end_time = roll_ref_end_time
        # self.roll_ref_start_time = roll_ref_start_time
        # if circle_switch is True:
        #     self.roll_ref_start_time1 = roll_ref_start_time1
        #     self.roll_ref_start_time2 = roll_ref_start_time2


        # Bike States
        self.roll = 0.0
        self.roll_gyro = 0.0
        self.rollRate = 0.0
        self.rollRate_prev = 0.0
        self.rollRate_rec = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.velocity = 0.0
        self.velocity_previous = 0.0
        self.AngVel = 0.0
        self.steeringAngle = 0.0
        self.sensor_read_timing = 0.0
        self.steeringCurrent = 0.0
        self.heartbeat_on_flag = False

        # States Estimators
        self.compute_estimators_flag = False
        self.beta = 0.0
        self.v_estimated = 0.0
        self.pos_estimated = 0.0
        self.x_estimated = 0.0
        self.y_estimated = 0.0
        self.lat_estimated = 0.0
        self.lon_estimated = 0.0
        self.yaw_estimated = 0.0
        self.heading_estimated = 0.0
        self.beta_previous = 0.0
        self.v_estimated_previous = 0.0
        self.pos_estimated_previous = 0.0
        self.x_estimated_previous = 0.0
        self.y_estimated_previous = 0.0
        self.yaw_estimated_previous = 0.0
        self.heading_estimated_previous = 0.0
        self.time_estimate_previous = 0.0

        # Steering angle offset esimation
        self.steering_angle_offset = 0
        self.steering_angle_offset_count = 0
        self.steering_angle_offset_computed_flag = True # False leads to calculation, True means unnecessary.
        self.cumsum_v_dt = 0.0
        self.cumsum_delta_v_dt = 0.0
        self.y_GPS_rot_previous = 0.0
        self.y_GPS_rot = 0.0

        # VESC data
        if read_vesc_data:
            self.rpm = 0.0
            self.v_in = 0.0
            self.avg_motor_current = 0.0
            self.avg_input_current = 0.0

        # Controller
        self.controller_active = False

        # Balancing Controller
        self.balancing_setpoint = 0.0
        self.steering_rate = 0.0
        self.steering_rate_previous = 0.0
        self.steering_rate_filt = 0.0
        self.steering_rate_filt_previous = 0.0

        # Path Trakcing Controller
        self.distanceTravelled = 0.0
        self.pos_ref = 0.0
        self.path_data = [0.0,0.0,0.0,0.0]
        self.path_time = 0.0
        self.path_x = 0.0
        self.path_y = 0.0
        self.path_heading = 0.0
        self.x_ref = 0.0
        self.y_ref = 0.0
        self.heading_ref = 0.0
        self.lateral_error = 0.0
        self.heading_error = 0.0
        self.path_tracking_engaged = False
        # self.roll_ref_period_mod_switch = False
        # self.roll_ref_period_mod_switch_time = 0.0

        # Speed lookup table for speed dependant controller gains
        if isinstance(speed_lookup_controllergains,list):
            self.speed_lookup_controllergains_np = np.array(speed_lookup_controllergains)

        # PID Balance Controller
        self.pid_balance = PID()
        self.pid_balance.setSampleTime(pid_balance_sample_time)
        self.pid_balance.setReference(pid_balance_reference)
        self.pid_balance_control_signal = 0.0  # control signal calculated by PID
        # If the gains are lists, prepare a numpy array to compute speed dependant gains
        # otherwise assign the gains to the controller
        if isinstance(pid_balance_P,list):
            self.pid_balance_P_np = np.array(pid_balance_P)
        else:
            self.pid_balance.setKp(pid_balance_P)
        if isinstance(pid_balance_I,list):
            self.pid_balance_I_np = np.array(pid_balance_I)
        else:
            self.pid_balance.setKi(pid_balance_I)
        if isinstance(pid_balance_D,list):
            self.pid_balance_D_np = np.array(pid_balance_D)
        else:
            self.pid_balance.setKd(pid_balance_D)

        # PID Balance Controller Outer Loop
        self.pid_balance_outerloop = PID()
        self.pid_balance_outerloop.setSampleTime(pid_balance_outerloop_sample_time)
        self.pid_balance_outerloop.setReference(pid_balance_outerloop_reference)
        self.pid_balance_outerloop_control_signal = 0.0  # control signal calculated by PID
        # If the gains are lists, prepare a numpy array to compute speed dependant gains
        # otherwise assign the gains to the controller
        if isinstance(pid_balance_outerloop_P,list):
            self.pid_balance_outerloop_P_np = np.array(pid_balance_outerloop_P)
        else:
            self.pid_balance_outerloop.setKp(pid_balance_outerloop_P)
        if isinstance(pid_balance_outerloop_I,list):
            self.pid_balance_outerloop_I_np = np.array(pid_balance_outerloop_I)
        else:
            self.pid_balance_outerloop.setKi(pid_balance_outerloop_I)
        if isinstance(pid_balance_outerloop_D,list):
            self.pid_balance_outerloop_D_np = np.array(pid_balance_outerloop_D)
        else:
            self.pid_balance_outerloop.setKd(pid_balance_outerloop_D)

        # PID Velocity Controller
        self.pid_velocity = PID(pid_velocity_P, pid_velocity_I, pid_velocity_D)
        self.pid_velocity.setSampleTime(pid_velocity_sample_time)
        self.pid_velocity.setReference(pid_velocity_reference)
        self.pid_velocity_control_signal = 0.0  # control signal calculated by PID

        # PID Lateral Position Controller
        self.pid_lateral_position = PID()
        self.pid_lateral_position.setSampleTime(pid_lateral_position_sample_time)
        self.pid_lateral_position.setReference(pid_lateral_position_reference)
        self.pid_lateral_position_control_signal = 0.0  # control signal calculated by PID
        # If the gains are lists, prepare a numpy array to compute speed dependant gains
        # otherwise assign the gains to the controller
        if isinstance(pid_lateral_position_P,list):
            self.pid_lateral_position_P_np = np.array(pid_lateral_position_P)
        else:
            self.pid_lateral_position.setKp(pid_lateral_position_P)
        if isinstance(pid_lateral_position_I,list):
            self.pid_lateral_position_I_np = np.array(pid_lateral_position_I)
        else:
            self.pid_lateral_position.setKi(pid_lateral_position_I)
        if isinstance(pid_lateral_position_D,list):
            self.pid_lateral_position_D_np = np.array(pid_lateral_position_D)
        else:
            self.pid_lateral_position.setKd(pid_lateral_position_D)

        # PID Direction Controller
        self.pid_direction = PID()
        self.pid_direction.setSampleTime(pid_direction_sample_time)
        self.pid_direction.setReference(pid_direction_reference)
        self.pid_direction_control_signal = 0.0  # control signal calculated by PID
        # If the gains are lists, prepare a numpy array to compute speed dependant gains
        # otherwise assign the gains to the controller
        if isinstance(pid_direction_P,list):
            self.pid_direction_P_np = np.array(pid_direction_P)
        else:
            self.pid_direction.setKp(pid_direction_P)
        if isinstance(pid_direction_I,list):
            self.pid_direction_I_np = np.array(pid_direction_I)
        else:
            self.pid_direction.setKi(pid_direction_I)
        if isinstance(pid_direction_D,list):
            self.pid_direction_D_np = np.array(pid_direction_D)
        else:
            self.pid_direction.setKd(pid_direction_D)

        # PID Steering Angle Controller
        self.pid_steeringangle = PID(pid_steeringangle_P, pid_steeringangle_I, pid_steeringangle_D)
        self.pid_steeringangle.setSampleTime(pid_steeringangle_sample_time)
        self.pid_steeringangle.setReference(0.0)
        self.pid_steeringangle_control_signal = 0.0   # control signal calculated by PID


    ####################################################################################################################
    ####################################################################################################################
    # Get bike states
    def get_states(self):
        self.steeringAngle_previous = self.steeringAngle
        if self.simulate_file == '':
            self.steeringAngle = self.bike.get_handlebar_angle()
        else:
            self.steeringAngle = self.simulate_data_SteeringAngle[self.idx_simulate_data]
        if self.steeringAngle > MAX_HANDLEBAR_ANGLE or self.steeringAngle < MIN_HANDLEBAR_ANGLE:
            print('[%f] WARNING : Steering angle exceeded limits' % (
                    time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))

        if self.steering_angle_offset_computed_flag and self.simulate_file == '':
            self.steeringAngle = self.steeringAngle + self.steering_angle_offset

        if self.simulate_file == '':
            self.steeringCurrent = self.bike.steering_motor.read_steer_current()
        else:
            self.steeringCurrent = self.simulate_data_SteerMotorCurrent[self.idx_simulate_data]

        # imu_data = [phi_comp, phi_gyro, gx (phidot), gy, gz, ax, ay, az]
        # self.imu_data = self.bike.get_imu_data(0, self.steeringAngle, self.roll)
        if self.simulate_file == '':
            self.imu_data = self.bike.get_imu_data(self.velocity, self.steeringAngle, self.roll)
        else:
            self.imu_data = (self.simulate_data_Roll[self.idx_simulate_data],
                self.simulate_data_Roll[self.idx_simulate_data],
                self.simulate_data_RollRate[self.idx_simulate_data],
                self.simulate_data_gy[self.idx_simulate_data],
                self.simulate_data_gz[self.idx_simulate_data],
                self.simulate_data_ax[self.idx_simulate_data],
                self.simulate_data_ay[self.idx_simulate_data],
                self.simulate_data_az[self.idx_simulate_data],
                0)

        # Extract states
        # print(self.imu_data)    # DebuggingONLY
        self.roll = self.imu_data[0]
        self.roll_gyro = self.imu_data[1]
        self.rollRate = self.imu_data[2]
        self.gy = self.imu_data[3]
        self.gz = self.imu_data[4]
        self.ax = self.imu_data[5]
        self.ay = self.imu_data[6]
        self.az = self.imu_data[7]
        self.sensor_read_timing = self.imu_data[8]

        self.rollRate_rec = self.rollRate
        # Outlier detection on roll rate
        if abs(self.rollRate) > 20*deg2rad:
            print('[%f] WARNING : Measured roll rate larger than 20deg/s, at %g deg/s' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data], self.rollRate * rad2deg))
            self.rollRate = self.rollRate_prev
            print("rollRate_rec = %f ; rollRate = %f ; rollRate_prev = %f" % (self.rollRate_rec,self.rollRate,self.rollRate_prev))

        self.X_est, self.P_est = self.bike.Klm_Estimator.estimate(self, self.time_count)
        # print(self.X_est[2],self.X_est[1],self.X_est[0])
        self.x_estimated = self.X_est[0]
        self.y_estimated = self.X_est[1]
        self.v_estimated = self.X_est[2]
        self.yaw_estimated = self.X_est[3]
        self.roll = self.X_est[4]
        self.steeringAngle_est = self.X_est[5]
        self.rollRate_est = self.X_est[6]
        self.steering_offset = self.X_est[7]



        # print(self.X_est)

    ####################################################################################################################
    ####################################################################################################################
    # Get position from GPS
    # @pysnoope
    def estimate_states(self):
        if self.simulate_file == '':
            dt = time.time() - self.time_estimate_previous
        else:
            dt = self.simulate_data_Time[self.idx_simulate_data] - self.simulate_data_Time[max(0,self.idx_simulate_data-1)]

        self.beta = np.arctan((LENGTH_A / LENGTH_B) * np.tan(self.steeringAngle))

        # Velocity estimators
        # self.v_estimated_onlyMeasurements = statesEstimators_KvH * self.velocity_rec + statesEstimators_KvGPS * np.linalg.norm(self.pos_GPS - self.pos_GPS_previous) / dt + statesEstimators_KvRef * initial_speed
        self.v_estimated_onlyMeasurements = statesEstimators_KvH * self.velocity + statesEstimators_KvGPS * np.linalg.norm(self.pos_GPS - self.pos_GPS_previous) / dt + statesEstimators_KvRef * initial_speed
        self.v_estimated = (1 - statesEstimators_Kv) * self.v_estimated_previous + statesEstimators_Kv * self.v_estimated_onlyMeasurements

        # Position estimators
        self.statesEstimators_Kxy_theta = statesEstimators_Kxy
        # self.statesEstimators_Kxy_theta = statesEstimators_Kxy * np.matrix([[np.cos(self.yaw_estimated_previous + self.beta_previous) , -np.sin(self.yaw_estimated_previous + self.beta_previous)], [np.sin(self.yaw_estimated_previous + self.beta_previous) , np.cos(self.yaw_estimated_previous + self.beta_previous)]])
        # self.pos_estimated = (np.eye(2) - self.statesEstimators_Kxy_theta) * (self.pos_estimated_previous + self.v_estimated_previous * dt * np.matrix([[np.cos(self.yaw_estimated_previous + self.beta_previous)],[np.sin(self.yaw_estimated_previous + self.beta_previous)]])) \
        #                      + self.statesEstimators_Kxy_theta * (self.pos_GPS_previous + self.v_estimated_previous * dt * np.matrix([[np.cos(self.yaw_estimated_previous + self.beta_previous)],[np.sin(self.yaw_estimated_previous + self.beta_previous)]]))
        self.pos_estimated = (np.eye(2) - self.statesEstimators_Kxy_theta) * (self.pos_estimated_previous + self.v_estimated_previous * dt * np.matrix([[np.cos(self.yaw_estimated_previous + self.beta_previous)],[np.sin(self.yaw_estimated_previous + self.beta_previous)]])) \
                             + self.statesEstimators_Kxy_theta * (self.pos_GPS)

        self.x_estimated = np.asscalar(self.pos_estimated[0])
        self.y_estimated = np.asscalar(self.pos_estimated[1])

        # Yaw angle estimators
        # # self.yaw_estimated = (1 - statesEstimators_Kpsi) * (self.yaw_estimated_previous + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous)) \
        # #                      + statesEstimators_Kpsi * ((np.arctan2(self.y_estimated - self.y_estimated_previous, self.x_estimated - self.x_estimated_previous) - self.beta_previous) + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous))
        # self.yaw_estimated = (1 - statesEstimators_Kpsi) * (self.yaw_estimated_previous + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous)) \
        #                      + statesEstimators_Kpsi * ((np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.x_measured_GPS_previous) - self.beta_previous) + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous))
        self.yaw_estimated = (1 - statesEstimators_Kpsi) * (self.yaw_estimated_previous + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous)) \
                             + statesEstimators_Kpsi * (self.theta_measured_GPS - self.beta)

        self.yaw_estimated = np.asscalar(self.yaw_estimated)
        self.heading_estimated = self.yaw_estimated + self.beta

        # Save variables of previous time step
        self.beta_previous = self.beta
        self.v_estimated_previous = self.v_estimated
        self.pos_estimated_previous = self.pos_estimated
        self.x_estimated_previous = self.x_estimated
        self.y_estimated_previous = self.y_estimated
        self.yaw_estimated_previous = self.yaw_estimated
        self.heading_estimated_previous = self.heading_estimated
        if self.simulate_file == '':
            self.time_estimate_previous = time.time()
        else:
            self.time_estimate_previous = self.simulate_data_Time[self.idx_simulate_data]

        # # Compute estimated lat/lon
        # self.lon_estimated = (self.x_estimated * rad2deg) / R / np.cos(self.lat_measured_GPS_ini * deg2rad) + self.lon_measured_GPS_ini
        # self.lat_estimated = (self.y_estimated * rad2deg) / R + self.lat_measured_GPS_ini


    ####################################################################################################################
    ####################################################################################################################
    # Get position from GPS
    # @pysnooper.snoop()
    def gps_read(self):
        # Store previous position
        self.x_measured_GPS_previous = self.x_measured_GPS
        self.y_measured_GPS_previous = self.y_measured_GPS
        self.pos_GPS_previous = np.array([[self.x_measured_GPS_previous],[self.y_measured_GPS_previous]])

        # Get GPS position
        if self.simulate_file == '':
            self.gpspos = self.bike.drive_gps_joint.get_position()
            # print(self.gpspos)
        else:
            self.gpspos = (self.simulate_data_x_GPS[self.idx_simulate_data],
                self.simulate_data_y_GPS[self.idx_simulate_data],
                self.simulate_data_latitude[self.idx_simulate_data],
                self.simulate_data_longitude[self.idx_simulate_data],
                self.simulate_data_GPS_timestamp[self.idx_simulate_data],
                self.simulate_data_status[self.idx_simulate_data],
                self.simulate_data_GPS_timestamp[self.idx_simulate_data])

        if ((self.gpspos[2] >= 53) and (self.gpspos[2] <= 70) and (self.gpspos[3] >= 8) and (self.gpspos[3] <= 26)):  # The location should be in SWEDEN
            self.x_measured_GPS = self.gpspos[0]
            self.y_measured_GPS = self.gpspos[1]
            self.lat_measured_GPS = self.gpspos[2]
            self.lon_measured_GPS = self.gpspos[3]
            if self.simulate_file == '':
                self.gps_timestamp = self.time_count  # Using the start of the loop
                # self.gps_timestamp = time.time() - self.time_run_start
                # print(self.gps_timestamp)
            else:
                self.gps_timestamp = self.simulate_data_Time[self.idx_simulate_data]
            if self.lat_measured_GPS_ini == 'none':
                self.lat_measured_GPS_ini = self.lat_measured_GPS
                self.lon_measured_GPS_ini = self.lon_measured_GPS
                self.x_measured_GPS_ini = self.x_measured_GPS
                self.y_measured_GPS_ini = self.y_measured_GPS
                # if self.path_lat:
                    # self.lat_offset = self.lat_measured_GPS_ini - self.path_lat[0]
                    # self.lon_offset = self.lon_measured_GPS_ini - self.path_lon[0]
                try:
                    self.x_offset = self.x_measured_GPS_ini
                    self.y_offset = self.y_measured_GPS_ini
                except:
                    self.x_offset = 0.0
                    self.y_offset = 0.0

            self.x_measured_GPS = self.x_measured_GPS - self.x_offset
            self.y_measured_GPS = self.y_measured_GPS - self.y_offset

            # Offset GPS position from center of mass of bike to position of antenna
            # self.x_measured_GPS -= antenna_offset_x * np.cos(self.theta_measured_GPS)
            # self.y_measured_GPS -= antenna_offset_y * np.sin(self.theta_measured_GPS)


            # print('DEBUG : [%f] x = %f ; x_prev = %f ; y = %f ; y_prev = %f' % (time.time() - self.time_run_start,self.x_measured_GPS,self.x_measured_GPS_previous,self.y_measured_GPS,self.y_measured_GPS_previous))
            # self.theta_measured_GPS_noUnwrap = np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.x_measured_GPS_previous)
            if self.time_count > walk_time:
                # Compute GPS heading - needs to be unwraped in case bike turns around or goes in a circle
                self.theta_measured_GPS = np.unwrap(np.array([self.theta_measured_GPS , np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.x_measured_GPS_previous)]))[-1]
                # self.theta_measured_GPS = np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.x_measured_GPS_previous)
            else:
                self.theta_measured_GPS = np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.x_measured_GPS_previous)


            # Remove offset from antenna not being placed on center of mass
            # self.x_mea

            # Read GPS status if it exists
            # if len(self.gpspos) >= 5:
            self.gps_status = self.gpspos[4]

            # Read GPS NMEA timestamp if it exists
            # if len(self.gpspos) >= 6:
            self.gps_nmea_timestamp = self.gpspos[5]
            self.speed_m_s = self.gpspos[6]
            if math.isnan(self.gpspos[7]) :
                course_reading = self.course
            else:
                course_reading = self.gpspos[7]
            # self.course = np.unwrap([self.course, course_reading])[-1]
            self.course = course_reading

            # Save x and y in a single vector
            self.pos_GPS = np.array([[self.x_measured_GPS],[self.y_measured_GPS]])
        else:
            print(self.gpspos)
            print('INVALID GPS RECEIVED!!!!!! CHECK GPS MODULE!!!!!')

            # Update Kalman filter
            # position_kalman = self.bike.Klm_Estimator.estimate(x_measured_GPS, y_measured_GPS, self.states[1], self.velocity)
            # print('x_measured_GPS = %f ; y_measured_GPS = %f ; x_kalman = %f ; y_kalman = %f' % (self.x_measured_GPS, self.y_measured_GPS, position_kalman[0], position_kalman[1]))


    ####################################################################################################################
    ####################################################################################################################
    # Set angular velocity of handlebar / Set steering rate of steering motor
    def controller_set_handlebar_angular_velocity(self, angular_velocity):
        if MAX_HANDLEBAR_ANGLE_ADJUST == 1:
            if self.upperSaturation:
                if angular_velocity > 0:  # we have exceeded the limit and are trying to move beyond it
                    self.bike.set_handlebar_angular_velocity(-HANDLEBAR_CORRECTION_ANGVEL)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            elif self.lowerSaturation:
                if angular_velocity < 0:  # we have exceeded the limit and are trying to move beyond it
                    self.bike.set_handlebar_angular_velocity(HANDLEBAR_CORRECTION_ANGVEL)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            else:
                self.bike.set_handlebar_angular_velocity(angular_velocity)
        else:
            if self.upperSaturation:
                if angular_velocity > 0:  # we have exceeded the limit and are trying to move beyond it
                    self.bike.set_handlebar_angular_velocity(0)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            elif self.lowerSaturation:
                if angular_velocity < 0:  # we have exceeded the limit and are trying to move beyond it
                    self.bike.set_handlebar_angular_velocity(0)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            else:
                self.bike.set_handlebar_angular_velocity(angular_velocity)


    ####################################################################################################################
    ####################################################################################################################
    # Check that steering angle remains within acceptable bounds
    def keep_handlebar_angle_within_safety_margins(self, handlebar_angle):
        if MAX_HANDLEBAR_ANGLE_ADJUST == 1:
            if self.velocity < HIGHSPEEDBOUND:
                max_handlebar_angle = LowSpeedMAX_HANDLEBAR_ANGLE - self.velocity * (
                        LowSpeedMAX_HANDLEBAR_ANGLE - HighSpeedMAX_HANDLEBAR_ANGLE) / HIGHSPEEDBOUND
                min_handlebar_angle = - max_handlebar_angle
            else:
                max_handlebar_angle = HighSpeedMAX_HANDLEBAR_ANGLE
                min_handlebar_angle = - max_handlebar_angle
        else:
            max_handlebar_angle = MAX_HANDLEBAR_ANGLE
            min_handlebar_angle = MIN_HANDLEBAR_ANGLE
        if handlebar_angle > max_handlebar_angle:
            # print('Exceeded MAX_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg))
            self.upperSaturation = True
            # self.bike.stop()
            self.safe_stop()
            exc_msg = '[%f] Exceeded MAX_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data],max_handlebar_angle * rad2deg)
            print(exc_msg)
            self.exception_log(1, exc_msg)
        elif handlebar_angle < min_handlebar_angle:
            # print('Exceeded MIN_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg))
            self.lowerSaturation = True
            # self.bike.stop()
            self.safe_stop()
            exc_msg = '[%f] Exceeded MIN_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data],max_handlebar_angle * rad2deg)
            print(exc_msg)
            self.exception_log(1, exc_msg)
        else:
            self.upperSaturation = False
            self.lowerSaturation = False


    ####################################################################################################################
    ####################################################################################################################
    # Initial Estop check
    def initial_Estop_Check(self):
        if self.simulate_file == '':
            self.ESTOP = self.bike.emergency_stop_check()
        else:
            self.ESTOP = False

        if self.ESTOP:
            print('[%f] Emergency stop pressed, the experiment will be aborted if it is not released now' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
            input_estop = input('Press ENTER to continue')
            if self.ESTOP:
                self.safe_stop()
                exc_msg = '[%f] Emergency stop was not released, aborting the experiment before it starts' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data])
                print(exc_msg)
                self.exception_log(0,exc_msg)


    ####################################################################################################################
    ####################################################################################################################
    # Update controller gains for current forward speed
    def update_controller_gains(self):
        # self.velocity = 4 + math.sin(self.time_count)
        # print('vel = %f' % (self.velocity))

        if len(speed_lookup_controllergains):
            self.idx_speed_lookup = bisect.bisect_right(self.speed_lookup_controllergains_np, self.velocity) + np.array([-1, 0])
            # print('idx_speed_lookup = ' + np.array2string(self.idx_speed_lookup))
            # print(self.speed_lookup_controllergains_np[self.idx_speed_lookup])

        # Balancing controller inner loop
        if isinstance(pid_balance_P, list):
            self.pid_balance.setKp(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_balance_P_np[self.idx_speed_lookup]))
        if isinstance(pid_balance_I, list):
            self.pid_balance.setKi(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_balance_I_np[self.idx_speed_lookup]))
        if isinstance(pid_balance_D, list):
            self.pid_balance.setKd(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_balance_D_np[self.idx_speed_lookup]))

        # Balancing controller outer loop
        if isinstance(pid_balance_outerloop_P, list):
            self.pid_balance_outerloop.setKp(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_balance_outerloop_P_np[self.idx_speed_lookup]))
        if isinstance(pid_balance_outerloop_I, list):
            self.pid_balance_outerloop.setKi(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_balance_outerloop_I_np[self.idx_speed_lookup]))
        if isinstance(pid_balance_outerloop_D, list):
            self.pid_balance_outerloop.setKd(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_balance_outerloop_D_np[self.idx_speed_lookup]))

        # Path tracking lateral position controller
        if isinstance(pid_lateral_position_P, list):
            self.pid_lateral_position.setKp(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_lateral_position_P_np[self.idx_speed_lookup]))
        if isinstance(pid_lateral_position_I, list):
            self.pid_lateral_position.setKi(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_lateral_position_I_np[self.idx_speed_lookup]))
        if isinstance(pid_lateral_position_D, list):
            self.pid_lateral_position.setKd(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_lateral_position_D_np[self.idx_speed_lookup]))

        # Balancing controller inner loop
        if isinstance(pid_direction_P, list):
            self.pid_direction.setKp(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_direction_P_np[self.idx_speed_lookup]))
        if isinstance(pid_direction_I, list):
            self.pid_direction.setKi(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_direction_I_np[self.idx_speed_lookup]))
        if isinstance(pid_direction_D, list):
            self.pid_direction.setKd(np.interp(self.velocity, self.speed_lookup_controllergains_np[self.idx_speed_lookup],
                                             self.pid_direction_D_np[self.idx_speed_lookup]))

        # print('Kpbal = %f ; Kibal = %f ; Kdbal = %f' % (self.pid_balance.Kp,self.pid_balance.Ki,self.pid_balance.Kd))
        # print('Kpbalouter = %f ; Kibalouter = %f ; Kdbalouter = %f' % (self.pid_balance_outerloop.Kp,self.pid_balance_outerloop.Ki,self.pid_balance_outerloop.Kd))


    ####################################################################################################################
    ####################################################################################################################
    # Get bike states references
    # @pysnooper.snoop()
    def get_balancing_setpoint(self):
        if path_tracking and not self.path_tracking_engaged:
            if time.time()-self.time_start_controller > balancing_time or self.simulate_file != '':
                self.path_tracking_engaged = True
                # self.reset_global_angles_and_coordinates()
                print("[%f] Now heading control and/or path tracking is engaged" % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
        if self.path_tracking_engaged:
            # Get reference position and heading
            if self.path_file_arg == 'pot':
                self.x_ref = 0.0
                self.heading_ref = 0.0
                if potentiometer_use:
                    self.pot = ((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 0.2 - 0.1)  # Potentiometer gives a position reference between -0.1m and 0.1m
                    self.y_ref = self.pot
                else:
                    self.y_ref = 0.0
            else:
                # print(self.distance_travelled, self.path_distanceTravelled[-1])
                if self.distance_travelled >= self.path_distanceTravelled[-1]:
                    self.x_ref = self.path_x[-1]
                    self.y_ref = self.path_y[-1]
                    self.heading_ref = self.path_heading[-1]
                else:
                    idx_path_currentDistanceTravelled = bisect.bisect_right(self.path_distanceTravelled,self.distance_travelled)+np.array([-1,0])
                    self.x_ref = np.interp(self.distance_travelled,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_x[idx_path_currentDistanceTravelled])
                    self.y_ref = np.interp(self.distance_travelled,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_y[idx_path_currentDistanceTravelled])
                    self.heading_ref = np.interp(self.distance_travelled,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_heading[idx_path_currentDistanceTravelled])
            # else:
            #     if (time.time() - self.time_start_controller) > self.path_time[-1]:
            #         self.x_ref = self.path_x[-1]
            #         self.y_ref = self.path_y[-1]
            #         self.heading_ref = self.path_heading[-1]
            #     else:
            #         idx_path_currentTime = bisect.bisect_right(self.path_time,time.time() - self.time_start_controller)+np.array([-1,0])
            #         self.x_ref = np.interp(time.time() - self.time_start_controller,self.path_time[idx_path_currentTime],self.path_x[idx_path_currentTime])
            #         self.y_ref = np.interp(time.time() - self.time_start_controller,self.path_time[idx_path_currentTime],self.path_y[idx_path_currentTime])
            #         self.heading_ref = np.interp(time.time() - self.time_start_controller,self.path_time[idx_path_currentTime],self.path_heading[idx_path_currentTime])

            # Compute position and heading errors
            try:
                self.x_error = self.x_ref - self.x_estimated
                self.y_error = self.y_ref - self.y_estimated
                self.heading_error = self.heading_ref - self.heading_estimated
            except:
                self.x_error = 0
                self.y_error = 0
                self.heading_error = 0
            # if virtual_odometer:
            #     # Using virtual Odometer to do heading control/position control
            #     # WHILE it is highly imprecise!!!
            #     self.x_error = self.x_ref - self.x_estimated
            #     self.y_error = self.y_ref - self.y_estimated
            #     self.heading_error = self.heading_ref - self.psi_estimated
            # elif gps_use:
            #     # We are outside so we get the position and heading measurement from the GPS
            #     self.x_error = self.x_ref - self.x_measured_GPS
            #     self.y_error = self.y_ref - self.y_measured_GPS
            #     self.heading_error = self.heading_ref - self.theta_measured_GPS
            # elif laserRanger_use:
            #     # We are on the roller so we neglect the angular error and the lateral error is given by the position
            #     # measured by the laser rangers
            #     self.x_error = 0
            #     self.y_error = self.y_ref - self.y_laser_ranger
            #     self.heading_error = 0
            # else:
            #     self.x_error = 0
            #     self.y_error = 0
            #     self.heading_error = 0

            # print('x_ref = %f ; x_GPS = %f ; x_error = %f' % (self.x_ref,self.x_measured_GPS,self.x_error))
            # print('y_ref = %f ; y_GPS = %f ; y_error = %f' % (self.y_ref,self.y_measured_GPS,self.y_error))
            # print('heading_ref = %f ; psi_GPS = %f ; heading_error = %f' % (self.heading_ref,self.theta_measured_GPS,self.heading_error))
            # print('')

            # Compute lateral and heading errors
            # self.lateral_error = self.x_error * np.sin(self.heading_ref) + self.y_error * np.cos(self.heading_ref)
            self.lateral_error = -self.x_error * np.sin(self.heading_ref) + self.y_error * np.cos(self.heading_ref)
            self.heading_error = self.heading_error
            # self.heading_error = self.heading_error % (2*np.pi)

            # if (time.time() - self.time_pathtracking) > 10 * sample_time:
            if True:
                # self.time_pathtracking = time.time()

                if path_tracking_structure == 'parallel':
                    # PID Lateral Position Controller
                    self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not measurement
                    # PID Direction/Heading Controller
                    self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not measurement
                    # Compute balancing setpoint
                    idx_rollref_currentTime = bisect.bisect_right(self.rollref_time,
                                                                  time.time() - self.time_start_controller) + np.array(
                        [-1, 0])
                    if idx_rollref_currentTime[0] < 0:
                        idx_rollref_currentTime[0] = 0
                    if time.time() - self.time_start_controller >= self.rollref_time[-1]:
                        self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal + self.rollref_roll[-1]
                    else:
                        self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal + np.interp(
                            time.time() - self.time_start_controller,
                            self.rollref_time[idx_rollref_currentTime],
                            self.rollref_roll[idx_rollref_currentTime])
                    # self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal
                elif path_tracking_structure == 'series':
                    # PID Lateral Position Controller
                    self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not measurement
                    if heading_controller:
                        # PID Direction/Heading Controller
                        # self.pid_direction.setReference(lateralError_controller * self.pid_lateral_position_control_signal)
                        self.pid_direction.setReference(-lateralError_controller * self.pid_lateral_position_control_signal)
                        self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not measurement
                        # Compute balancing setpoint
                        idx_rollref_currentTime = bisect.bisect_right(self.rollref_time,
                                                                      time.time() - self.time_start_controller) + np.array(
                            [-1, 0])
                        if idx_rollref_currentTime[0] < 0:
                            idx_rollref_currentTime[0] = 0
                        if time.time() - self.time_start_controller >= self.rollref_time[-1]:
                            self.balancing_setpoint = self.pid_direction_control_signal + self.rollref_roll[-1]
                        else:
                            self.balancing_setpoint = self.pid_direction_control_signal + np.interp(time.time() - self.time_start_controller,
                                                                self.rollref_time[idx_rollref_currentTime],
                                                                self.rollref_roll[idx_rollref_currentTime])
                    else: # Path-Tracking
                        # Compute balancing setpoint
                        idx_rollref_currentTime = bisect.bisect_right(self.rollref_time,
                                                                      time.time() - self.time_start_controller) + np.array(
                            [-1, 0])
                        if idx_rollref_currentTime[0] < 0:
                            idx_rollref_currentTime[0] = 0
                        if time.time() - self.time_start_controller >= self.rollref_time[-1]:
                            self.balancing_setpoint = self.pid_lateral_position_control_signal + self.rollref_roll[-1]
                        else:
                            self.balancing_setpoint = self.pid_lateral_position_control_signal + np.interp(time.time() - self.time_start_controller,
                                                                self.rollref_time[idx_rollref_currentTime],
                                                                self.rollref_roll[idx_rollref_currentTime])
                else:
                    print("Path tracking controller structure choice is not valid : \"%s\"; Using parallel structure as default.\n" % (balancing_controller_structure))
                    # PID Lateral Position Controller
                    self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not measurement
                    # PID Direction/Heading Controller
                    self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not measurement
                    # Compute balancing setpoint
                    self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal

            # Go into circle with constant 6deg roll reference after we reach end of the reference path
            if self.distance_travelled >= self.path_distanceTravelled[-1] and path_end == 'circle':
                self.balancing_setpoint = path_end_circle_rollRef*deg2rad

                # Saturation of balancing setpoint
            self.balancing_setpoint_sat = max(min(self.balancing_setpoint,max_rollref*deg2rad),-max_rollref*deg2rad)
            if self.balancing_setpoint_sat != self.balancing_setpoint:
                print('[%f] WARNING : Balancing setpoint saturated' % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
            self.balancing_setpoint = self.balancing_setpoint_sat + self.rollref_roll[-1]

        elif potentiometer_use:
            self.pot = -((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 2.5 - 1.25) * deg2rad * 2 # Potentiometer gives a position reference between -2.5deg and 2.5deg
            self.balancing_setpoint = self.pot
        elif roll_ref_use:
            # if rollref_file == 'nofile':
            #     self.balancing_setpoint = 0
            # else:  # For pure roll-tracking
            if rollref_file != 'nofile':
                idx_rollref_currentTime = bisect.bisect_right(self.rollref_time, time.time() - self.time_start_controller)+np.array([-1, 0])
                if idx_rollref_currentTime[0]  < 0:
                    idx_rollref_currentTime[0] = 0
                if time.time() - self.time_start_controller >= self.rollref_time[-1]:
                    self.balancing_setpoint = self.rollref_roll[-1]
                else:
                    self.balancing_setpoint = np.interp(time.time() - self.time_start_controller, self.rollref_time[idx_rollref_currentTime], self.rollref_roll[idx_rollref_currentTime])


    ####################################################################################################################
    ####################################################################################################################
    # Balancing controller
    def keep_the_bike_stable(self):
        # self.get_balancing_setpoint()
        if gps_use:
            rollrate_for_control = self.rollRate_est
        else:
            rollrate_for_control = self.rollRate
        if balancing_controller_structure == 'chalmers':
            # Chalmers Controller structure : deltadot = PID(phidot)
            self.pid_balance.setReference(self.balancing_setpoint)

            self.pid_balance_control_signal = self.pid_balance.update(rollrate_for_control)
            self.steering_rate = self.pid_balance_control_signal
        elif balancing_controller_structure == 'technion':
            # Technion Controller Structure : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
            if roll_ref_use:
                self.pid_balance_outerloop.setReference(self.balancing_setpoint)
                self.pid_balance.setReference(self.pid_balance_outerloop.update(self.roll))
            else:
                self.pid_balance.setReference(0)
            self.pid_balance_control_signal = self.pid_balance.update(rollrate_for_control)
            self.steering_rate = self.pid_balance_control_signal
        elif balancing_controller_structure == 'mdh':
            # MDH Controller structure : delta = PID(phi) ; deltadot = PID(delta)
            self.pid_balance.setReference(self.balancing_setpoint)
            self.pid_balance_control_signal = self.pid_balance.update(self.roll)
            self.steering_rate = self.pos2vel(self.pid_balance_control_signal)
        else:
            print("Bike controller structure choice is not valid : \"%s\"; Using Technion's structure as default.\n" % (balancing_controller_structure))
            # Technion Controller Structure : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
            self.pid_balance_outerloop.setReference(self.balancing_setpoint)
            self.pid_balance.setReference(self.pid_balance_outerloop.update(self.roll))
            self.pid_balance_control_signal = self.pid_balance.update(rollrate_for_control)
            self.steering_rate = self.pid_balance_control_signal

        # Low-pass filter
        # Butterworth 1st order 1Hz cutoff
        self.steering_rate_filt = 0.0305 * self.steering_rate + 0.0305 * self.steering_rate_previous + 0.9391 * self.steering_rate_filt_previous
        self.steering_rate_filt_previous = self.steering_rate_filt

        # self.steering_rate = self.steering_rate_filt

        if strdistbref_file != 'nofile':
            # print('SteeringRate Overwritten')
            idx_strdistbref_currentTime = bisect.bisect_right(self.strdistbref_time, time.time() - self.time_start_controller) + np.array([-1, 0])
            if idx_strdistbref_currentTime[1] >= len(self.strdistbref_time):
                idx_strdistbref_currentTime = np.array([len(self.strdistbref_time) - 2,len(self.strdistbref_time) - 1])
            if idx_strdistbref_currentTime[0] < 0:
                idx_strdistbref_currentTime[0] = 0
            if time.time() - self.time_start_controller >= self.strdistbref_time[-1]:
                self.steering_rate += self.strdistbref_str[-1]
            else:
                self.steering_rate += np.interp(time.time() - self.time_start_controller,
                                                self.strdistbref_time[idx_strdistbref_currentTime],
                                                self.strdistbref_str[idx_strdistbref_currentTime])

        self.steering_rate_previous = self.steering_rate
        # Send Steering Rate Reference value to steering motor controller
        if self.simulate_file == '':
            self.controller_set_handlebar_angular_velocity(self.steering_rate)

        # if self.pid_balance_control_signal > deadband or self.pid_balance_control_signal < -deadband:
        #     self.controller_set_handlebar_angular_velocity(self.steering_rate)
        # else:
        #     self.controller_set_handlebar_angular_velocity(0)


    ####################################################################################################################
    ####################################################################################################################
    # Exit the code safely and put bike on a stable circle with a 5deg constant roll reference
    def safe_stop(self):
        if (abs(self.roll) < 30 * deg2rad and abs(self.steeringAngle) < 40 * deg2rad):
            # print("Bike was in unsafe conditions or the code stopped. Putting the bike in a stable circle with a 5deg constant roll reference.")
            # self.balancing_setpoint = 5*deg2rad*np.sign(self.roll)
            # self.keep_the_bike_stable()
            print("[%f] Bike was in unsafe conditions or the code stopped. Stopping the bike and disabling motors." % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
            self.stop()
        else:
            print("[%f] Conditions too extreme, bike probably felt down, turning off motors." % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
            self.stop()


    ####################################################################################################################
    ####################################################################################################################
    # Steering Angle to Steering Rate Controller
    def pos2vel(self, AngRef):
        self.pid_steeringangle.setReference(AngRef)
        # self.AngVel = self.pid_steeringangle.update(-self.steeringAngle)
        self.AngVel = self.pid_steeringangle.update(self.steeringAngle)
        return self.AngVel


    ####################################################################################################################
    ####################################################################################################################
    # Log data
    def log_headerline(self):
        # Data logging setup
        timestr = time.strftime("%Y%m%d-%H%M%S")
        if self.simulate_file == '':
            csv_path = './ExpData_%s/BikeData_%s.csv' % (bike, timestr)
        else:
            csv_path = './ExpData_%s/BikeData_simulateData_%s.csv' % (bike, timestr)
        print("EXP LOG PATH is: ")
        print(csv_path)
        results_csv = open(csv_path, 'w')
        # results_csv = open('./ExpData_%s/BikeData_%s.csv' % (bike, timestr), 'wb')
        self.writer = csv.writer(results_csv)

        # Create Hexdump of param files
        param_file = open('param.py','rb').read()
        if bike == 'blackbike':
            param_bike_file = open('param_blackbike.py', 'rb').read()
        elif bike == 'redbike':
            param_bike_file = open('param_redbike.py', 'rb').read()
        else:
            param_bike_file = open('param_blackbike.py', 'rb').read()
        param_hexdump = codecs.encode(param_file, 'base64')
        param_bike_hexdump = codecs.encode(param_bike_file, 'base64')



        if path_tracking:
            if self.path_file_arg == 'newest':
                path_file = self.latest_file
            else:
                path_file = self.path_file_arg
            self.writer.writerow(['Description : ' + str(self.descr) + ' ; walk_time = ' + str(walk_time) + ' ; speed_up_time = ' + str(speed_up_time) + ' ; balancing_time = ' + str(balancing_time) , self.reverse , self.straight , self.rollref_file_arg , self.steeringdist_file_arg , path_file , param_hexdump , param_bike_hexdump])
        else:
            path_file = 'non-applicable'
            self.writer.writerow(['Description : ' + str(self.descr) + ' ; walk_time = ' + str(walk_time) + ' ; speed_up_time = ' + str(speed_up_time) + ' ; balancing_time = ' + str(balancing_time) , self.reverse , self.straight , self.rollref_file_arg , self.steeringdist_file_arg , path_file , param_hexdump , param_bike_hexdump])

        self.log_header_str = ['RealTime','Time', 'CalculationTime', 'MeasuredVelocity', 'FilteredVelocity', 'BalancingGainsInner', 'BalancingGainsOuter', 'Roll', 'SteeringAngle', 'RollRate',
                               'ControlInput', 'BalancingSetpoint', 'gy', 'gz', 'ax', 'ay', 'az', 'imu_read_timing', 'SteerMotorCurrent']
        if read_vesc_data:
            self.log_header_str += ['vesc_rpm', 'vesc_v_in', 'vesc_avg_motor_current', 'vesc_avg_input_current']
        
        if potentiometer_use:
            self.log_header_str += ['Potentiometer']
        if gps_use:
            # self.log_header_str += ['v_estimated', 'yaw_estimated', 'heading_estimated', 'x_estimated', 'y_estimated', 'steering_offset', 'GPS_timestamp', 'x_GPS', 'y_GPS', 'theta_GPS', 'theta_GPS_noUnwrap', 'latitude', 'longitude', 'lat_estimated', 'lon_estimated', 'v_gps', 'course_gps','status','NMEA timestamp']
            self.log_header_str += ['v_estimated', 'yaw_estimated', 'x_estimated', 'y_estimated', 'steering_offset', 'GPS_timestamp', 'theta_GPS',  'latitude', 'longitude',  'v_gps', 'course_gps','status','NMEA timestamp','estimated_str_angle','estimated_rollrate']
        if laserRanger_use:
            self.log_header_str += ['laserRanger_dist1', 'laserRanger_dist2', 'laserRanger_y']
        if path_tracking:
            self.log_header_str += ['DirectionGains', 'LateralPositionGains', 'distance_travelled', 'x_ref', 'y_ref', 'heading_ref', 'lateral_error', 'heading_error']

        self.writer.writerow(self.log_header_str)

    def log_regular(self):
        #print("time = %f ; roll = %f ; steering = %f" % (self.time_count,self.roll,self.steeringAngle))

        # Log data
        self.time_log = time.time()
        self.log_str = [
            datetime.now().strftime("%H:%M:%S.%f"),
            "{0:.5f}".format(self.time_count),
            "{0:.5f}".format(self.loop_time),
            "{0:.5f}".format(self.velocity_rec),
            "{0:.5f}".format(self.velocity),
            "[{0:.5f} {1:.5f} {2:.5f}]".format(self.pid_balance.Kp,self.pid_balance.Ki,self.pid_balance.Kd),
            "[{0:.5f} {1:.5f} {2:.5f}]".format(self.pid_balance_outerloop.Kp,self.pid_balance_outerloop.Ki,self.pid_balance_outerloop.Kd),
            "{0:.5f}".format(self.roll),
            "{0:.5f}".format(self.steeringAngle),
            "{0:.5f}".format(self.rollRate_rec),
            "{0:.5f}".format(self.steering_rate),
            "{0:.5f}".format(self.balancing_setpoint),
            "{0:.5f}".format(self.gy),
            "{0:.5f}".format(self.gz),
            "{0:.5f}".format(self.ax),
            "{0:.5f}".format(self.ay),
            "{0:.5f}".format(self.az),
            "{0:.5f}".format(self.sensor_read_timing),
            "{0:.5f}".format(self.steeringCurrent)
        ]
        if read_vesc_data:
            self.log_str += ["{0:.5f}".format(self.rpm),
                             "{0:.4f}".format(self.v_in),
                             "{0:.5f}".format(self.avg_motor_current),
                             "{0:.4f}".format(self.avg_input_current)]


        if potentiometer_use:
            self.log_str += ["{0:.5f}".format(self.pot)]
        if gps_use:
            self.log_str += [
                "{0:.5f}".format(self.v_estimated),
                "{0:.5f}".format(self.yaw_estimated), # "{0:.5f}".format(self.heading_estimated),
                "{0:.5f}".format(self.x_estimated),
                "{0:.5f}".format(self.y_estimated),
                "{0:.5f}".format(self.steering_offset),
                "{0:.5f}".format(self.gps_timestamp), # "{0:.5f}".format(self.x_measured_GPS), # "{0:.5f}".format(self.y_measured_GPS),
                "{0:.5f}".format(self.theta_measured_GPS),                # "{0:.5f}".format(self.theta_measured_GPS_noUnwrap),
                "{0:.8f}".format(self.lat_measured_GPS),
                "{0:.8f}".format(self.lon_measured_GPS),                # "{0:.8f}".format(self.lat_estimated), # "{0:.8f}".format(self.lon_estimated),
                "{0:.8f}".format(self.speed_m_s),
                "{0:.8f}".format(self.course),
                self.gps_status,
                self.gps_nmea_timestamp,
                "{0:.5f}".format(self.steeringAngle_est),
                "{0:.8f}".format(self.rollRate_est)
            ]
            # print(self.x_estimated, self.y_estimated)
            # self.log_str += [
            #     "{0:.5f}".format(self.v_estimated),
            #     "{0:.5f}".format(self.yaw_estimated),
            #     "{0:.5f}".format(self.heading_estimated),
            #     "{0:.5f}".format(self.x_estimated),
            #     "{0:.5f}".format(self.y_estimated),
            #     "{0:.5f}".format(self.steering_offset),
            #     "{0:.5f}".format(self.gps_timestamp),
            #     "{0:.5f}".format(self.x_measured_GPS),
            #     "{0:.5f}".format(self.y_measured_GPS),
            #     "{0:.5f}".format(self.theta_measured_GPS),
            #     "{0:.5f}".format(self.theta_measured_GPS_noUnwrap),
            #     "{0:.8f}".format(self.lat_measured_GPS),
            #     "{0:.8f}".format(self.lon_measured_GPS),
            #     "{0:.8f}".format(self.lat_estimated),
            #     "{0:.8f}".format(self.lon_estimated),
            #     "{0:.8f}".format(self.speed_m_s),
            #     "{0:.8f}".format(self.course),
            #     self.gps_status,
            #     self.gps_nmea_timestamp
            # ]
        if laserRanger_use:
            self.log_str += [
                "{0:.5f}".format(self.bike.laser_ranger.distance1),
                "{0:.5f}".format(self.bike.laser_ranger.distance2),
                "{0:.5f}".format(self.y_laser_ranger)
            ]
        if path_tracking:
            self.log_str += [
                "[{0:.5f},{1:.5f},{2:.5f}]".format(self.pid_direction.Kp,self.pid_direction.Ki,self.pid_direction.Kd),
                "[{0:.5f},{1:.5f},{2:.5f}]".format(self.pid_lateral_position.Kp,self.pid_lateral_position.Ki,self.pid_lateral_position.Kd),
                "{0:.5f}".format(self.distance_travelled),
                "{0:.5f}".format(self.x_ref),
                "{0:.5f}".format(self.y_ref),
                "{0:.5f}".format(self.heading_ref),
                "{0:.5f}".format(self.lateral_error),
                "{0:.5f}".format(self.heading_error)
            ]

        self.writer.writerow(self.log_str)
        self.time_log = time.time() - self.time_log

        if debug or (self.loop_time > sample_time):
            if self.loop_time > sample_time:
                # print("[%f] WARNING : The calculation time exceeds the sampling time!" % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data]))
                self.exceedscount += 1

            # Print sensor reading time, control calculation time, IMU data reading time and logging time
            #     print("[%f] sensor_reading_time \t control calculation \t IMU \t log = %g \t %g \t %g \t %g \t" % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data],
            #                                                                                                        self.sensor_reading_time, self.control_cal_time, self.time_get_states,
            #                                                                                                        self.time_log))

            # Stop experiment if exceeded sampling time too many times
        #     if self.exceedscount > max_exceed_count:
        #         print("")
        #         self.safe_stop()
        #     exc_msg = 'Calculation time exceeded sampling time too often (%d times) , aborting the experiment' % (max_exceed_count)
        #     print(exc_msg)
        #     self.exception_log(3, exc_msg)
        # self.rollRate_prev = self.rollRate


    def reset_global_angles_and_coordinates(self):
        self.x_estimated = initial_speed * (time.time() - self.time_start_controller)
        self.y_estimated = 0.0
        self.psi_estimated = 0.0
        self.nu_estimated = 0.0
        print("[%f] Odometry reset for heading control" % (time.time() - self.time_run_start))

    def exception_log(self,exc_flag,exc_msg):
        if exc_flag is 1:
            exceptioncode = 'Steering Angle exceeds Bound'
        elif exc_flag is 2:
            exceptioncode = 'Roll Angle exceeds Bound'
        elif exc_flag is 0:
            exceptioncode = 'Emergency Pressed'
        elif exc_flag is -1:
            exceptioncode = 'Test finished in the set exp duration'
        elif exc_flag is -2:
            exceptioncode = 'Interruption by the program'
        elif exc_flag is 3:
            exceptioncode = 'samples delayed for too many times'
        else:
            exceptioncode = 'Exception Not LISTED by exception_log()'
        if self.gainingSpeedOver_flag: # To guarantee no error is printed in data file when there is
            self.log_str = [
                "{0:.5f}".format(self.time_count),
                exc_flag,
                exceptioncode,
                exc_msg]
            # self.writer.writerow(self.log_str)

    ####################################################################################################################
    ####################################################################################################################
    # Log data for path recording
    def log_headerline_recordPath(self):
        # Data logging setup
        timestr = time.strftime("%Y%m%d-%H%M%S")
        csv_path = './paths/path_%s.csv' % (timestr)
        print("RECORDED PATH is: ")
        print(csv_path)
        results_csv = open(csv_path, 'w')
        # results_csv = open('./ExpData_%s/BikeData_%s.csv' % (bike, timestr), 'wb')
        self.writer = csv.writer(results_csv)

        self.log_header_str = ['Time', 'lat_estimated', 'lon_estimated', 'latitude', 'longitude', 'RealTime',
                               'CalculationTime', 'MeasuredVelocity', 'FilteredVelocity', 'Roll', 'SteeringAngle', 'RollRate', 'gy', 'gz',
                               'ax', 'ay', 'az', 'imu_read_timing']

        if gps_use:
            self.log_header_str += ['v_estimated', 'yaw_estimated', 'heading_estimated', 'x_estimated',
                                    'y_estimated', 'GPS_timestamp', 'x_GPS', 'y_GPS', 'theta_GPS', 'theta_GPS_noUnwrap', 'status', 'NMEA timestamp']

        self.writer.writerow(self.log_header_str)

    def log_regular_recordPath(self):
        # print("time = %f ; roll = %f ; steering = %f" % (self.time_count,self.roll,self.steeringAngle))

        # Log data
        self.time_log = time.time()
        self.log_str = [
            "{0:.5f}".format(self.time_count),
            "{0:.8f}".format(self.lat_estimated),
            "{0:.8f}".format(self.lon_estimated),
            "{0:.8f}".format(self.lat_measured_GPS),
            "{0:.8f}".format(self.lon_measured_GPS),
            datetime.now().strftime("%H:%M:%S.%f"),
            "{0:.5f}".format(self.loop_time),
            "{0:.5f}".format(self.velocity_rec),
            "{0:.5f}".format(self.velocity),
            "{0:.5f}".format(self.roll),
            "{0:.5f}".format(self.steeringAngle),
            "{0:.5f}".format(self.rollRate_rec),
            "{0:.5f}".format(self.gy),
            "{0:.5f}".format(self.gz),
            "{0:.5f}".format(self.ax),
            "{0:.5f}".format(self.ay),
            "{0:.5f}".format(self.az),
            "{0:.5f}".format(self.sensor_read_timing),
        ]

        if gps_use:
            self.log_str += [
                "{0:.5f}".format(self.v_estimated),
                "{0:.5f}".format(self.yaw_estimated),
                "{0:.5f}".format(self.heading_estimated),
                "{0:.5f}".format(self.x_estimated),
                "{0:.5f}".format(self.y_estimated),
                "{0:.5f}".format(self.gps_timestamp),
                "{0:.5f}".format(self.x_measured_GPS),
                "{0:.5f}".format(self.y_measured_GPS),
                "{0:.5f}".format(self.theta_measured_GPS),
                "{0:.5f}".format(self.theta_measured_GPS_noUnwrap),
                self.gps_status,
                self.gps_nmea_timestamp
            ]

        self.writer.writerow(self.log_str)
        self.time_log = time.time() - self.time_log


        # if debug or (self.loop_time > sample_time):
        #     if self.loop_time > sample_time:
        #         print("[%f] WARNING : The calculation time exceeds the sampling time!" % (time.time() - self.time_run_start) if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data])
        #         self.exceedscount += 1
        #
        #     # Print sensor reading time, control calculation time, IMU data reading time and logging time
        #         print("[%f] sensor_reading_time \t control calculation \t IMU \t log = %g \t %g \t %g \t %g \t" % (time.time() - self.time_run_start if self.simulate_file == '' else self.simulate_data_Time[self.idx_simulate_data],
        #                                                                                                            self.sensor_reading_time, self.control_cal_time, self.time_get_states,
        #                                                                                                            self.time_log))
        #
        #     # Stop experiment if exceeded sampling time too many times
        #     if self.exceedscount > max_exceed_count:
        #         print("")
        #         self.safe_stop()
        #     exc_msg = 'Calculation time exceeded sampling time too often (%d times) , aborting the experiment' % (max_exceed_count)
        #     print(exc_msg)
        #     self.exception_log(3, exc_msg)
        # self.rollRate_prev = self.rollRate