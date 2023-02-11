from param import *
from constants import *
from utils import *

import csv
import math
import time
import numpy as np
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import pysnooper
from scipy import signal
import bisect
import traceback
from datetime import datetime
import glob
import os

# @pysnooper.snoop()
class RecordPathLatLon(object):
    # @pysnooper.snoop()
    def __init__(self, bike):
        self.bike = bike
        self.variable_init()

        # Check Estop before starting the experiment
        self.initial_Estop_Check()  # Check if the Estop Engaged

        # Get experiment (if any) of the experiment
        self.file_name = raw_input('Input the name of the file where the path will be recorded: ')

        # Create log file and add header line
        self.log_headerline()

        # Wait before starting experiment
        print("")
        for i in range(0,int(math.ceil(start_up_interval))):
            time.sleep(1)
            print("Experiment starting in %is" % (int(math.ceil(start_up_interval))-i))
        print("")


    ####################################################################################################################
    ####################################################################################################################
    # Run bike
    # @pysnooper.snoop()
    def run(self):
        # Flush GPS data buffer
        if gps_use:
            while self.bike.gps.ser_gps.inWaiting() > 100:
                self.bike.gps.ser_gps.flushInput()
                # self.bike.gps.ser_gps.flush()
                # Wait 0.1s to receive new GPS data
            time.sleep(0.1)

        self.gaining_speed_start = time.time()
        while self.controller_active or not self.gainingSpeedOver_flag:
            self.time_start_current_loop = time.time()

            try:
                # Get velocity
                self.velocity = self.bike.get_velocity()
                self.velocity_rec = self.velocity
                if self.broken_speed_flag:
                    self.velocity = initial_speed
                if abs(self.velocity) > 1.25*abs(initial_speed):
                    print('WARNING : [%f] Measured speed larger than 1.25 times reference speed' % (
                                time.time() - self.gaining_speed_start))
                if self.velocity-self.velocity_previous > 1.5:
                    print('WARNING : [%f] Measured speed change between two samples too large' % (
                            time.time() - self.gaining_speed_start))
                    self.velocity = 1.5 + self.velocity
                self.velocity_previous = self.velocity

                # Get states
                self.time_get_states = time.time()
                self.get_states()
                self.time_get_states = time.time() - self.time_get_states

                # Get position from GPS
                if gps_use:
                    if ((time.time() - self.gaining_speed_start) - self.gps_timestamp) > 1.0 / gps_dataUpdateRate or self.gps_timestamp <= 0.01:
                        # self.bike.get_gps_data()
                        self.time_gps = time.time()
                        self.gps_read()

                        if self.gps_nmea_timestamp_ini == 0.0:
                            self.gps_nmea_timestamp_ini = self.gps_nmea_timestamp

                        # Check if GPS NMEA timestamp is more than 1s away from BeagleBone timestamp

                        if abs((datetime.strptime(self.gps_nmea_timestamp, '%H%M%S.%f') - datetime.strptime(self.gps_nmea_timestamp_ini, '%H%M%S.%f')).total_seconds() - self.time_count) > 1:
                            print("WARNING: the GPS NMEA timestamp is more than 1s away from BeagleBone timestamp. Check GPS data, it might be compromised.")

                    # Estimate states (v, yaw, heading, x, y)
                    self.estimate_states()
                else:
                    self.x_measured_GPS = 0.0
                    self.y_measured_GPS = 0.0
                    self.lat_measured_GPS = 0.0
                    self.lon_measured_GPS = 0.0
                    self.gps_timestamp = 0.0

                self.sensor_reading_time = time.time() - self.time_start_current_loop

                # Let bike get up to speed for a few seconds before starting controllers
                if (self.time_count < walk_time):
                    if not self.walk_message_printed_flag:
                        self.walk_message_printed_flag = True
                        print("Please walk the bike as straight as possible and let go of the handle bar as soon as it gets stiff")

                    # Estimate steering angle offset from mean of steering angle during walk
                    self.steering_angle_offset = self.steering_angle_offset + self.steeringAngle
                    self.steering_angle_offset_count = self.steering_angle_offset_count + 1
                elif (self.time_count < speed_up_time+walk_time and self.time_count >= walk_time) and not self.gainingSpeedOver_flag:
                    # Compute mean of steering angle during walk
                    if not self.steering_angle_offset_computed_flag:
                        self.steering_angle_offset_computed_flag = True
                        self.steering_angle_offset = self.steering_angle_offset / self.steering_angle_offset_count
                        print('Steering angle offset : %.2f deg' % (self.steering_angle_offset*rad2deg))

                    if not self.speed_up_message_printed_flag:
                        self.speed_up_message_printed_flag = True
                        print('Gaining speed ...')
                elif (self.time_count >= speed_up_time+walk_time) and not self.gainingSpeedOver_flag:
                    # Once enough time has passed, start controller
                    self.gainingSpeedOver_flag = True
                    print('Gaining speed phase over')

                    # Check that speed if high enough
                    if self.velocity < 0.5*initial_speed:
                        print("WARNING : speed is lower than half the reference speed. Hall sensors or drive motor might be faulty. Will use reference speed instead of measured speed in calculations.")
                        self.broken_speed_flag = True

                    self.controller_active = True

                    # Time at which the controller starts running
                    self.time_start_controller = time.time()

                    # Abort experiment if bike is in unsafe conditions at this point
                    # if (abs(self.roll)>20*deg2rad or abs(self.rollRate)>20*deg2rad or abs(self.steeringAngle)>20*deg2rad):
                    if (abs(self.roll) > 20*deg2rad or abs(self.steeringAngle) > 20*deg2rad):
                        exc_msg = 'Bike is in unsafe conditions, aborting the experiment'
                        print(exc_msg)
                        self.exception_log(-1, exc_msg)
                        break

                    # Reset estimated roll to zero
                    self.roll = 0
                    self.bike.imu.phi = 0
            except Exception as e:
                print("Number of times sampling time was exceeded : %d" %(self.exceedscount))

                # e = sys.exc_info()[0]
                print("Detected error :")
                print(e)
                print(traceback.print_exc())

                self.stop()
                exc_msg = 'Error or keyboard interrupt, aborting the experiment'
                print(exc_msg)
                self.exception_log(-2, exc_msg)
                break


            # Control Frequency
            self.loop_time = time.time() - self.time_start_current_loop
            self.time_count = time.time() - self.gaining_speed_start
            

            # Check for ESTOP
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                exc_msg = 'Emergency stop pressed, aborting the experiment'
                print(exc_msg)
                self.exception_log(0,exc_msg)
                break

            # Check for extreme PHI
            if self.roll > MAX_LEAN_ANGLE or self.roll < MIN_LEAN_ANGLE:
                self.stop()
                exc_msg = 'Exceeded min/max lean (roll) angle abs %3f > abs %3f, aborting the experiment' %(self.roll, MAX_LEAN_ANGLE)
                print(exc_msg)
                self.exception_log(2, exc_msg)
                break


            # End test time condition
            if self.time_count > test_duration:
                # Print number of times sampling time was exceeded if experiment is aborted early
                self.stop()
                exc_msg = 'Exceeded test duration, aborting the experiment'
                print(exc_msg)
                self.exception_log(-1, exc_msg)
                break

            # Log data
            self.log_regular()

            # Compute total time for current loop
            self.loop_time = time.time() - self.time_start_current_loop
            # Sleep to match sampling time
            if self.loop_time < sample_time:
                time.sleep((sample_time - self.loop_time))

            # Distance travelled
            self.distance_travelled += self.velocity * (time.time() - self.time_start_current_loop)

        # Print number of times sampling time was exceeded after experiment is over
        print("Number of times sampling time was exceeded : %d" % (self.exceedscount))


    ####################################################################################################################
    ####################################################################################################################
    # Stop bike
    def stop(self):
        self.controller_active = False
        self.bike.stop_all()


    ####################################################################################################################
    ####################################################################################################################
    # Initialize bike variables
    def variable_init(self):
        # Gaining Speed Phase
        self.walk_message_printed_flag = False
        self.speed_up_message_printed_flag = False
        self.gaining_speed_start = 0.0
        self.gainingSpeedOver_flag = False
        self.broken_speed_flag = False

        self.roll_ref_imp_doneflag1 = False
        self.roll_ref_imp_doneflag2 = False

        # GPS
        self.time_gps = 0.0
        self.gps_nmea_timestamp_ini = 0.0
        self.gps_nmea_timestamp = 0.0
        self.gpspos = 0.0
        self.x_measured_GPS = 0.0
        self.y_measured_GPS = 0.0
        self.pos_GPS = np.matrix([[self.x_measured_GPS],[self.y_measured_GPS]])
        self.lat_measured_GPS = 0.0
        self.lon_measured_GPS = 0.0
        self.lat_measured_GPS_ini = 'none'
        self.gps_status = 'No status'
        self.gps_timestamp = 0.0
        self.psi_measured_GPS = 0.0
        self.x_measured_GPS_previous = 0.0
        self.y_measured_GPS_previous = 0.0
        self.pos_GPS_previous = np.matrix([[self.x_measured_GPS_previous],[self.y_measured_GPS_previous]])

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
        self.time_pathtracking = 0.0
        self.roll_ref_end_time = roll_ref_end_time
        self.roll_ref_start_time = roll_ref_start_time
        if circle_switch is True:
            self.roll_ref_start_time1 = roll_ref_start_time1
            self.roll_ref_start_time2 = roll_ref_start_time2


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

        # States Estimators
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
        self.steering_angle_offset_computed_flag = False

        # Controller
        self.controller_active = False

        # Path Trakcing Controller
        self.distanceTravelled = 0.0


    ####################################################################################################################
    ####################################################################################################################
    # Get bike states
    def get_states(self):
        self.steeringAngle = self.bike.get_handlebar_angle()
        if self.steeringAngle > MAX_HANDLEBAR_ANGLE or self.steeringAngle < MIN_HANDLEBAR_ANGLE:
            print('WARNING : [%f] Steering angle exceeded limits' % (
                    time.time() - self.gaining_speed_start))

        if self.steering_angle_offset_computed_flag:
            self.steeringAngle = self.steeringAngle - self.steering_angle_offset

        # imu_data = [phi_comp, phi_gyro, gx (phidot), gy, gz, ax, ay, az]
        # self.imu_data = self.bike.get_imu_data(0, self.steeringAngle, self.roll)
        self.imu_data = self.bike.get_imu_data(self.velocity, self.steeringAngle, self.roll)

        # Extract states
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
            print('WARNING : [%f] Measured roll rate larger than 20deg/s, at %g deg/s' % (time.time() - self.gaining_speed_start, self.rollRate * rad2deg))
            self.rollRate = self.rollRate_prev


    ####################################################################################################################
    ####################################################################################################################
    # Get position from GPS
    # @pysnoope
    def estimate_states(self):
        dt = time.time() - self.time_estimate_previous

        self.beta = np.arctan((LENGTH_A / LENGTH_B) * np.tan(self.steeringAngle))

        # Velocity estimators
        # self.v_estimated_onlyMeasurements = statesEstimators_KvH * self.velocity + statesEstimators_KvGPS * np.linalg.norm(self.pos_GPS - self.pos_GPS_previous) / dt + statesEstimators_KvRef * initial_speed
        self.v_estimated_onlyMeasurements = statesEstimators_KvH * self.velocity_rec + statesEstimators_KvGPS * np.linalg.norm(self.pos_GPS - self.pos_GPS_previous) / dt + statesEstimators_KvRef * initial_speed
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
        # self.yaw_estimated = (1 - statesEstimators_Kpsi) * (self.yaw_estimated_previous + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous)) \
        #                      + statesEstimators_Kpsi * ((np.arctan2(self.y_estimated - self.y_estimated_previous, self.x_estimated - self.x_estimated_previous) - self.beta_previous) + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous))
        self.yaw_estimated = (1 - statesEstimators_Kpsi) * (self.yaw_estimated_previous + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous)) \
                             + statesEstimators_Kpsi * ((np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.x_measured_GPS_previous) - self.beta_previous) + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous))
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
        self.time_estimate_previous = time.time()

        # Compute estimated lat/lon
        self.lon_estimated = (self.x_estimated * rad2deg) / R / np.cos(self.lat_measured_GPS_ini * deg2rad) + self.lon_measured_GPS_ini
        self.lat_estimated = (self.y_estimated * rad2deg) / R + self.lat_measured_GPS_ini


    ####################################################################################################################
    ####################################################################################################################
    # Get position from GPS
    # @pysnoope
    def gps_read(self):
        # Get GPS position
        self.gpspos = self.bike.gps.get_position()
        if ((self.gpspos[2] >= 53) and (self.gpspos[2] <= 70) and (self.gpspos[3] >= 8) and (self.gpspos[3] <= 26)):  # The location should be in SWEDEN
            self.x_measured_GPS = self.gpspos[0]
            self.y_measured_GPS = self.gpspos[1]
            self.psi_measured_GPS = np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.x_measured_GPS_previous)
            self.lat_measured_GPS = self.gpspos[2]
            self.lon_measured_GPS = self.gpspos[3]
            self.gps_timestamp = time.time() - self.gaining_speed_start

            if self.lat_measured_GPS_ini == 'none':
                self.lat_measured_GPS_ini = self.lat_measured_GPS
                self.lon_measured_GPS_ini = self.lon_measured_GPS

            # Read GPS status if it exists
            if len(self.gpspos) >= 5:
                self.gps_status = self.gpspos[4]

            # Read GPS NMEA timestamp if it exists
            if len(self.gpspos) >= 6:
                self.gps_nmea_timestamp = self.gpspos[5]

            # Save previous x and y positions to use in heading computation
            self.x_measured_GPS_previous = self.x_measured_GPS
            self.y_measured_GPS_previous = self.y_measured_GPS

            # Save x and y in a single vector
            self.pos_GPS = np.array([[self.x_measured_GPS],[self.y_measured_GPS]])
            self.pos_GPS_previous = np.array([[self.x_measured_GPS_previous],[self.y_measured_GPS_previous]])

            # Update Kalman filter
            # position_kalman = self.bike.kf.update(x_measured_GPS, y_measured_GPS, self.states[1], self.velocity)
            # print('x_measured_GPS = %f ; y_measured_GPS = %f ; x_kalman = %f ; y_kalman = %f' % (self.x_measured_GPS, self.y_measured_GPS, position_kalman[0], position_kalman[1]))


    ####################################################################################################################
    ####################################################################################################################
    # Initial Estop check
    def initial_Estop_Check(self):
        self.ESTOP = self.bike.emergency_stop_check()
        if self.ESTOP:
            print('Emergency stop pressed, the experiment will be aborted if it is not released now')
            input_estop = raw_input('Press ENTER to continue')
            if input_estop:
                self.stop()
                exc_msg = 'Emergency stop was not released, aborting the experiment before it starts'
                print(exc_msg)
                self.exception_log(0,exc_msg)


    ####################################################################################################################
    ####################################################################################################################
    # Log data
    def log_headerline(self):
        # Data logging setup
        timestr = time.strftime("%Y%m%d-%H%M%S")
        csv_path = './paths/%s-%s.csv' % (self.file_name, timestr)
        results_csv = open(csv_path, 'wb')

        self.writer = csv.writer(results_csv)

        self.log_header_str = ['Time', 'lat_estimated', 'lon_estimated', 'latitude', 'longitude', 'RealTime', 'CalculationTime', 'MeasuredVelocity','Roll', 'SteeringAngle',
                               'RollRate','gy', 'gz', 'ax', 'ay', 'az', 'imu_read_timing']

        if gps_use:
            self.log_header_str += ['v_estimated', 'yaw_estimated', 'heading_estimated', 'x_estimated',
                                    'y_estimated', 'GPS_timestamp', 'x_GPS', 'y_GPS', 'status', 'NMEA timestamp']

        self.writer.writerow(self.log_header_str)

    def log_regular(self):
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
            "{0:.5f}".format(self.roll),
            "{0:.5f}".format(self.steeringAngle),
            "{0:.5f}".format(self.rollRate_rec),
            "{0:.5f}".format(self.gy),
            "{0:.5f}".format(self.gz),
            "{0:.5f}".format(self.ax),
            "{0:.5f}".format(self.ay),
            "{0:.5f}".format(self.az),
            "{0:.5f}".format(self.sensor_read_timing)
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
                self.gps_status,
                self.gps_nmea_timestamp
            ]

        self.writer.writerow(self.log_str)
        self.time_log = time.time() - self.time_log

        if debug or (self.loop_time > sample_time):
            if self.loop_time > sample_time:
                print("WARNING: The calculation time exceeds the sampling time!")
                self.exceedscount += 1

            # Print sensor reading time, control calculation time, IMU data reading time and logging time
            print("sensor_reading_time   control calculation   IMU   log  = %g \t %g \t %g \t %g \t" % (
                self.sensor_reading_time, self.control_cal_time, self.time_get_states,
                self.time_log))

        self.rollRate_prev = self.rollRate


    def reset_global_angles_and_coordinates(self):
        self.x_estimated = initial_speed * (time.time() - self.time_start_controller)
        self.y_estimated = 0.0
        self.psi_estimated = 0.0
        self.nu_estimated = 0.0
        print "Odometry reset for heading control"

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
            self.writer.writerow(self.log_str)
