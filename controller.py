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
class Controller(object):
    # @pysnooper.snoop()
    def __init__(self, bike):
        self.bike = bike
        self.variable_init()

        # Check Estop before starting the experiment
        self.initial_Estop_Check()  # Check if the Estop Engaged

        # Load path
        if path_tracking and path_file != 'pot':
            try:
                if path_file == 'newest':
                    list_of_files = glob.glob('./paths/*.csv')
                    latest_file = max(list_of_files, key=os.path.getctime)
                    print("Loading newest path %s ..." % (latest_file))
                    self.path_data = np.genfromtxt(latest_file, delimiter=",", skip_header=1)
                else:
                    print("Loading path %s ..." % (path_file))
                    # self.path_data = np.genfromtxt('paths/' + path_file, delimiter=",", skip_header=1)
                    self.path_data = np.genfromtxt('paths/' + path_file, delimiter=",")
                self.path_time = self.path_data[:,0]
                if self.path_data[0,1] == 0:
                    # Case if the path in defined in (x,y,psi)
                    self.path_x = self.path_data[:,1]
                    self.path_y = self.path_data[:,2]
                    self.path_heading = self.path_data[:,3]
                else:
                    # Case if the path in defined in (lat,lon)
                    self.path_lat = self.path_data[:,1]
                    self.path_lon = self.path_data[:,2]
                    self.path_x = R * self.path_lon * deg2rad * np.cos(self.path_lat[0] * deg2rad)
                    self.path_y = R * self.path_lat * deg2rad
                    self.path_x = self.path_x - self.path_x[0]
                    self.path_y = self.path_y - self.path_y[0]
                    self.path_heading = np.arctan2(self.path_y[1:] - self.path_y[0:-1], (self.path_x[1:] - self.path_x[0:-1]))
                    self.path_heading = np.append(self.path_heading, self.path_heading[-1])
                print("Path loaded, loading roll reference if needed, otherwise starting experiment ...")
            except:
                print("Path file not found, setting all path references to 0 as default")
                # self.path_data = np.array([[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]) # Using two rows with zeros for np.interp to work
                self.path_data = np.array([[0.0,0.0,0.0,0.0],[1000000.0,0.0,0.0,0.0]]) # Using two rows with zeros for np.interp to work
                self.path_time = self.path_data[:,0]
                self.path_x = self.path_data[:,1]
                self.path_y = self.path_data[:,2]
                self.path_heading = self.path_data[:,3]
            self.path_distanceTravelled = np.cumsum(np.sqrt((self.path_x[1:] - self.path_x[0:-1])**2 + (self.path_y[1:] - self.path_y[0:-1])**2))
            self.path_distanceTravelled = np.append(0,self.path_distanceTravelled)

        # Load roll reference
        if rollref_file != 'nofile':
            print("Loading roll reference %s ..." % (rollref_file))
            try:
                self.rollref_data = np.genfromtxt('rollref/' + rollref_file, delimiter=",", skip_header=1)
                self.rollref_time = self.rollref_data[:, 0]
                self.rollref_roll = self.rollref_data[:, 1]
                print("Roll reference loaded, starting experiment.")
            except:
                print([rollref_file, "Path file not found, setting roll reference to 0 as default"])
                # self.rollref_data = np.array([[0.0, 0.0], [0.0, 0.0]]) # Using two rows with zeros for np.interp to work
                self.rollref_data = np.array([[0.0, 0.0], [1000000.0, 0.0],]) # Using three rows with zeros for np.interp to work
                self.rollref_time = self.rollref_data[:, 0]
                self.rollref_roll = self.rollref_data[:, 1]

        if strdistbref_file != 'nofile':
            print("Loading steering rate disturbance reference %s ..." % (strdistbref_file))
            try:
                self.strdistbref_data = np.genfromtxt('strratedistbref/' + strdistbref_file, delimiter=",", skip_header=1)
                self.strdistbref_time = self.strdistbref_data[:, 0]
                self.strdistbref_str = self.strdistbref_data[:, 1]
                print("Steering rate disturbance reference loaded, starting experiment.")
            except:
                print([strdistbref_file, "Steering rate disturbance file not found, setting roll reference to 0 as default"])
                # self.strdistbref_data = np.array([[0.0, 0.0], [0.0, 0.0]]) # Using two rows with zeros for np.interp to work
                self.strdistbref_data = np.array([[0.0, 0.0], [1000000.0, 0.0],]) # Using three rows with zeros for np.interp to work
                self.strdistbref_time = self.strdistbref_data[:, 0]
                self.strdistbref_str = self.strdistbref_data[:, 1]

        # Read the IMU complementary filter Phi as the initial phi estimation
        # self.roll = self.bike.get_imu_data(0, self.steeringAngle, self.roll)[0]
        # self.roll = self.bike.get_imu_data(self.velocity, self.steeringAngle, self.roll)[0]

        # Get experiment (if any) of the experiment
        self.descr = raw_input('Type a description for the experiment if necessary. Press ENTER to start the experiment. ')

        # Create log file and add header line
        self.log_headerline()

        # Wait before starting experiment
        print("")
        for i in range(0,int(math.ceil(start_up_interval))):
            time.sleep(1)
            print("Experiment starting in %is" % (int(math.ceil(start_up_interval))-i))
        print("")
        # print("\nExperiment starting in %is\n" % start_up_interval)
        # time.sleep(start_up_interval)

    ####################################################################################################################
    ####################################################################################################################
    # Run bike
    # @pysnooper.snoop()
    def run(self):
        # Clear PID controller memory
        self.pid_velocity.clear()
        self.pid_balance.clear()
        self.pid_balance_outerloop.clear()
        self.pid_steeringangle.clear()
        self.pid_lateral_position.clear()
        self.pid_direction.clear()

        # self.bike.steering_motor.enable()

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

                # Estimate states (v, yaw, heading, x, y)
                # self.x_estimated, self.y_estimated, self.psi_estimated, self.nu_estimated = global_angles_and_coordinates(self.velocity, sample_time,
                #                                                                                                           LENGTH_A, LENGTH_B,
                #                                                                                                           self.steeringAngle,self.psi_estimated,
                #                                                                                                           self.x_estimated, self.y_estimated)
                # self.estimate_states()

                # Estimate states (v, yaw, heading, x, y)
                if self.compute_estimators_flag:
                    self.estimate_states()
                    self.compute_estimators_flag = False

                # Get position from GPS
                if gps_use:
                    if ((time.time() - self.gaining_speed_start) - self.gps_timestamp) > 1.0 / gps_dataUpdateRate or self.gps_timestamp <= 0.01:
                        # self.bike.get_gps_data()
                        self.time_gps = time.time()
                        self.gps_read()

                        if self.gps_nmea_timestamp_ini == 0.0:
                            self.gps_nmea_timestamp_ini = self.gps_nmea_timestamp

                        # Set flag to compute estimated states at next time step
                        self.compute_estimators_flag = True

                        # Check if GPS NMEA timestamp is more than 1s away from BeagleBone timestamp
                        if abs((datetime.strptime(self.gps_nmea_timestamp, '%H%M%S.%f') - datetime.strptime(self.gps_nmea_timestamp_ini, '%H%M%S.%f')).total_seconds() - self.time_count) > 1:
                            print("WARNING: the GPS NMEA timestamp is more than 1s away from BeagleBone timestamp. Check GPS data, it might be compromised.")
                else:
                    self.x_measured_GPS = 0.0
                    self.y_measured_GPS = 0.0
                    self.lat_measured_GPS = 0.0
                    self.lon_measured_GPS = 0.0
                    self.gps_timestamp = 0.0

                # Get laser ranger position (y position on the roller)
                if laserRanger_use:
                    #if (time.time() - self.time_laserranger) > 1.1 * self.bike.laser_ranger.timing:
                    if (time.time() - self.time_laserranger) > 10 * sample_time:
                        self.time_laserranger = time.time()
                        self.y_laser_ranger = self.bike.get_laserRanger_data()
                        if abs(self.y_laser_ranger)>0.25:
                            print('WARNING : [%f] Laser ranger position outside of roller' % (time.time() - self.gaining_speed_start))
                else:
                    self.y_laser_ranger = 0
                # Compute time needed to read from all sensors
                self.sensor_reading_time = time.time() - self.time_start_current_loop

                # if laserRanger_use:
                #     print("Laser ranger reading time : %f" %(time.time()-self.time_laserranger))
                # if gps_use:
                #     print("GPS reading time : %f" %(time.time()-self.time_gps))

                # PID Velocity Control
                if pid_velocity_active:
                    self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                    self.bike.set_velocity(self.pid_velocity_control_signal)

                # Let bike get up to speed for a few seconds before starting controllers
                if (self.time_count < walk_time):
                    if not self.walk_message_printed_flag:
                        self.walk_message_printed_flag = True
                        print("Please walk the bike as straight as possible and let go of the handle bar as soon as it gets stiff")

                    self.bike.steering_motor.disable()

                    # Estimate steering angle offset from mean of steering angle during walk
                    self.steering_angle_offset = self.steering_angle_offset + self.steeringAngle
                    self.steering_angle_offset_count = self.steering_angle_offset_count + 1
                elif (self.time_count < speed_up_time+walk_time and self.time_count >= walk_time) and not self.gainingSpeedOver_flag:
                    # Compute mean of steering angle during walk
                    if not self.steering_angle_offset_computed_flag:
                        self.steering_angle_offset_computed_flag = True
                        self.steering_angle_offset = self.steering_angle_offset / self.steering_angle_offset_count
                        print('Steering angle offset : %.2f deg' % (self.steering_angle_offset*rad2deg))

                    # Do not start controllers until bike ran for enough time to get up to speed
                    # self.bike.steering_motor.enable()
                    self.bike.steering_motor.disable()

                    if not self.speed_up_message_printed_flag:
                        self.speed_up_message_printed_flag = True
                        print('Gaining speed ...')

                    self.bike.set_velocity(initial_speed)
                elif (self.time_count >= speed_up_time+walk_time) and not self.gainingSpeedOver_flag:
                    # Once enough time has passed, start controller
                    self.gainingSpeedOver_flag = True
                    print('Gaining speed phase over')

                    # Check that speed if high enough
                    if self.velocity < 0.5*initial_speed:
                        print("WARNING : speed is lower than half the reference speed. Hall sensors or drive motor might be faulty. Will use reference speed instead of measured speed in calculations.")
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
                    self.bike.steering_motor.enable()

                    # Restart PWM before using steering motor because it gets deactivated at the some point before in the code
                    # TO DO : CHECK WHY THIS HAPPENS !
                    PWM.start(steeringMotor_Channel, steeringMotor_IdleDuty, steeringMotor_Frequency)


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
                elif self.controller_active:
                    # Check steering angle
                    self.keep_handlebar_angle_within_safety_margins(self.steeringAngle)

                    # Balancing and path tracking control
                    # self.bike.steering_motor.enable()
                    # self.controller_set_handlebar_angular_velocity(0)
                    # self.update_controller_gains()
                    self.keep_the_bike_stable()


                # Compute time needed to run controllers
                self.control_cal_time = time.time() - self.time_start_current_loop - self.sensor_reading_time
            #except (ValueError, KeyboardInterrupt):
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
        # Log File Description
        self.descr = 'NoComments'

        # Gaining Speed Phase
        self.walk_message_printed_flag = False
        self.speed_up_message_printed_flag = False
        self.gaining_speed_start = 0.0
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
        self.gps_nmea_timestamp_ini = 0.0
        self.gps_nmea_timestamp = 0.0
        self.gpspos = 0.0
        self.x_measured_GPS = 0.0
        self.y_measured_GPS = 0.0
        self.pos_GPS = np.matrix([[self.x_measured_GPS],[self.y_measured_GPS]])
        self.lat_measured_GPS = 0.0
        self.lon_measured_GPS = 0.0
        self.gps_status = 'No status'
        self.gps_timestamp = 0.0
        self.psi_measured_GPS = 0.0
        self.x_measured_GPS_previous = 0.0
        self.y_measured_GPS_previous = 0.0
        self.pos_GPS_previous = np.matrix([[self.x_measured_GPS_previous],[self.y_measured_GPS_previous]])
        # self.x_estimated_Odo = 0.0
        # self.y_estimated_Odo = 0.0
        # self.psi_estimated_Odo = 0.0

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
        self.steeringCurrent = 0.0

        # States Estimators
        self.compute_estimators_flag = False
        self.beta = 0.0
        self.v_estimated = 0.0
        self.pos_estimated = 0.0
        self.x_estimated = 0.0
        self.y_estimated = 0.0
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
        self.steeringAngle = self.bike.get_handlebar_angle()
        if self.steeringAngle > MAX_HANDLEBAR_ANGLE or self.steeringAngle < MIN_HANDLEBAR_ANGLE:
            print('WARNING : [%f] Steering angle exceeded limits' % (
                    time.time() - self.gaining_speed_start))

        if self.steering_angle_offset_computed_flag:
            self.steeringAngle = self.steeringAngle - self.steering_angle_offset

        self.steeringCurrent = self.bike.steering_motor.read_steer_current()

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
                             + statesEstimators_Kpsi * ((np.arctan2(self.y_measured_GPS - self.y_measured_GPS_previous,self.x_measured_GPS - self.y_measured_GPS_previous) - self.beta_previous) + (self.v_estimated_previous / LENGTH_A) * dt * np.sin(self.beta_previous))
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
            self.bike.stop()
            exc_msg = 'Exceeded MAX_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg)
            print(exc_msg)
            self.exception_log(1, exc_msg)
        elif handlebar_angle < min_handlebar_angle:
            # print('Exceeded MIN_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg))
            self.lowerSaturation = True
            self.bike.stop()
            exc_msg = 'Exceeded MIN_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg)
            print(exc_msg)
            self.exception_log(1, exc_msg)
        else:
            self.upperSaturation = False
            self.lowerSaturation = False


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
    # Update controller gains for current forward speed
    def update_controller_gains(self):
        # self.velocity = 4 + math.sin(self.time_count)
        # print('vel = %f' % (self.velocity))

        if len(speed_lookup_controllergains):
            self.idx_speed_lookup = bisect.bisect_left(self.speed_lookup_controllergains_np, self.velocity) + np.array([-1, 0])
            print('idx_speed_lookup = ' + np.array2string(self.idx_speed_lookup))
            print(self.speed_lookup_controllergains_np[self.idx_speed_lookup])

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

        print('Kpbal = %f ; Kibal = %f ; Kdbal = %f' % (self.pid_balance.Kp,self.pid_balance.Ki,self.pid_balance.Kd))
        print('Kpbalouter = %f ; Kibalouter = %f ; Kdbalouter = %f' % (self.pid_balance_outerloop.Kp,self.pid_balance_outerloop.Ki,self.pid_balance_outerloop.Kd))

    ####################################################################################################################
    ####################################################################################################################
    # Get bike states references
    #@pysnooper.snoop()
    def get_balancing_setpoint(self):
        if path_tracking and not self.path_tracking_engaged:
            if time.time()-self.time_start_controller > balancing_time:
                self.path_tracking_engaged = True
                self.reset_global_angles_and_coordinates()
                print "Now heading or path tracking is engaged."
        if self.path_tracking_engaged:
            # Get reference position and heading
            if path_file == 'pot':
                self.x_ref = 0.0
                self.heading_ref = 0.0
                if potentiometer_use:
                    self.pot = ((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 0.2 - 0.1)  # Potentiometer gives a position reference between -0.1m and 0.1m
                    self.y_ref = self.pot
                else:
                    self.y_ref = 0.0
            else:
                if self.distance_travelled >= self.path_distanceTravelled[-1]:
                    self.x_ref = self.path_x[-1]
                    self.y_ref = self.path_y[-1]
                    self.heading_ref = self.path_heading[-1]
                else:
                    idx_path_currentDistanceTravelled = bisect.bisect_left(self.path_distanceTravelled,self.distance_travelled)+np.array([-1,0])
                    self.x_ref = np.interp(self.distance_travelled,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_x[idx_path_currentDistanceTravelled])
                    self.y_ref = np.interp(self.distance_travelled,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_y[idx_path_currentDistanceTravelled])
                    self.heading_ref = np.interp(self.distance_travelled,self.path_distanceTravelled[idx_path_currentDistanceTravelled],self.path_heading[idx_path_currentDistanceTravelled])
            # else:
            #     if (time.time() - self.time_start_controller) > self.path_time[-1]:
            #         self.x_ref = self.path_x[-1]
            #         self.y_ref = self.path_y[-1]
            #         self.heading_ref = self.path_heading[-1]
            #     else:
            #         idx_path_currentTime = bisect.bisect_left(self.path_time,time.time() - self.time_start_controller)+np.array([-1,0])
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
            #     self.heading_error = self.heading_ref - self.psi_measured_GPS
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
            # print('heading_ref = %f ; psi_GPS = %f ; heading_error = %f' % (self.heading_ref,self.psi_measured_GPS,self.heading_error))
            # print('')
                
            # Compute lateral and heading errors
            self.lateral_error = self.x_error * np.sin(self.heading_ref) + self.y_error * np.cos(self.heading_ref)
            self.heading_error = self.heading_error

            if (time.time() - self.time_pathtracking) > 10 * sample_time:
                self.time_pathtracking = time.time()

                if path_tracking_structure == 'parallel':
                    # PID Lateral Position Controller
                    self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not measurement
                    # PID Direction/Heading Controller
                    self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not measurement
                    # Compute balancing setpoint
                    self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal
                elif path_tracking_structure == 'series':
                    # PID Lateral Position Controller
                    self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not measurement
                    if heading_controller:
                        # PID Direction/Heading Controller
                        self.pid_direction.setReference(lateralError_controller * self.pid_lateral_position_control_signal)
                        self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not measurement
                        # Compute balancing setpoint
                        self.balancing_setpoint = self.pid_direction_control_signal
                    else:
                        # Compute balancing setpoint
                        self.balancing_setpoint = self.pid_lateral_position_control_signal
                else:
                    print("Path tracking controller structure choice is not valid : \"%s\"; Using parallel structure as default.\n" % (balancing_controller_structure))
                    # PID Lateral Position Controller
                    self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not measurement
                    # PID Direction/Heading Controller
                    self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not measurement
                    # Compute balancing setpoint
                    self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal
        elif potentiometer_use:
            self.pot = -((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 2.5 - 1.25) * deg2rad * 2 # Potentiometer gives a position reference between -2.5deg and 2.5deg
            self.balancing_setpoint = self.pot
        else:
            if rollref_file == 'nofile':
                if roll_ref_use and not roll_ref_step_imp_flag: # Do step
                    if not rol_ref_periodic:
                        if self.time_count < self.roll_ref_end_time: #__(ref_start_time)------(ref_end_time)_____
                            if self.time_count > self.roll_ref_start_time:
                                self.balancing_setpoint = roll_ref_Mag
                            else:
                                self.balancing_setpoint = 0
                            # print self.time_count, roll_ref_start_time, roll_ref_end_time
                        else:
                            self.balancing_setpoint = 0
                    else:
                        if not circle_switch:
                            if self.time_count < self.roll_ref_end_time:
                                if self.time_count > self.roll_ref_start_time:
                                    self.balancing_setpoint = roll_ref_Mag
                                else:
                                    self.balancing_setpoint = 0
                                # print self.time_count, roll_ref_start_time, roll_ref_end_time
                            else:
                                self.balancing_setpoint = 0
                                self.roll_ref_start_time = self.roll_ref_start_time + roll_ref_period
                                self.roll_ref_end_time = self.roll_ref_end_time + roll_ref_period
                        else:
                            if self.time_count < self.roll_ref_end_time: # One loop Not finished yet
                                if self.time_count > self.roll_ref_start_time1 and self.time_count < self.roll_ref_start_time2:
                                    self.balancing_setpoint = roll_ref_Mag1
                                elif self.time_count >= self.roll_ref_start_time2:
                                    self.balancing_setpoint = roll_ref_Mag2
                                else:
                                    self.balancing_setpoint = 0
                                # print self.time_count, roll_ref_start_time, roll_ref_end_time
                            else:
                                self.balancing_setpoint = roll_ref_Mag1
                                self.roll_ref_start_time1 = self.roll_ref_start_time1 + roll_ref_totalperiod
                                self.roll_ref_start_time2 = self.roll_ref_start_time2 + roll_ref_totalperiod
                                self.roll_ref_end_time = self.roll_ref_end_time + roll_ref_totalperiod
                elif roll_ref_use and roll_ref_step_imp_flag:
                    if not self.roll_ref_imp_doneflag1 and self.time_count > roll_ref_imp_start_time1:
                        self.balancing_setpoint = roll_ref_imp_Mag
                        self.roll_ref_imp_doneflag1 = True
                    elif not self.roll_ref_imp_doneflag2 and self.time_count > roll_ref_imp_start_time2:
                        self.balancing_setpoint = roll_ref_imp_Mag
                        self.roll_ref_imp_doneflag2 = True
                    else:
                        self.balancing_setpoint = 0
                else:
                    self.balancing_setpoint = 0
            else:
                idx_rollref_currentTime = bisect.bisect_left(self.rollref_time, time.time() - self.time_start_controller)+np.array([-1, 0])
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
        self.get_balancing_setpoint()

        if balancing_controller_structure == 'chalmers':
            # Chalmers Controller structure : deltadot = PID(phidot)
            self.pid_balance.setReference(self.balancing_setpoint)
            self.pid_balance_control_signal = self.pid_balance.update(self.rollRate)
            self.steering_rate = self.pid_balance_control_signal
        elif balancing_controller_structure == 'technion':
            # Technion Controller Structure : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
            self.pid_balance_outerloop.setReference(self.balancing_setpoint)
            self.pid_balance.setReference(self.pid_balance_outerloop.update(self.roll))
            self.pid_balance_control_signal = self.pid_balance.update(self.rollRate)
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
            self.pid_balance_control_signal = self.pid_balance.update(self.rollRate)
            self.steering_rate = self.pid_balance_control_signal

        # Low-pass filter
        # Butterworth 1st order 1Hz cutoff
        self.steering_rate_filt = 0.0305 * self.steering_rate + 0.0305 * self.steering_rate_previous + 0.9391 * self.steering_rate_filt_previous
        self.steering_rate_filt_previous = self.steering_rate_filt

        # self.steering_rate = self.steering_rate_filt

        if strdistbref_file != 'nofile':
            # print('SteeringRate Overwritten')
            idx_strdistbref_currentTime = bisect.bisect_left(self.strdistbref_time, time.time() - self.time_start_controller) + np.array([-1, 0])
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
        self.controller_set_handlebar_angular_velocity(self.steering_rate)
        # if self.pid_balance_control_signal > deadband or self.pid_balance_control_signal < -deadband:
        #     self.controller_set_handlebar_angular_velocity(self.steering_rate)
        # else:
        #     self.controller_set_handlebar_angular_velocity(0)


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
        csv_path = './ExpData_%s/BikeData_%s.csv' % (bike, timestr)
        print "EXP LOG PATH is: "
        print csv_path
        results_csv = open(csv_path, 'wb')
        # results_csv = open('./ExpData_%s/BikeData_%s.csv' % (bike, timestr), 'wb')
        self.writer = csv.writer(results_csv)

        if path_tracking:
            self.writer.writerow(['Description : ' + str(self.descr) + ' ; walk_time = ' + str(walk_time) + ' ; speed_up_time = ' + str(speed_up_time) + ' ; balancing_time = ' + str(balancing_time)])
        else:
            self.writer.writerow(['Description : ' + str(self.descr) + ' ; walk_time = ' + str(walk_time) + ' ; speed_up_time = ' + str(speed_up_time) + ' ; balancing_time = ' + str(balancing_time)])

        self.log_header_str = ['RealTime','Time', 'CalculationTime', 'MeasuredVelocity', 'BalancingGainsInner', 'BalancingGainsOuter', 'Roll', 'SteeringAngle', 'RollRate',
                               'ControlInput', 'BalancingSetpoint', 'gy', 'gz', 'ax', 'ay', 'az', 'imu_read_timing', 'SteerMotorCurrent']

        if potentiometer_use:
            self.log_header_str += ['Potentiometer']
        if gps_use:
            self.log_header_str += ['v_estimated', 'yaw_estimated', 'heading_estimated', 'x_estimated', 'y_estimated', 'GPS_timestamp', 'x_GPS', 'y_GPS', 'latitude', 'longitude','status','NMEA timestamp']
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
            "[{0:.5f},{1:.5f},{2:.5f}]".format(self.pid_balance.Kp,self.pid_balance.Ki,self.pid_balance.Kd),
            "[{0:.5f},{1:.5f},{2:.5f}]".format(self.pid_balance_outerloop.Kp,self.pid_balance_outerloop.Ki,self.pid_balance_outerloop.Kd),
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

        if potentiometer_use:
            self.log_str += ["{0:.5f}".format(self.pot)]
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
                "{0:.5f}".format(self.lat_measured_GPS),
                "{0:.5f}".format(self.lon_measured_GPS),
                self.gps_status,
                self.gps_nmea_timestamp
            ]
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
                print("WARNING: The calculation time exceeds the sampling time!")
                self.exceedscount += 1

            # Print sensor reading time, control calculation time, IMU data reading time and logging time
            print("sensor_reading_time   control calculation   IMU   log  = %g \t %g \t %g \t %g \t" % (
                self.sensor_reading_time, self.control_cal_time, self.time_get_states,
                self.time_log))

            # Stop experiment if exceeded sampling time too many times
            # if self.exceedscount > max_exceed_count:
            #     print(")
            #     self.stop()
            # exc_msg = 'Calculation time exceeded sampling time too often (%d times) , aborting the experiment' % (max_exceed_count)
            # print(exc_msg)
            # self.exception_log(3, exc_msg)
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
            # self.writer.writerow(self.log_str)
