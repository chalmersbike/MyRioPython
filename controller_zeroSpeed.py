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
import dubins

# @pysnooper.snoop()
class Controller(object):
    # @pysnooper.snoop()
    def __init__(self, bike):
        self.bike = bike

        self.variable_init()

        self.initial_Estop_Check()  # Check if the Estop Engaged

        # Get experiment (if any) of the experiment
        self.descr = raw_input('\nType a description for the experiment if necessary. Press ENTER to start the experiment. ')

        # Create log file and add header line
        self.log_headerline()

        # Wait before starting experiment
        print("")
        # for i in range(0,int(math.ceil(start_up_interval))):
        for i in range(0, int(math.ceil(5))):
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

        self.run_start = time.time()

        self.bike.steering_motor.enable()
        # Restart PWM before using steering motor because it gets deactivated at the some point before in the code
        # TO DO : CHECK WHY THIS HAPPENS !
        PWM.start(steeringMotor_Channel, steeringMotor_IdleDuty, steeringMotor_Frequency)

        while self.controller_active or not self.gainingSpeedOver_flag:
            self.time_start_current_loop = time.time()

            try:
                # Get states
                self.time_get_states = time.time()
                self.get_states()
                self.time_get_states = time.time() - self.time_get_states

                # Compute time needed to read from all sensors
                self.sensor_reading_time = time.time() - self.time_start_current_loop

                self.controller_active = True

                # Time at which the controller starts running
                self.time_start_controller = time.time()

                # Abort experiment if bike is in unsafe conditions at this point
                # if (abs(self.roll)>20*deg2rad or abs(self.rollRate)>20*deg2rad or abs(self.steeringAngle)>20*deg2rad):
                if (abs(self.roll) > 30*deg2rad or abs(self.steeringAngle) > 90*deg2rad):
                    exc_msg = ('[%f] Bike is in unsafe conditions, aborting the experiment' % (time.time() - self.run_start))
                    self.safe_stop()
                    print(exc_msg)
                    self.exception_log(-1, exc_msg)
                    break

                # Check steering angle
                self.keep_handlebar_angle_within_safety_margins(self.steeringAngle)

                # Balancing and path tracking control
                self.keep_the_bike_stable()

                # Compute time needed to run controllers
                self.control_cal_time = time.time() - self.time_start_current_loop - self.sensor_reading_time
            except Exception as e:
                print("[%f] Number of times sampling time was exceeded : %d" % (self.exceedscount,time.time() - self.run_start))

                # e = sys.exc_info()[0]
                print("[%f] Detected error :" % (time.time() - self.run_start))
                print(e)
                print(traceback.print_exc())

                self.safe_stop()
                exc_msg = ('[%f] Error or keyboard interrupt, aborting the experiment' % (time.time() - self.run_start))
                print(exc_msg)
                self.exception_log(-2, exc_msg)


            # Control Frequency
            self.loop_time = time.time() - self.time_start_current_loop
            self.time_count = time.time() - self.run_start
            


            # Check for ESTOP
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                exc_msg = '[%f] Emergency stop pressed, aborting the experiment' % (time.time() - self.run_start)
                print(exc_msg)
                self.exception_log(0,exc_msg)
                self.safe_stop()
                break

            # Check for extreme PHI
            if self.roll > MAX_LEAN_ANGLE or self.roll < MIN_LEAN_ANGLE:
                self.safe_stop()
                exc_msg = ('[%f] Exceeded min/max lean (roll) angle abs %3f > abs %3f, aborting the experiment' % (time.time() - self.run_start,self.roll, MAX_LEAN_ANGLE))
                print(exc_msg)
                self.exception_log(2, exc_msg)


            # End test time condition
            if self.time_count > test_duration:
                # Print number of times sampling time was exceeded if experiment is aborted early
                self.safe_stop()
                exc_msg = ('[%f] Exceeded test duration, aborting the experiment' % (time.time() - self.run_start))
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

            self.time_start_previous_loop = self.time_start_current_loop

        # Print number of times sampling time was exceeded after experiment is over
        print("[%f] Number of times sampling time was exceeded : %d" % (time.time() - self.run_start,self.exceedscount))

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
        self.run_start = 0.0
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
        self.lat_measured_GPS_ini = 'none'
        self.gps_status = 'No status'
        self.gps_timestamp = 0.0
        self.theta_measured_GPS = 0.0
        self.theta_measured_GPS_noUnwrap = 0.0
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
        self.time_deltadot_previous = 0.0

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
        self.velocity_rec = 0.0
        self.AngVel = 0.0
        self.steeringAngle = 0.0
        self.sensor_read_timing = 0.0
        self.steeringCurrent = 0.0
        self.deltadot = 0.0
        self.steering_acc  = 0.0
        self.steering_rate  = 0.0
        self.time_deltadot_previous = 0.0

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
        self.steering_angle_offset_computed_flag = False
        self.cumsum_v_dt = 0.0
        self.cumsum_delta_v_dt = 0.0
        self.y_GPS_rot_previous = 0.0
        self.y_GPS_rot = 0.0

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
        self.steeringAngle = self.bike.get_handlebar_angle()
        if self.steeringAngle > MAX_HANDLEBAR_ANGLE or self.steeringAngle < MIN_HANDLEBAR_ANGLE:
            print('[%f] WARNING : Steering angle exceeded limits' % (
                    time.time() - self.run_start))

        if self.steering_angle_offset_computed_flag:
            self.steeringAngle = self.steeringAngle + self.steering_angle_offset

        self.steeringCurrent = self.bike.steering_motor.read_steer_current()

        # imu_data = [phi_comp, phi_gyro, gx (phidot), gy, gz, ax, ay, az]
        self.imu_data = self.bike.get_imu_data(0, self.steeringAngle, self.roll)
        # self.imu_data = self.bike.get_imu_data(self.velocity, self.steeringAngle, self.roll)

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
            print('[%f] WARNING : Measured roll rate larger than 20deg/s, at %g deg/s' % (time.time() - self.run_start, self.rollRate * rad2deg))
            self.rollRate = self.rollRate_prev


    ####################################################################################################################
    ####################################################################################################################
    # Set angular velocity of handlebar / Set steering rate of steering motor
    def controller_set_handlebar_angular_velocity(self, angular_velocity):
        self.bike.set_handlebar_angular_velocity(angular_velocity)
        # if MAX_HANDLEBAR_ANGLE_ADJUST == 1:
        #     if self.upperSaturation:
        #         if angular_velocity > 0:  # we have exceeded the limit and are trying to move beyond it
        #             self.bike.set_handlebar_angular_velocity(-HANDLEBAR_CORRECTION_ANGVEL)
        #         else:  # we have exceeded the limit and are trying to move away from it
        #             self.bike.set_handlebar_angular_velocity(angular_velocity)
        #     elif self.lowerSaturation:
        #         if angular_velocity < 0:  # we have exceeded the limit and are trying to move beyond it
        #             self.bike.set_handlebar_angular_velocity(HANDLEBAR_CORRECTION_ANGVEL)
        #         else:  # we have exceeded the limit and are trying to move away from it
        #             self.bike.set_handlebar_angular_velocity(angular_velocity)
        #     else:
        #         self.bike.set_handlebar_angular_velocity(angular_velocity)
        # else:
        #     if self.upperSaturation:
        #         if angular_velocity > 0:  # we have exceeded the limit and are trying to move beyond it
        #             self.bike.set_handlebar_angular_velocity(0)
        #         else:  # we have exceeded the limit and are trying to move away from it
        #             self.bike.set_handlebar_angular_velocity(angular_velocity)
        #     elif self.lowerSaturation:
        #         if angular_velocity < 0:  # we have exceeded the limit and are trying to move beyond it
        #             self.bike.set_handlebar_angular_velocity(0)
        #         else:  # we have exceeded the limit and are trying to move away from it
        #             self.bike.set_handlebar_angular_velocity(angular_velocity)
        #     else:
        #         self.bike.set_handlebar_angular_velocity(angular_velocity)


    ####################################################################################################################
    ####################################################################################################################
    # Check that steering angle remains within acceptable bounds
    def keep_handlebar_angle_within_safety_margins(self, handlebar_angle):
        1
        # if MAX_HANDLEBAR_ANGLE_ADJUST == 1:
        #     if self.velocity < HIGHSPEEDBOUND:
        #         max_handlebar_angle = LowSpeedMAX_HANDLEBAR_ANGLE - self.velocity * (
        #                 LowSpeedMAX_HANDLEBAR_ANGLE - HighSpeedMAX_HANDLEBAR_ANGLE) / HIGHSPEEDBOUND
        #         min_handlebar_angle = - max_handlebar_angle
        #     else:
        #         max_handlebar_angle = HighSpeedMAX_HANDLEBAR_ANGLE
        #         min_handlebar_angle = - max_handlebar_angle
        # else:
        #     max_handlebar_angle = MAX_HANDLEBAR_ANGLE
        #     min_handlebar_angle = MIN_HANDLEBAR_ANGLE
        # if handlebar_angle > max_handlebar_angle:
        #     # print('Exceeded MAX_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg))
        #     self.upperSaturation = True
        #     # self.bike.stop()
        #     self.safe_stop()
        #     exc_msg = '[%f] Exceeded MAX_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (time.time() - self.run_start,max_handlebar_angle * rad2deg)
        #     print(exc_msg)
        #     self.exception_log(1, exc_msg)
        # elif handlebar_angle < min_handlebar_angle:
        #     # print('Exceeded MIN_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg))
        #     self.lowerSaturation = True
        #     # self.bike.stop()
        #     self.safe_stop()
        #     exc_msg = '[%f] Exceeded MIN_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (time.time() - self.run_start,max_handlebar_angle * rad2deg)
        #     print(exc_msg)
        #     self.exception_log(1, exc_msg)
        # else:
        #     self.upperSaturation = False
        #     self.lowerSaturation = False


    ####################################################################################################################
    ####################################################################################################################
    # Initial Estop check
    def initial_Estop_Check(self):
        self.ESTOP = self.bike.emergency_stop_check()

        if self.ESTOP:
            print('[%f] Emergency stop pressed, the experiment will be aborted if it is not released now' % (time.time() - self.run_start))
            input_estop = raw_input('Press ENTER to continue')
            if self.ESTOP:
                self.safe_stop()
                exc_msg = '[%f] Emergency stop was not released, aborting the experiment before it starts' % (time.time() - self.run_start)
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
    # Balancing controller
    def keep_the_bike_stable(self):
        # # lqr_gains = (9.69,-181.88,-42.37)
        # # lqr_gains = (9.69,-100,-42.37)
        # lqr_gains = (9.6,-177,-41)
        # self.steering_rate = -(lqr_gains[0]*self.steeringAngle + lqr_gains[1]*self.roll + lqr_gains[2]*self.rollRate)

        # lqr_gains = (77,15,-1340,-313)
        # self.deltadot = (self.steeringAngle - self.steeringAngle_previous)/((time.time() - self.run_start) - self.time_deltadot_previous)
        # self.steering_acc = -(lqr_gains[0]*self.steeringAngle + lqr_gains[1]*self.deltadot + lqr_gains[2]*self.roll + lqr_gains[3]*self.rollRate)
        # self.steering_rate += self.steering_acc * ((time.time() - self.run_start) - self.time_deltadot_previous)
        # self.time_deltadot_previous = time.time() - self.run_start
        # print("delta = %f ; deltadot = %f ; phi = %f ; phidot = %f ; deltaddot = %f ; controlInput = %f" % (self.steeringAngle,self.deltadot,self.roll,self.rollRate,self.steering_acc,self.steering_rate))

        # self.delta_ref = (0 if (time.time() - self.run_start < 1) else 20*deg2rad if (time.time() - self.run_start < 5) else -20*deg2rad)
        self.delta_ref = -45*deg2rad # Keep wheel in place
        # self.steering_rate = self.delta_ref - self.steeringAngle
        self.steering_rate = self.pos2vel(self.delta_ref)

        # # Low-pass filter
        # # Butterworth 1st order 1Hz cutoff
        # self.steering_rate_filt = 0.0305 * self.steering_rate + 0.0305 * self.steering_rate_previous + 0.9391 * self.steering_rate_filt_previous
        # self.steering_rate_filt_previous = self.steering_rate_filt

        self.steering_rate_previous = self.steering_rate

        # print(self.steering_rate)
        print("[%f] deltadot = %f ; delta = %f ; phi = %f ; phidot = %f" % (time.time() - self.run_start, self.steering_rate*rad2deg, self.steeringAngle*rad2deg, self.roll*rad2deg, self.rollRate*rad2deg))

        # Send Steering Rate Reference value to steering motor controller
        self.controller_set_handlebar_angular_velocity(self.steering_rate)


    ####################################################################################################################
    ####################################################################################################################
    # Exit the code safely and put bike on a stable circle with a 5deg constant roll reference
    def safe_stop(self):
        if (abs(self.roll) < 30 * deg2rad and abs(self.steeringAngle) < 90 * deg2rad):
            # print("Bike was in unsafe conditions or the code stopped. Putting the bike in a stable circle with a 5deg constant roll reference.")
            # self.balancing_setpoint = 5*deg2rad*np.sign(self.roll)
            # self.keep_the_bike_stable()
            print("[%f] Bike was in unsafe conditions or the code stopped. Stopping the bike and disabling motors." % (time.time() - self.run_start))
            self.stop()
        else:
            print("[%f] Conditions too extreme, bike probably felt down, turning off motors." % (time.time() - self.run_start))
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
        csv_path = './ExpData_zeroSpeed_%s/BikeData_zeroSpeed_%s.csv' % (bike, timestr)
        print "EXP LOG PATH is: "
        print csv_path
        results_csv = open(csv_path, 'wb')
        # results_csv = open('./ExpData_%s/BikeData_%s.csv' % (bike, timestr), 'wb')
        self.writer = csv.writer(results_csv)

        if path_tracking:
            self.writer.writerow(['Description : ' + str(self.descr) + ' ; walk_time = ' + str(walk_time) + ' ; speed_up_time = ' + str(speed_up_time) + ' ; balancing_time = ' + str(balancing_time)])
        else:
            self.writer.writerow(['Description : ' + str(self.descr) + ' ; walk_time = ' + str(walk_time) + ' ; speed_up_time = ' + str(speed_up_time) + ' ; balancing_time = ' + str(balancing_time)])

        self.log_header_str = ['RealTime','Time (s)', 'CalculationTime (s)', 'Roll (deg)', 'SteeringAngle (deg)', 'RollRate (deg)',
                               'ControlInput (deg)', 'gy (rad/s)', 'gz (rad/s)', 'ax (g)', 'ay (g)', 'az (g)', 'imu_read_timing (s)', 'SteerMotorCurrent (A)']

        # if self.deltadot:
        self.log_header_str += ['deltadot_computed']

        # if self.steering_acc:
        self.log_header_str += ['steering_acc']

        self.writer.writerow(self.log_header_str)

    def log_regular(self):
        # Log data
        self.time_log = time.time()
        self.log_str = [
            datetime.now().strftime("%H:%M:%S.%f"),
            "{0:.5f}".format(self.time_count),
            "{0:.5f}".format(self.loop_time),
            "{0:.5f}".format(self.roll*rad2deg),
            "{0:.5f}".format(self.steeringAngle*rad2deg),
            "{0:.5f}".format(self.rollRate_rec*rad2deg),
            "{0:.5f}".format(self.steering_rate*rad2deg),
            "{0:.5f}".format(self.gy),
            "{0:.5f}".format(self.gz),
            "{0:.5f}".format(self.ax),
            "{0:.5f}".format(self.ay),
            "{0:.5f}".format(self.az),
            "{0:.5f}".format(self.sensor_read_timing),
            "{0:.5f}".format(self.steeringCurrent)
        ]

        if self.deltadot:
            self.log_str += [
                "{0:.5f}".format(self.deltadot)
            ]

        if self.steering_acc:
            self.log_str += [
                "{0:.5f}".format(self.steering_acc)
            ]

        self.writer.writerow(self.log_str)
        self.time_log = time.time() - self.time_log

        if debug or (self.loop_time > sample_time):
            if self.loop_time > sample_time:
                print("[%f] WARNING : The calculation time exceeds the sampling time!" % (time.time() - self.run_start))
                self.exceedscount += 1

            # Print sensor reading time, control calculation time, IMU data reading time and logging time
            print("[%f] sensor_reading_time \t control calculation \t IMU \t log = %g \t %g \t %g \t %g \t" % (time.time() - self.run_start,
                self.sensor_reading_time, self.control_cal_time, self.time_get_states,
                self.time_log))

            # Stop experiment if exceeded sampling time too many times
            # if self.exceedscount > max_exceed_count:
            #     print(")
            #     self.safe_stop()
            # exc_msg = 'Calculation time exceeded sampling time too often (%d times) , aborting the experiment' % (max_exceed_count)
            # print(exc_msg)
            # self.exception_log(3, exc_msg)
        self.rollRate_prev = self.rollRate

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