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

#@pysnooper.snoop()
class Controller(object):
    # @pysnooper.snoop()
    def __init__(self, bike):
        self.bike = bike
        self.variable_init()

        # Check Estop before starting the experiment
        self.initial_Estop_Check()  # Check if the Estop Engaged

        # Get experiment (if any) of the experiment
        self.descr = raw_input('Type a description for the experiment if necessary. Press ENTER to start the experiment. ')

        # Load path

        if path_file != 'pot':
            print("Loading path ...")
            try:
                self.path_data = np.genfromtxt('paths/' + path_file, delimiter=",", skip_header=1)
                self.path_time = self.path_data[:,0]
                self.path_x = self.path_data[:,1]
                self.path_y = self.path_data[:,2]
                self.path_psi = self.path_data[:,3]
                print("Path Loaded, starting experiment.")
            except:
                print("Path file not found, setting all path references to 0 as default")
                self.path_data = np.array([[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]) # Using two rows with zeros for np.interp to work
                self.path_time = self.path_data[:,0]
                self.path_x = self.path_data[:,1]
                self.path_y = self.path_data[:,2]
                self.path_psi = self.path_data[:,3]

        # Create log file and add header line
        self.log_headerline()

        # Read the IMU complementary filter Phi as the initial phi estimation
        self.roll = self.bike.get_imu_data()[0]

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

        self.gaining_speed_start = time.time()

        while self.controller_active or not self.gainingSpeedOver_flag:
            self.time_start_current_loop = time.time()

            try:
                # Get velocity
                self.velocity = self.bike.get_velocity()

                # Get states
                self.time_get_states = time.time()
                self.get_states()
                self.time_get_states = time.time() - self.time_get_states

                # Compute Global Angles and Coordinates
                self.x_estimated, self.y_estimated, self.psi_estimated, self.nu_estimated = global_angles_and_coordinates(self.velocity, sample_time,
                                                                                                                          LENGTH_A, LENGTH_B,
                                                                                                                          self.steeringAngle,self.psi_estimated,
                                                                                                                          self.x_estimated, self.y_estimated)
                self.distance_travelled += self.velocity * sample_time

                # Get position from GPS
                if gps_use:
                    self.bike.get_gps_data()
                else:
                    self.x_measured_GPS = 0.0
                    self.y_measured_GPS = 0.0
                    self.lat_measured_GPS = 0.0
                    self.lon_measured_GPS = 0.0
                    self.gps_timestamp = 0.0

                # Get laser ranger position (y position on the roller)
                time_laserranger = time.time()
                if laserRanger_use:
                    if (time.time() - self.time_laserranger) > 1.1 * self.bike.laser_ranger.timing:
                        self.time_laserranger = time.time()
                        self.y_laser_ranger = self.bike.get_laserRanger_data()
                else:
                    self.y_laser_ranger = 0
                print("time laser ranger = %f" %(time.time()-time_laserranger))

                # Compute time needed to read from all sensors
                self.sensor_reading_time = time.time() - self.time_start_current_loop

                # PID Velocity Control
                if pid_velocity_active:
                    self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                    self.bike.set_velocity(self.pid_velocity_control_signal)

                # Let bike get up to speed for a few seconds before starting controllers
                if (self.time_count < speed_up_time) and not self.gainingSpeedOver_flag:
                    # Do not start controllers until bike ran for enough time to get up to speed
                    print('Gaining speed ...')
                    self.bike.set_velocity(initial_speed)
                elif (self.time_count >= speed_up_time) and not self.gainingSpeedOver_flag:
                    # Once enough time has passed, start controller
                    self.gainingSpeedOver_flag = True
                    print('Gaining speed phase over')

                    self.controller_active = True

                    # Clear PID controllers memory
                    self.pid_velocity.clear()
                    self.pid_balance.clear()
                    self.pid_balance_outerloop.clear()
                    self.pid_steeringangle.clear()
                    self.pid_lateral_position.clear()
                    self.pid_direction.clear()

                    # Restart PWM before using steering motor because it gets deactivated at the some point before in the code
                    # TO DO : CHECK WHY THIS HAPPENS !
                    PWM.start(steeringMotor_Channel, steeringMotor_IdleDuty, steeringMotor_Frequency)

                    # Time at which the controller starts running
                    self.time_start_controller = time.time()
                elif self.controller_active:
                    # Check steering angle
                    self.keep_handlebar_angle_within_safety_margins(self.steeringAngle)

                    # Balancing and path tracking control
                    self.bike.steering_motor.enable()
                    self.keep_the_bike_stable()

                # Compute time needed to run controllers
                self.control_cal_time = time.time() - self.time_start_current_loop - self.sensor_reading_time
            #except (ValueError, KeyboardInterrupt):
            except:
                print("Number of times sampling time was exceeded : %d" %(self.exceedscount))
                self.stop()
                print('Error or keyboard interrupt, aborting the experiment')

            # Control Frequency
            self.loop_time = time.time() - self.time_start_current_loop
            self.time_count = time.time() - self.gaining_speed_start
            
            # Log data
            self.log_regular() 

            # Check for ESTOP
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print('Emergency stop pressed, aborting the experiment')
                break

            # Check for extreme PHI
            if self.roll > MAX_LEAN_ANGLE or self.roll < MIN_LEAN_ANGLE:
                self.stop()
                print('Exceeded min/max lean (roll) angle, aborting the experiment')

            # End test time condition
            if self.time_count > test_duration:
                # Print number of times sampling time was exceeded if experiment is aborted early
                print('Exceeded test duration, aborting the experiment')
                self.stop()
                break

            # Compute total time for current loop
            self.loop_time = time.time() - self.time_start_current_loop
            # Sleep to match sampling time
            if self.loop_time < sample_time:
                time.sleep((sample_time - self.loop_time))

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
        self.gaining_speed_start = 0.0
        self.gainingSpeedOver_flag = False

        # Potentiometer
        self.pot = 0.0

        # Laser Ranger
        self.time_laserranger = 0.0

        # GPS
        self.gpspos = 0.0
        self.x_measured_GPS = 0.0
        self.y_measured_GPS = 0.0
        self.lat_measured_GPS = 0.0
        self.lon_measured_GPS = 0.0
        self.gps_timestamp = 0.0
        self.psi_measured_GPS = 0.0
        self.x_measured_GPS_old = 0.0
        self.y_measured_GPS_old = 0.0

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
        self.gps_timestamp = 0.0
        self.exceedscount = 0.0
        self.time_start_controller = 0.0

        # Bike States
        self.roll = 0.0
        self.roll_gyro = 0.0
        self.rollRate = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.velocity = 0.0
        self.AngVel = 0.0

        # Controller
        self.controller_active = False

        # Balancing Controller
        self.balancing_setpoint = 0.0
        self.steering_rate = 0.0
        self.steering_rate_previous = 0.0
        self.steering_rate_filt = 0.0
        self.steering_rate_filt_previous = 0.0

        # Path Trakcing Controller
        self.pos_ref = 0.0
        self.path_data = [0.0,0.0,0.0,0.0]
        self.path_time = 0.0
        self.path_x = 0.0
        self.path_y = 0.0
        self.path_psi = 0.0
        self.x_ref = 0.0
        self.y_ref = 0.0
        self.psi_ref = 0.0
        self.lateral_error = 0.0
        self.heading_error = 0.0

        # PID Balance Controller
        self.pid_balance = PID(pid_balance_P, pid_balance_I, pid_balance_D)
        self.pid_balance.setSampleTime(pid_balance_sample_time)
        self.pid_balance.setReference(pid_balance_reference)
        self.pid_balance_control_signal = 0.0  # control signal calculated by PID

        # PID Balance Controller Outer Loop
        self.pid_balance_outerloop = PID(pid_balance_outerloop_P, pid_balance_outerloop_I, pid_balance_outerloop_D)
        self.pid_balance_outerloop.setSampleTime(pid_balance_outerloop_sample_time)
        self.pid_balance_outerloop.setReference(pid_balance_outerloop_reference)
        self.pid_balance_outerloop_control_signal = 0.0  # control signal calculated by PID

        # PID Velocity Controller
        self.pid_velocity = PID(pid_velocity_P, pid_velocity_I, pid_velocity_D)
        self.pid_velocity.setSampleTime(pid_velocity_sample_time)
        self.pid_velocity.setReference(pid_velocity_reference)
        self.pid_velocity_control_signal = 0.0  # control signal calculated by PID

        # PID Lateral Position Controller
        self.pid_lateral_position = PID(pid_lateral_position_P, pid_lateral_position_I, pid_lateral_position_D)
        self.pid_lateral_position.setSampleTime(pid_lateral_position_sample_time)
        self.pid_lateral_position.setReference(pid_lateral_position_reference)
        self.pid_lateral_position_control_signal = 0.0  # control signal calculated by PID

        # PID Direction Controller
        self.pid_direction = PID(pid_direction_P, pid_direction_I, pid_direction_D)
        self.pid_direction.setSampleTime(pid_direction_sample_time)
        self.pid_direction.setReference(pid_direction_reference)
        self.pid_direction_control_signal = 0.0  # control signal calculated by PID

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

        # imu_data = [phi_comp, phi_gyro, gx (phidot), gy, gz, ax, ay, az]
        self.imu_data = self.bike.get_imu_data()

        # Extract states
        self.roll = self.imu_data[0]
        self.roll_gyro = self.imu_data[1]
        self.rollRate = self.imu_data[2]
        self.gy = self.imu_data[3]
        self.gz = self.imu_data[4]
        self.ax = self.imu_data[5]
        self.ay = self.imu_data[6]
        self.az = self.imu_data[7]


    ####################################################################################################################
    ####################################################################################################################
    # Get position from GPS
    # @pysnooper.snoop()
    def gps_read(self):
        # Get GPS position
        self.gpspos = self.bike.gps.get_position()
        self.x_measured_GPS = self.gpspos[0]
        self.y_measured_GPS = self.gpspos[1]
        self.psi_measured_GPS = np.arctan2(self.y_measured_GPS - self.y_measured_GPS_old,self.x_measured_GPS - self.x_measured_GPS_old)
        self.lat_measured_GPS = self.gpspos[2]
        self.lon_measured_GPS = self.gpspos[3]
        self.gps_timestamp = self.bike.gps.lastread - self.gaining_speed_start  # The gps timestamp

        # Save previous x and y positions to use in heading computation
        self.x_measured_GPS_old = self.x_measured_GPS
        self.y_measured_GPS_old = self.y_measured_GPS

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
            print('Exceeded MAX_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg))
            self.upperSaturation = True
            self.bike.stop_all()
        elif handlebar_angle < min_handlebar_angle:
            print('Exceeded MIN_HANDLEBAR_ANGLE of %f deg, aborting the experiment' % (max_handlebar_angle * rad2deg))
            self.lowerSaturation = True
            self.bike.stop_all()
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
                print('Emergency stop was not released, aborting the experiment')


    ####################################################################################################################
    ####################################################################################################################
    # Get bike states references
    def get_balancing_setpoint(self):
        if path_tracking:
            # Get reference position and heading
            if path_file == 'pot':
                self.x_ref = 0.0
                self.psi_ref = 0.0
                if potentiometer_use:
                    self.pot = ((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 0.2 - 0.1)  # Potentiometer gives a position reference between -0.1m and 0.1m
                    self.y_ref = self.pot
                else:
                    self.y_ref = 0.0
            else:
                idx_path_currentTime = bisect.bisect_left(self.path_time,time.time() - self.time_start_controller)+np.array([-1,0])
                self.x_ref = np.interp(time.time() - self.time_start_controller,self.path_time[idx_path_currentTime],self.path_x[idx_path_currentTime])
                self.y_ref = np.interp(time.time() - self.time_start_controller,self.path_time[idx_path_currentTime],self.path_y[idx_path_currentTime])
                self.psi_ref = np.interp(time.time() - self.time_start_controller,self.path_time[idx_path_currentTime],self.path_psi[idx_path_currentTime])

            # Compute position and heading errors
            if gps_use:
                # We are outside so we get the position and heading measurement from the GPS
                self.x_error = self.x_ref - self.x_measured_GPS
                self.y_error = self.y_ref - self.y_measured_GPS
                self.psi_error = self.psi_ref - self.psi_measured_GPS
            elif laserRanger_use:
                # We are on the roller so we neglect the angular error and the lateral error is given by the position
                # measured by the laser rangers
                self.x_error = 0
                self.y_error = self.y_ref - self.y_laser_ranger
                self.psi_error = 0
            else:
                self.x_error = 0
                self.y_error = 0
                self.psi_error = 0
                
            # Compute lateral and heading errors
            self.lateral_error = self.x_error * np.sin(self.psi_ref) + self.y_error * np.cos(self.psi_ref)
            self.heading_error = self.psi_error

            if path_tracking_structure == 'parallel':
                # PID Lateral Position Controller
                self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not maesurement
                # PID Direction/Heading Controller
                self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not maesurement
                # Compute balancing setpoint
                self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal
            elif path_tracking_structure == 'series':
                # PID Lateral Position Controller
                self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not maesurement
                if heading_controller:
                    # PID Direction/Heading Controller
                    self.pid_direction.setReference(lateralError_controller * self.pid_lateral_position_control_signal)
                    self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not maesurement
                    # Compute balancing setpoint
                    self.balancing_setpoint = self.pid_direction_control_signal
                else:
                    # Compute balancing setpoint
                    self.balancing_setpoint = self.pid_lateral_position_control_signal
            else:
                print("Path tracking controller structure choice is not valid : \"%s\"; Using parallel structure as default.\n" % (balancing_controller_structure))
                # PID Lateral Position Controller
                self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-self.lateral_error) # Minus sign due to using error and not maesurement
                # PID Direction/Heading Controller
                self.pid_direction_control_signal = self.pid_direction.update(-self.heading_error) # Minus sign due to using error and not maesurement
                # Compute balancing setpoint
                self.balancing_setpoint = lateralError_controller * self.pid_lateral_position_control_signal + heading_controller * self.pid_direction_control_signal
        elif potentiometer_use:
            self.pot = -((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 2.5 - 1.25) * deg2rad * 2 # Potentiometer gives a position reference between -2.5deg and 2.5deg
            self.balancing_setpoint = self.pot
        else:
            self.balancing_setpoint = 0


    ####################################################################################################################
    ####################################################################################################################
    # Balancing controller
    def keep_the_bike_stable(self):
        # if path_tracking:
        #     # Choice of path, variable path_choice is set in param.py file
        #     if path_choice == 'pot':
        #         # Position reference from potentiometer
        #         if potentiometer_use:
        #             self.pot = ((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 0.2 - 0.1)  # Potentiometer gives a position reference between -0.1m and 0.1m
        #         else:
        #             self.pot = 0
        #         self.pos_ref = self.pot
        #     elif path_choice == 'sine':
        #         # Position reference is a sine wave
        #         # Frequency is path_sine_freq
        #         # Amplitude is path_sine_amp
        #         self.pos_ref = path_sine_amp * math.sin(self.time_count * 2 * math.pi * path_sine_freq)
        #     elif path_choice == 'overtaking':
        #         # Position reference is an overtaking (set parameters in param.py file)
        #         # All sections going straight last time_path_stay
        #         # All inclined sections last time_path_slope
        #         # Slope of the inclined sections is slope
        #         self.pos_ref = slope * (self.time_count - time_path_stay) * (
        #                     time_path_stay < self.time_count < (time_path_stay + time_path_slope)) \
        #                        + slope * time_path_slope * ((time_path_stay + time_path_slope) < self.time_count < (
        #                     2 * time_path_stay + time_path_slope)) \
        #                        + (slope * time_path_slope - slope * (
        #                     self.time_count - (2 * time_path_stay + time_path_slope))) * (
        #                                    (2 * time_path_stay + time_path_slope) < self.time_count < (
        #                                        2 * time_path_stay + 2 * time_path_slope))
        #     self.pid_lateral_position.setReference(self.pos_ref)
        #     self.balancing_setpoint = self.pid_balance_outerloop.setReference(self.pid_lateral_position.update(self.y_laser_ranger))
        # elif potentiometer_use:
        #     self.pot = -((self.bike.get_potentiometer_value() / potentiometer_maxVoltage) * 2.5 - 1.25) * deg2rad * 1 # Potentiometer gives a position reference between -2.5deg and 2.5deg
        #     self.balancing_setpoint = self.pot
        # else:
        #     self.balancing_setpoint = 0

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
        self.steering_rate_previous = self.steering_rate
        # self.steering_rate = self.steering_rate_filt

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
        results_csv = open('./ExpData_%s/BikeData_%s.csv' % (bike, timestr), 'wb')
        self.writer = csv.writer(results_csv)

        self.writer.writerow(['Description : ' + str(self.descr)])

        self.log_header_str = ['Time', 'CalculationTime', 'MeasuredVelocity', 'Roll', 'SteeringAngle', 'RollRate',
                               'ControlInput', 'BalancingSetpoint', 'gy', 'gz', 'ax', 'ay', 'az', 'x_estimated', 'y_estimated', 'psi_estimated', 'nu_estimated']

        if potentiometer_use:
            self.log_header_str += ['Potentiometer']
        if gps_use:
            self.log_header_str += ['GPS_timestamp', 'x_GPS', 'y_GPS', 'latitude', 'longitude']
        if laserRanger_use:
            self.log_header_str += ['laserRanger_dist1', 'laserRanger_dist2', 'laserRanger_y']
        if path_tracking:
            self.log_header_str += ['x_ref', 'y_ref', 'psi_ref', 'lateral_error', 'heading_error']

        self.writer.writerow(self.log_header_str)

    def log_regular(self):
        #print("time = %f ; roll = %f ; steering = %f" % (self.time_count,self.roll,self.steeringAngle))

        # Log data
        self.time_log = time.time()
        self.log_str = [
            "{0:.5f}".format(self.time_count),
            "{0:.5f}".format(self.loop_time),
            "{0:.5f}".format(self.velocity),
            "{0:.5f}".format(self.roll),
            "{0:.5f}".format(self.steeringAngle),
            "{0:.5f}".format(self.rollRate),
            "{0:.5f}".format(self.pid_balance_control_signal),
            "{0:.5f}".format(self.balancing_setpoint),
            "{0:.5f}".format(self.gy),
            "{0:.5f}".format(self.gz),
            "{0:.5f}".format(self.ax),
            "{0:.5f}".format(self.ay),
            "{0:.5f}".format(self.az),
            "{0:.5f}".format(self.x_estimated),
            "{0:.5f}".format(self.y_estimated),
            "{0:.5f}".format(self.psi_estimated),
            "{0:.5f}".format(self.nu_estimated)
        ]

        if potentiometer_use:
            self.log_str += ["{0:.5f}".format(self.pot)]
        if gps_use:
            self.log_str += [
                "{0:.5f}".format(self.gps_timestamp),
                "{0:.5f}".format(self.x_measured_GPS),
                "{0:.5f}".format(self.y_measured_GPS),
                "{0:.5f}".format(self.lat_measured_GPS),
                "{0:.5f}".format(self.lon_measured_GPS)
            ]
        if laserRanger_use:
            self.log_str += [
                "{0:.5f}".format(self.bike.laser_ranger.distance1),
                "{0:.5f}".format(self.bike.laser_ranger.distance2),
                "{0:.5f}".format(self.y_laser_ranger)
            ]
        if path_tracking:
            self.log_str += [
                "{0:.5f}".format(self.x_ref),
                "{0:.5f}".format(self.y_ref),
                "{0:.5f}".format(self.psi_ref),
                "{0:.5f}".format(self.lateral_error),
                "{0:.5f}".format(self.heading_error)
            ]

        self.writer.writerow(self.log_str)

        self.time_log = time.time() - self.time_log

        if debug or (self.loop_time > sample_time):
            if self.loop_time > sample_time:
                print("Warning: The calculation time exceeds the sampling time!")
                self.exceedscount += 1

            # Print sensor reading time, control calculation time, IMU data reading time and logging time
            print("sensor_reading_time   control calculation   IMU   log  = %g \t %g \t %g \t %g \t" % (
                self.sensor_reading_time, self.control_cal_time, self.time_get_states,
                self.time_log))

            # Stop experiment if exceeded sampling time too many times
            # if self.exceedscount > max_exceed_count:
            #     print("Calculation time exceeded sampling time too often (%d times) , aborting the experiment" % (max_exceed_count))
            #     self.stop()