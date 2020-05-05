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

# @pysnooper.snoop()
class Controller(object):
    def __init__(self, bike):
        self.bike = bike
        self.variable_init()

        # Check Estop before starting the experiment
        self.initial_Estop_Check()  # Check if the Estop Engaged

        # Get experiment (if any) of the experiment
        self.descr = raw_input('Type a description for the experiment if necessary. Press ENTER to start the experiment. ')

        # Wait before starting experiment
        print("Experiment starting in %is" % start_up_interval)
        time.sleep(start_up_interval)

        # Read the IMU complementary filter Phi as the initial phi estimation
        self.states[0] = self.bike.get_imu_data()[0]

        # Create log file and add header line
        self.log_headerline()


    ####################################################################################################################
    ####################################################################################################################
    # Start-up bike
    def startup(self):
        # Let the bike get to speed
        self.gaining_speed_start = time.time()
        self.bike.set_velocity(initial_speed)
        while self.time_count < speed_up_time:
            time_start_current_loop = time.time()
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print('Emergency stop pressed, aborting the experiment')
                break
            else:
                print('Gaining speed ...')

            # Get states and calculate state_references
            self.velocity = self.bike.get_velocity()

            self.time_get_states = time.time()
            self.states_and_extra_data = self.get_states()
            self.time_get_states = time.time() - self.time_get_states

            self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
            self.extra_data = self.states_and_extra_data[3][:]

            self.sensor_reading_time = time.time() - time_start_current_loop
            # Get y position on the roller
            # print("Reading Laser at %d" %(time.time() - self.gaining_speed_start))
            if laserRanger_use:
                if (time.time() - self.time_laserranger) > 1.1 * self.bike.laser_ranger.timing:
                    self.y_laser_ranger = self.bike.laser_ranger.get_y()
                    self.time_laserranger = time.time()
            else:
                self.y_laser_ranger = 0
                self.time_laserranger = 0

            # self.gps_read()
            # Find Global Angles and Coordinates
            if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                self.x, self.y, self.psi, self.nu = global_angles_and_coordinates(self.velocity, sample_time,
                                                                                  LENGTH_A, LENGTH_B, self.states[1],
                                                                                  self.psi, self.x, self.y)
            # PID Velocity Control
            if pid_velocity_active:
                self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                self.bike.set_velocity(self.pid_velocity_control_signal)

            # Check for ESTOP
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print('Emergency stop pressed, aborting the experiment')
                break

            # Check for extreme PHI
            if self.states[0] > MAX_LEAN_ANGLE or self.states[0] < MIN_LEAN_ANGLE:
                self.stop()
                print('Exceeded min/max lean (roll) angle, aborting the experiment')

            self.status_check_time = time.time() - time_start_current_loop - self.sensor_reading_time
            # Do a speed test
            # self.bike.set_velocity(initial_speed)
            pid_velocity_reference = initial_speed
            self.lqr_balance_control_signal = 0

            # # Calculation Time
            # self.calculation_time = time.time() - time_start_current_loop
            # self.time_count = time.time() - self.gaining_speed_start
            # # Log data
            # self.log_regular() # timestamp self.time_count, self.calculation_time
            # # self.status_print()

            self.time_count = time.time() - self.gaining_speed_start
            self.calculation_time = time.time() - time_start_current_loop  # The time elapsed
            self.log_regular()  # self.time_count, self.calculation_time
            self.calculation_time = time.time() - time_start_current_loop  # The time elapsed
            if self.calculation_time < sample_time:
                time.sleep((sample_time - self.calculation_time))

            # Log data

        print('Gaining speed phase over')

        self.states[0] = self.bike.get_imu_data()[0]  # Reset State as Arduino Roll data again
        self.controller_active = True
        self.bike.steering_motor.enable()
        # self.thread = Thread(target=lambda: self.worker())
        # self.thread.start()

        self.pid_balance.clear()
        self.pid_steeringangle.clear()


    ####################################################################################################################
    ####################################################################################################################
    # Run bike
    # @pysnooper.snoop()
    def run(self):
        self.pid_velocity.clear()
        self.pid_balance.clear()
        self.pid_balance_outerloop.clear()
        self.pid_steeringangle.clear()
        self.pid_lateral_position.clear()
        self.pid_direction.clear()

        self.gaining_speed_start = time.time()

        while self.controller_active or not self.gainingSpeedOver_flag:
            time_start_current_loop = time.time()

            try:

                # Get states and calculate state_references
                self.velocity = self.bike.get_velocity()

                self.time_get_states = time.time()
                self.states_and_extra_data = self.get_states()
                self.time_get_states = time.time() - self.time_get_states

                self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
                self.extra_data = self.states_and_extra_data[3][:]  # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
                self.sensor_reading_time = time.time() - time_start_current_loop

                # self.gps_read()
                # Find Global Angles and Coordinates
                self.x, self.y, self.psi, self.nu = global_angles_and_coordinates(self.velocity, sample_time,
                                                                                  LENGTH_A, LENGTH_B,
                                                                                  self.states[1],
                                                                                  self.psi, self.x, self.y)
                self.distance_travelled += self.velocity * sample_time

                # Get y position on the roller
                if potentiometer_use:
                    if (time.time() - self.time_laserranger) > 1.1 * self.bike.laser_ranger.timing:
                        self.y_laser_ranger = self.bike.laser_ranger.get_y()
                else:
                    self.y_laser_ranger = 0
                self.time_laserranger = time.time()

                # PID Velocity Control
                if pid_velocity_active:
                    self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                    self.bike.set_velocity(self.pid_velocity_control_signal)

                self.sensor_reading_time = time.time() - time_start_current_loop


                if (self.time_count < speed_up_time) and not self.gainingSpeedOver_flag:
                    # Do not start controllers until bike ran for enough time to get up to speed
                    print('Gaining speed ...')
                elif (self.time_count >= speed_up_time) and not self.gainingSpeedOver_flag:
                    # Once enough time has passed, start controller
                    self.gainingSpeedOver_flag = True

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
                else:
                    # Check steering angle
                    self.keep_handlebar_angle_within_safety_margins(self.states[1])

                    # Balancing and path tracking control
                    self.bike.steering_motor.enable()
                    self.keep_the_bike_stable(self.states)

                self.control_cal_time = time.time() - time_start_current_loop - self.sensor_reading_time
            except (ValueError, KeyboardInterrupt):
                self.stop()
                print('Error or keyboard interrupt, aborting the experiment')

            # Check for ESTOP
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print('Emergency stop pressed, aborting the experiment')
                break

            # Check for extreme PHI
            if self.states[0] > MAX_LEAN_ANGLE or self.states[0] < MIN_LEAN_ANGLE:
                self.stop()
                print('Exceeded min/max lean (roll) angle, aborting the experiment')

            # End test time condition
            if self.time_count > test_duration:
                self.stop()
                print('Exceeded test duration, aborting the experiment')
                break

            self.status_check_time = time.time() - time_start_current_loop - self.control_cal_time - self.sensor_reading_time
            # Calculation Time
            # self.calculation_time = time.time() - time_start_current_loop
            # self.time_count = time.time() - self.gaining_speed_start
            #
            # # Log data
            # self.log_regular() #self.time_count, self.calculation_time
            # self.status_print()

            # Control Frequency
            self.calculation_time = time.time() - time_start_current_loop
            self.time_count = time.time() - self.gaining_speed_start
            # Log data
            self.log_regular()  # self.time_count, self.calculation_time

            self.calculation_time = time.time() - time_start_current_loop
            # print('Time spent' + str(self.calculation_time))
            if self.calculation_time < sample_time:
                time.sleep((sample_time - self.calculation_time))


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
        # Gaining speed
        self.gainingSpeedOver_flag = False

        # Bike
        self.time_count = 0.0
        self.controller_start_time = 0.0
        self.calculation_time = 0.0
        self.gaining_speed_start = 0.0
        self.gaining_speed_start = 0.0
        self.states = [0.0, 0.0, 0.0]
        self.velocity = 0.0
        self.distance_travelled = 0.0
        self.ESTOP = False
        self.exceedscount = 0
        self.sensor_reading_time = 0
        self.status_check_time = 0
        self.control_cal_time = 0
        self.latest_gps_time = 0
        self.ini_comp_read = 0.0
        self.AngVel = 0.0
        self.AngVel_prec = 0.0
        self.AngVel_filt = 0.0
        self.AngVel_filt_prec = 0.0
        self.ref_diff_old = np.zeros((2, 1))
        self.steering_rate = 0.0
        self.steering_rate_previous = 0.0
        self.steering_rate_filt = 0.0
        self.steering_rate_filt_previous = 0.0
        self.pos_ref = 0

        # Controller
        self.controller_active = False
        self.calculation_time = 0.0
        self.complementary_coef = imu_complementaryFilterRatio
        self.CP_comp = imu_centripetal_compensation
        self.roll_comp = imu_roll_compensation

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

        # PID Steering Controller
        self.pid_steering = PID(pid_steering_P, pid_steering_I, pid_steering_D)
        self.pid_steering.setSampleTime(pid_steering_sample_time)
        self.pid_steering.setReference(pid_steering_reference)
        self.pid_steering_control_signal = 0.0  # This signal is called "Delta Reference" in Simulink.

        # PID Steering ANGLE Controller
        self.pid_steeringangle = PID(pid_steeringangle_P, pid_steeringangle_I, pid_steeringangle_D)
        self.pid_steeringangle.setSampleTime(pid_steeringangle_sample_time)
        self.pid_steeringangle.setReference(0.0)
        self.pid_steeringangle_control_signal = 0.0  # This signal is called "Delta Reference" in Simulink.

        self.balancing_setpoint = 0

        # Global Angles and Coordinates
        self.psi = 0.0
        self.nu = 0.0
        if PATH_TYPE == 'CIRCLE':
            self.x = PATH_RADIUS
        else:
            self.x = 0.0
        self.y = 0.0

        # self.delta_state_previous = 0
        self.delta_state_list = list()

        self.descr = 'NoComments'

        # Laser Ranger
        self.time_laserranger = 0


    ####################################################################################################################
    ####################################################################################################################
    # Get bike states
    def get_states(self):
        delta_state = self.bike.get_handlebar_angle()

        # imu_data = [phi_comp, phi_gyro, gx (phidot), gy, gz, a_x, ay, a_z]
        imu_data = self.bike.get_imu_data()
        phi_state = imu_data[0]
        phi_d_state = imu_data[2]

        return [phi_state, delta_state, phi_d_state, imu_data]


    ####################################################################################################################
    ####################################################################################################################
    # Get bike states references
    def get_balancing_setpoint(self, velocity):
        if path_tracking:
            if gps_use:
                # We are outside so we get the lateral and angular errors from GPS measurements
                lateral_error, angular_error, psi_ref = calculate_path_error(PATH_TYPE, self.distance_travelled, self.x,
                                                                          self.y, self.psi, PATH_RADIUS)
            elif laserRanger_use:
                # We are on the roller so we neglect the angular error and the lateral error is given by the position
                # measured by the laser rangers
                lateral_error = self.y_laser_ranger
                angular_error = 0
            else:
                lateral_error = 0
                angular_error = 0
                
            # PID Lateral Position Controller
            self.pid_lateral_position_control_signal = self.pid_lateral_position.update(-lateral_error)

            # PID Direction/Heading Controller
            self.pid_direction_control_signal = self.pid_direction.update(-angular_error)

            # Parallel path tracking controller structure
            balancing_setpoint = self.pid_lateral_position_control_signal + self.pid_direction_control_signal
        else:
            balancing_setpoint = 0
        return balancing_setpoint


    ####################################################################################################################
    ####################################################################################################################
    # Get position from GPS
    # @pysnooper.snoop()
    def gps_read(self):
        # Get GPS position
        self.gpspos = self.bike.gps.get_position()
        self.x_measured_GPS = self.gpspos[0]
        self.y_measured_GPS = self.gpspos[1]
        self.latest_gps_time = self.bike.gps.lastread - self.gaining_speed_start  # The gps timestamp
        # Update Kalman filter
        # position_kalman = self.bike.kf.update(x_measured_GPS, y_measured_GPS, self.states[1], self.velocity)
        # print('x_measured_GPS = %f ; y_measured_GPS = %f ; x_kalman = %f ; y_kalman = %f' % (self.x_measured_GPS, self.y_measured_GPS, position_kalman[0], position_kalman[1]))
        if debug:
            print('GPS : x_measured_GPS = %f ; y_measured_GPS = %f ;' % (self.x_measured_GPS, self.y_measured_GPS))


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
    # Balancing and path tracking controllers
    def keep_the_bike_stable(self, states):
        if path_tracking:
            # Choice of path, variable path_choice is set in param.py file
            if path_choice == 'pot':
                # Position reference from potentiometer
                if potentiometer_use:
                    self.potential = ((self.bike.potent.read_pot_value() / 0.29) * 0.2 - 0.1)  # Potentiometer gives a position reference between -0.1m and 0.1m
                else:
                    self.potential = 0
                self.pos_ref = self.potential
            elif path_choice == 'sine':
                # Position reference is a sine wave
                # Frequency is path_sine_freq
                # Amplitude is path_sine_amp
                self.pos_ref = path_sine_amp * math.sin(self.time_count * 2 * math.pi * path_sine_freq)
            elif path_choice == 'overtaking':
                # Position reference is an overtaking (set parameters in param.py file)
                # All sections going straight last time_path_stay
                # All inclined sections last time_path_slope
                # Slope of the inclined sections is slope
                self.pos_ref = slope * (self.time_count - time_path_stay) * (
                            time_path_stay < self.time_count < (time_path_stay + time_path_slope)) \
                               + slope * time_path_slope * ((time_path_stay + time_path_slope) < self.time_count < (
                            2 * time_path_stay + time_path_slope)) \
                               + (slope * time_path_slope - slope * (
                            self.time_count - (2 * time_path_stay + time_path_slope))) * (
                                           (2 * time_path_stay + time_path_slope) < self.time_count < (
                                               2 * time_path_stay + 2 * time_path_slope))
            self.pid_lateral_position.setReference(self.pos_ref)
            self.balancing_setpoint = self.pid_balance_outerloop.setReference(self.pid_lateral_position.update(self.y_laser_ranger))
        elif potentiometer_use:
            self.potential = -((self.bike.potent.read_pot_value() / 0.29) * 2.5 - 1.25) * deg2rad * 2  # Potentiometer gives a position reference between -2.5deg and 2.5deg
            self.balancing_setpoint = self.potential
        else:
            self.balancing_setpoint = 0

        if balancing_controller_structure == 'chalmers':
            # Chalmers Controller structure : deltadot = PID(phidot)
            self.pid_balance.setReference(self.balancing_setpoint)
            self.pid_balance_control_signal = self.pid_balance.update(states[2])
            self.steering_rate = self.pid_balance_control_signal
        elif balancing_controller_structure == 'technion':
            # Technion Controller Structure : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
            self.pid_balance_outerloop.setReference(self.balancing_setpoint)
            self.pid_balance.setReference(self.pid_balance_outerloop.update(states[0]))
            self.pid_balance_control_signal = self.pid_balance.update(states[2])
            self.steering_rate = self.pid_balance_control_signal
        elif balancing_controller_structure == 'mdh':
            # MDH Controller structure : delta = PID(phi) ; deltadot = PID(delta)
            self.pid_balance.setReference(self.balancing_setpoint)
            self.pid_balance_control_signal = self.pid_balance.update(states[0])
            self.steering_rate = self.pos2vel(self.pid_balance_control_signal)
        else:
            print("Bike controller structure choice is not valid : \"%s\"; Using Technion's structure as default.\n" % (balancing_controller_structure))
            # Technion Controller Structure : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
            self.pid_balance_outerloop.setReference(self.balancing_setpoint)
            self.pid_balance.setReference(self.pid_balance_outerloop.update(states[0]))
            self.pid_balance_control_signal = self.pid_balance.update(states[2])
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

    def pos2vel(self, AngRef):
        self.pid_steeringangle.setReference(AngRef)
        # self.AngVel = self.pid_steeringangle.update(-self.states[1])
        self.AngVel = self.pid_steeringangle.update(self.states[1])
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

        self.log_header_str = ['Time', 'CalculationTime', 'MeasuredVelocity', 'phi', 'delta', 'phidot',
                               'ControlInput', 'ax', 'ay', 'az', 'x', 'y', 'psi', 'nu']

        if potentiometer_use:
            self.log_header_str += ['Potentiometer']
        if gps_use:
            self.log_header_str += ['GPS_timestamp', 'GPS_x', 'GPS_y', 'latitude', 'longitude']
        if laserRanger_use:
            self.log_header_str += ['laserRanger_dist1', 'laserRanger_dist2', 'laserRanger_y']

        self.writer.writerow(self.log_header_str)

    def log_regular(self):
        # Log data
        self.time_log = time.time()
        self.log_str = [
            "{0:.5f}".format(self.time_count),
            "{0:.5f}".format(self.calculation_time),
            "{0:.5f}".format(self.velocity),
            "{0:.5f}".format(self.states[0]),
            "{0:.5f}".format(self.states[1]),
            "{0:.5f}".format(self.states[2]),
            "{0:.5f}".format(self.pid_balance_control_signal),
            "{0:.5f}".format(self.extra_data[5]),
            "{0:.5f}".format(self.extra_data[6]),
            "{0:.5f}".format(self.extra_data[7]),
            "{0:.5f}".format(self.x),
            "{0:.5f}".format(self.y),
            "{0:.5f}".format(self.psi),
            "{0:.5f}".format(self.nu)
        ]

        if potentiometer_use:
            self.log_str += ["{0:.5f}".format(self.potential)]
        if gps_use:
            self.log_str += [
                "{0:.5f}".format(self.latest_gps_time),
                "{0:.5f}".format(self.gpspos[0]),
                "{0:.5f}".format(self.gpspos[1]),
                "{0:.5f}".format(self.bike.gps.latitude),
                "{0:.5f}".format(self.bike.gps.longitude)
            ]
        if laserRanger_use:
            self.log_str += [
                "{0:.5f}".format(self.bike.laser_ranger.distance1),
                "{0:.5f}".format(self.bike.laser_ranger.distance2),
                "{0:.5f}".format(self.y_laser_ranger)
            ]

        self.writer.writerow(self.log_str)

        self.time_log = time.time() - self.time_log

        if debug:
            # Print sensor reading time, control calculation time, IMU data reading time and logging time
            print("sensor_reading_time, control calculation, status_check   IMU   log  = %g \t %g \t %g \t %g \t %g \t" % (
                self.sensor_reading_time, self.control_cal_time, self.status_check_time, self.time_get_states,
                self.time_log))

            # Check the calculation time
            if self.calculation_time > sample_time:
                print("Warning: The calculation time exceeds the sampling time!")
                self.exceedscount += 1
                print("sensor_reading_time, control calculation, status_check = %g \t %g \t %g" 
                      % (self.sensor_reading_time, self.control_cal_time, self.status_check_time))
                if self.exceedscount > max_exceed_count:
                    print("Calculation time exceeded sampling time too often (%d times) , aborting the experiment" % (max_exceed_count))
                    self.stop()
