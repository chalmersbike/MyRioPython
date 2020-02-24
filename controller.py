from param import *
import csv
import math
import time
import numpy

from time import sleep

from constants import *
from utils import *
import Adafruit_BBIO.ADC as ADC
from actuators import butter_lowpass, butter_lowpass_filter, moving_average_filter
#import pysnooper


# @pysnooper.snoop('./log/file.log')
class Controller(object):

    def __init__(self, bike):

        self.bike = bike
        self.variable_init()

    def startup(self):

        self.initial_Estop_Check()  # Check if the Estop Engaged
        self.descr = raw_input('type the description if necessary, Press ENTER to start,')
        print "start in %i secs" % start_up_interval
        time.sleep(start_up_interval)
        self.states[0] = self.bike.get_imu_data()[
            0]  # Read the IMU complementary filter Phi as the initial phi estimation

        self.log_headerline()

        # Let the bike get to speed

        self.gaining_speed_start = time.time()

        self.bike.set_velocity(INITIAL_SPEED)

        while self.time_count < speed_up_time:
            start_time_current_loop = time.time()
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print 'EMERGENCY STOP'
                break
            else:
                print 'GAINING SPEED'

            # Get states and calculate state_references
            self.velocity = self.bike.get_velocity()
            self.states_and_extra_data = self.get_states()
            self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
            self.extra_data = self.states_and_extra_data[3][
                              :]  # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]

            self.sensor_reading_time = time.time() - start_time_current_loop
            # Get y position on the roller
            # print "reading Laser at %d" %(time.time() - self.gaining_speed_start)
            # self.y_laser_ranger = self.bike.laser_ranger.get_y()
            self.state_references = self.get_state_references(self.velocity)
            # self.gps_read()
            # Find Global Angles and Coordinates
            if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                self.x, self.y, self.psi, self.nu = global_angles_and_coordinates(self.velocity, self.TIME_CONSTANT,
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
                print 'EMERGENCY STOP'
                break

            # Check for extreme PHI
            if self.states[0] > MAX_LEAN_ANGLE or self.states[0] < MIN_LEAN_ANGLE:
                self.stop()
                print 'Exceeded MAX/MIN LEAN ANGLE'

            self.status_check_time = time.time() - start_time_current_loop - self.sensor_reading_time
            # Do a speed test
            # self.bike.set_velocity(INITIAL_SPEED)
            pid_velocity_reference = INITIAL_SPEED
            self.lqr_balance_control_signal = 0

            # # Calculation Time
            # self.calculation_time = time.time() - start_time_current_loop
            # self.time_count = time.time() - self.gaining_speed_start
            # # Log data
            # self.log_regular() # timestamp self.time_count, self.calculation_time
            # # self.status_print()

            self.time_count = time.time() - self.gaining_speed_start
            self.calculation_time = time.time() - start_time_current_loop  # The time elapsed
            self.log_regular()  # self.time_count, self.calculation_time
            self.calculation_time = time.time() - start_time_current_loop  # The time elapsed
            if self.calculation_time < self.TIME_CONSTANT:
                time.sleep((self.TIME_CONSTANT - self.calculation_time))

            # Log data

        self.states[0] = self.bike.get_imu_data()[0]  # Reset State as Arduino Roll data again
        self.controller_active = True
        self.bike.steering_motor.enable()
        # self.thread = Thread(target=lambda: self.worker())
        # self.thread.start()

    # @pysnooper.snoop()
    def run(self):

        while self.controller_active:
            start_time_current_loop = time.time()

            try:

                # Get states and calculate state_references
                self.velocity = self.bike.get_velocity()
                self.states_and_extra_data = self.get_states()
                self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
                self.extra_data = self.states_and_extra_data[3][
                                  :]  # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]

                # self.gps_read()
                # Find Global Angles and Coordinates
                if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                    self.x, self.y, self.psi, self.nu = global_angles_and_coordinates(self.velocity, self.TIME_CONSTANT,
                                                                                      LENGTH_A, LENGTH_B,
                                                                                      self.states[1],
                                                                                      self.psi, self.x, self.y)
                self.state_references = self.get_state_references(self.velocity)

                # print "reading Laser at %f" % (time.time() - self.gaining_speed_start)
                # Get y position on the roller
                # self.y_laser_ranger = self.bike.laser_ranger.get_y()

                # PID Velocity Control
                if pid_velocity_active:
                    self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                    self.bike.set_velocity(self.pid_velocity_control_signal)

                if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                    self.distance_travelled += self.velocity * self.TIME_CONSTANT
                self.sensor_reading_time = time.time() - start_time_current_loop

                # Balance Control
                self.keep_handlebar_angle_within_safety_margins(self.states[1])
                self.keep_the_bike_stable(self.velocity, self.state_references, self.states)

                self.control_cal_time = time.time() - start_time_current_loop - self.sensor_reading_time
            except (ValueError, KeyboardInterrupt):
                self.stop()
                print 'BREAK'

            # Check for ESTOP
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print 'EMERGENCY STOP'
                break

            # Check for extreme PHI
            if self.states[0] > MAX_LEAN_ANGLE or self.states[0] < MIN_LEAN_ANGLE:
                self.stop()
                print 'Exceeded MAX/MIN LEAN ANGLE'

            # End test time condition
            if self.time_count > TEST_TIME:
                self.stop()
                print 'Exceeded TEST TIME'
                break
            self.status_check_time = time.time() - start_time_current_loop - self.control_cal_time - self.sensor_reading_time
            # Calculation Time
            # self.calculation_time = time.time() - start_time_current_loop
            # self.time_count = time.time() - self.gaining_speed_start
            #
            # # Log data
            # self.log_regular() #self.time_count, self.calculation_time
            # self.status_print()

            # Control Frequency
            self.calculation_time = time.time() - start_time_current_loop

            if self.calculation_time < self.TIME_CONSTANT:
                time.sleep((self.TIME_CONSTANT - self.calculation_time))
            self.time_count = time.time() - self.gaining_speed_start
            # Log data
            self.log_regular()  # self.time_count, self.calculation_time

    def stop(self):
        self.controller_active = False
        self.bike.stop_all()

    def get_states(self):
        delta_state = self.bike.get_handlebar_angle()
        # try:
        #     if abs(delta_state - self.delta_state_previous) > abs(2*sample_time*self.lqr_balance_control_signal) and abs(delta_state - self.delta_state_previous) > 0.01:
        #         print 'Time = ' + str(self.time_count) + ' ; delta_state = ' + str(delta_state) + ' ; delta_state_previous = ' + str(self.delta_state_previous)
        #         delta_state = self.delta_state_previous + sample_time*self.lqr_balance_control_signal
        # except:
        #     pass
        # self.delta_state_previous = delta_state

        ##self.delta_state_list.append(delta_state)
        # self.delta_state_list = np.append(self.delta_state_list,delta_state)
        ##self.delta_state_list = butter_lowpass_filter(self.delta_state_list, cutoff, fs, order)
        # self.delta_state_list = moving_average_filter(self.delta_state_list, windowSize)
        # delta_state_filt = self.delta_state_list[-1]
        # print 'delta_state = ' + str(delta_state) +  ' ; delta_state_filt = ' + str(delta_state_filt)
        ##delta_state = delta_state_filt

        imu_data = self.bike.get_imu_data()

        # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
        phi_d_state = imu_data[2]
        ay = imu_data[5 - self.roll_comp]  # 4=roll compensated, 5=roll uncompensated
        az = imu_data[6]

        b = 1.095  # length between wheel centers [m]
        CP_acc_g = self.CP_comp * ((self.velocity * self.velocity) / b) * math.tan(delta_state * 0.94) * (
                    1 / 9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
        phi_acc = math.atan2(ay - CP_acc_g * math.cos(self.states[0]), az + CP_acc_g * math.sin(self.states[
                                                                                                    0]))  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        ## phi_acc=math.atan2(ay,  az)
        # phi_state = self.complementary_coef * (self.states[0]+phi_d_state*self.TIME_CONSTANT) + (1 - self.complementary_coef) * phi_acc
        ## phi_state = self.complementary_coef * (imu_data[0]) + (
        ##             1 - self.complementary_coef) * phi_acc
        ## phi_state = (imu_data[0])
        ## phi_state = imu_data[0] # for only roll compensated phi
        phi_state = imu_data[1]  # for uncompensated phi
        # if self.ini_comp_read == 0.0:
        #     self.ini_comp_read = imu_data[0]-imu_data[7]
        # phi_state = imu_data[7]  + self.ini_comp_read

        return [phi_state, delta_state, phi_d_state, imu_data]

    def get_state_references(self, velocity):

        if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
            lateral_error, angular_error, psi_ref = calculate__path_error(PATH_TYPE, self.distance_travelled, self.x,
                                                                          self.y, self.psi, PATH_RADIUS)

            # lateral_error = self.y_laser_ranger

            # PID Control Lateral Position Block in Simulink
            self.pid_lateral_position_control_signal = self.pid_lateral_position.update(
                lateral_error)  # Negative input due to PID code. See initiliazation for more info.
            # self.pid_lateral_position_control_signal = self.pid_lateral_position.update(
            #    lateral_error)  # Negative input due to PID code. See initiliazation for more info.

            # PID Control Direction Block in Simulink
            self.pid_direction_control_signal = self.pid_direction.update(
                -(-self.pid_lateral_position_control_signal + psi_ref - self.nu))  # Negative input due to PID code.
            # self.pid_direction_control_signal = self.pid_direction.update(
            #    (-self.pid_lateral_position_control_signal + psi_ref - self.nu))  # Negative input due to PID code.

            # PID Control Steering Block in Simulink
            self.pid_steering_control_signal = self.pid_steering.update(
                -(self.pid_direction_control_signal - self.states[1]))
            # self.pid_steering_control_signal = self.pid_steering.update(
            #    (self.pid_direction_control_signal - self.states[1]))

            delta_ref = self.pid_steering_control_signal
            phi_ref = -math.atan(((numpy.power(velocity, 2) / (GRAVITY * LENGTH_B)) * delta_ref))

            # print 'lateral_error = ' + str(lateral_error) + ' ; angular_error = ' + str(angular_error) + ' ; psi_ref = ' + str(delta_ref) + ' ; psi_ref = ' + str(delta_ref) + ' ; pid_lateral_position_control_signal = ' + str(self.pid_lateral_position_control_signal) + str(psi_ref) + ' ; pid_direction_control_signal = ' + str(self.pid_direction_control_signal) + ' ; pid_steering_control_signal = ' + str(self.pid_steering_control_signal )
        else:
            delta_ref = ((self.bike.potent.read_pot_value() / 0.29) * 5 - 2.5) * deg2rad
            # delta_ref = 0.0
            phi_ref = 0.0

        return [phi_ref, delta_ref, 0.0]

    def get_feedback_gains(self, velocity):

        if velocity > 4.0:
            velocity_saturated = 4.0
        elif velocity < 1.5:
            velocity_saturated = 1.5
        else:
            velocity_saturated = velocity

        vel = velocity_saturated

        # return [np.interp(velocity, K_VELOCITY_LOOKUP, gain['LQR_gain_multipliers']) for gain in K_MATRIX]
        # return [-18.7580, 4.1700, -4.9389]
        # return [-22.3223,    5.3046,   -5.4850]  # for 2m/s 50hz
        # return [-15.9870,    6.0476,   -3.9479]  # for 2.5m/s 50hz
        # return [-12.3295,    6.6724,   -3.0655]  # for 3m/s 50hz
        # return [-10.0619,    7.2404,   -2.5222]  # for 3.5m/s 50hz

        # return [-20.8204,    5.0703,   -5.1136]  # for 2m/s 25hz
        # return [-14.8850,    5.7641,   -3.6725]  # for 2.5m/s 25hz
        # return [-11.4480,    6.3461,   -2.8422]  # for 3m/s 25hz
        # return [-11.4480,    6.3461,   -2.8422]  # for 3.5m/s 25hz
        if fixed_gains == 0:
            vel3 = vel * vel * vel
            vel2 = vel * vel
            gain1 = P1[0] * vel3 + P1[1] * vel2 + P1[2] * vel + P1[3]
            gain2 = P2[0] * vel3 + P2[1] * vel2 + P2[2] * vel + P2[3]
            gain3 = P3[0] * vel3 + P3[1] * vel2 + P3[2] * vel + P3[3]
            return [gain1, gain2, gain3]
            # return [gain1/2, gain2/2, gain3/2]
            # return [gain1*2, gain2*2, gain3*2]
            # return [gain1*1, gain2*2, gain3*0.5]
        else:
            return gains

    def controller_set_handlebar_angular_velocity(self, angular_velocity):
        if MAX_HANDLEBAR_ANGLE_ADJUST == 1:
            if self.upperSaturation:
                if angular_velocity > 0:  # we have exceeded the limit and are trying to move beyond it
                    self.lqr_balance_control_signal = -HANDLEBAR_CORRECTION_ANGVEL
                    self.bike.set_handlebar_angular_velocity(-HANDLEBAR_CORRECTION_ANGVEL)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            elif self.lowerSaturation:
                if angular_velocity < 0:  # we have exceeded the limit and are trying to move beyond it
                    self.lqr_balance_control_signal = HANDLEBAR_CORRECTION_ANGVEL
                    self.bike.set_handlebar_angular_velocity(HANDLEBAR_CORRECTION_ANGVEL)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            else:
                self.bike.set_handlebar_angular_velocity(angular_velocity)
        else:
            if self.upperSaturation:
                if angular_velocity > 0:  # we have exceeded the limit and are trying to move beyond it
                    self.lqr_balance_control_signal = 0
                    self.bike.set_handlebar_angular_velocity(0)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            elif self.lowerSaturation:
                if angular_velocity < 0:  # we have exceeded the limit and are trying to move beyond it
                    self.lqr_balance_control_signal = 0
                    self.bike.set_handlebar_angular_velocity(0)
                else:  # we have exceeded the limit and are trying to move away from it
                    self.bike.set_handlebar_angular_velocity(angular_velocity)
            else:
                self.bike.set_handlebar_angular_velocity(angular_velocity)

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
            print 'Exceeded MAX_HANDLEBAR_ANGLE of %f deg' % (max_handlebar_angle * 57.3)
            self.upperSaturation = True
        elif handlebar_angle < min_handlebar_angle:
            print 'Exceeded MIN_HANDLEBAR_ANGLE of %f deg' % (max_handlebar_angle * 57.3)
            self.lowerSaturation = True
        else:
            self.upperSaturation = False
            self.lowerSaturation = False

    def keep_the_bike_stable(self, velocity, references, states):

        feedback_gains = self.get_feedback_gains(velocity)

        self.lqr_balance_control_signal = sum([(r - x) * k for r, x, k in zip(
            references, states, feedback_gains
        )])

        # print 'velocity = ' + str(velocity) + ' ; references = ' + str(references) + ' ; states = ' + str(states) + ' ; feedback_gains = ' + str(feedback_gains) + ' ; lqr_balance_control_signal = ' + str(self.lqr_balance_control_signal)
        ##print 'velocity = %.3f ; references = %.3f ; states = %.3f ; feedback_gains = %.3f ; lqr_balance_control_signal = %.3f' % (velocity,57.29577*references,57.29577*states,feedback_gains,57.29577*self.lqr_balance_control_signal)
        ##print 'velocity = %.3f ; references = %.3f ; states = %.3f' % (velocity,references,57.29577*states)

        # self.pid_balance_control_signal = self.pid_balance.update(states[0])
        # print 'phi(deg) = ' + str(57.29577*states[0]) + ' ; pid_signal(deg) = ' + str(57.29577*self.pid_balance_control_signal)

        if self.lqr_balance_control_signal > deadband or self.lqr_balance_control_signal < -deadband:
            self.controller_set_handlebar_angular_velocity(self.lqr_balance_control_signal)
            # self.controller_set_handlebar_angular_velocity(self.pid_balance_control_signal)
        else:
            self.controller_set_handlebar_angular_velocity(0)

    def variable_init(self):

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

        # Controller
        self.controller_active = False
        self.TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
        self.calculation_time = 0.0
        self.complementary_coef = comp_coef
        self.CP_comp = centripetal_compensation
        self.roll_comp = roll_compensation

        # LQR Balance Controller
        self.state_references = []
        self.state_references = [state['default_reference'] for state in K_MATRIX]
        self.lqr_balance_input = 0.0
        self.lqr_balance_control_signal = 0

        # PID Balance Controller
        self.pid_balance = PID(pid_balance_P, pid_balance_I, pid_balance_D)
        self.pid_balance.setSampleTime(pid_balance_sample_time)
        self.pid_balance.setReference(pid_balance_reference)
        self.pid_balance_control_signal = 0.0  # control signal calculated by PID

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

    def initial_Estop_Check(self):
        self.ESTOP = self.bike.emergency_stop_check()
        if self.ESTOP:
            print 'WARNING: EMERGENCY STOP ACTIVE. TEST WILL ABORT IF NOT ACTIVATED NOW'
            input_estop = raw_input('Press ENTER to continue')
            if input_estop:
                self.stop()
                print 'Experiment Terminated'

    def log_headerline(self):
        # Data logging setup
        timestr = time.strftime("%Y%m%d-%H%M%S")
        RESULTS = open('../ExpData/BikeData-%s.csv' % timestr, 'wb')
        self.writer = csv.writer(RESULTS)
        if pid_velocity_active:
            self.writer.writerow(('P = %f' % pid_velocity_P, 'I = %f' % pid_velocity_I, 'D = %f' % pid_velocity_D))
        self.writer.writerow(
            ('R=' + str(Rparam), ' Q=' + str(Qparam), ' MassCenterhAndb=' + str(Box),
             'Description = ' + str(self.descr)))

        self.writer.writerow(
            ('Time', 'Calculation Time', 'Reference Velocity', 'Measured Velocity', 'Phi', 'Delta', 'Phi Dot',
             'Control Input', 'Velocity Control Input', 'phi_roll_comp', 'phi_uncomp', 'phi_dot',
             'a_x', 'ay_roll_comp', 'ay', 'az', 'phi_Gyro_intArd', 'x', 'y', 'psi', 'nu', 'phi_ref', 'delta_ref'))
        # ,'GPS_timestamp','GPS_x','GPS_y','latitude','longitude'))
        # ,'laserranger'))  # headers

    def log_regular(self):
        # Log data
        self.writer.writerow(
            (
                self.time_count, self.calculation_time, pid_velocity_reference, self.velocity, self.states[0],
                self.states[1],
                self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
                self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
                self.extra_data[4], self.extra_data[5], self.extra_data[6], self.extra_data[7], self.x, self.y,
                self.psi, self.nu,
                self.state_references[0], self.state_references[1]))
        # self.latest_gps_time,self.gpspos[0],self.gpspos[1],self.bike.gps.latitude,self.bike.gps.longitude))
        # ,self.y_laser_ranger))

        # Check the calculation time
        if self.calculation_time > sample_time:
            print "Warning: The calculation time exceeds the sampling time!"
            self.exceedscount += 1
            print "sensor_reading_time, control calculation, status_check = %g \t %g \t %g" % (
            self.sensor_reading_time, self.control_cal_time, self.status_check_time)
            # if self.exceedscount > 10:
            #     print "Error: Too long calculation time, experiment aborted"
            #     self.stop()

        # print out data

        print 'Time=%f\tCalc Time = %f\tVel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f\tx = %f \ty = %f\tpsi = %f\tnu = %f\t ' % (
            self.time_count, self.calculation_time, self.velocity, 57.29577 * self.states[0],
            57.29577 * self.extra_data[0], 57.29577 * self.states[1], self.x, self.y, self.psi, self.nu)

    def status_print(self):
        print 'Time=%f\tCalc Time = %f\tVel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f\tx = %f \ty = %f\tpsi = %f\tnu = %f\t ' % (
            self.time_count, self.calculation_time, self.velocity, 57.29577 * self.states[0],
            57.29577 * self.extra_data[0], 57.29577 * self.states[1], self.x, self.y, self.psi, self.nu)

    # @pysnooper.snoop()
    def gps_read(self):
        # Get GPS position
        # self.gpspos = self.bike.gps.get_position()
        x_measured_GPS = self.gpspos[0]
        y_measured_GPS = self.gpspos[1]
        self.latest_gps_time = self.bike.gps.lastread - self.gaining_speed_start  # The gps timestamp
        # Update Kalman filter
        # position_kalman = self.bike.kf.update(x_measured_GPS, y_measured_GPS, self.states[1], self.velocity)
        # print 'x_measured_GPS = %f ; y_measured_GPS = %f ; x_kalman = %f ; y_kalman = %f' % (x_measured_GPS, y_measured_GPS, position_kalman[0], position_kalman[1])

        print 'x_measured_GPS = %f ; y_measured_GPS = %f ;' % (x_measured_GPS, y_measured_GPS)

