from param import *
import csv
import math
import time

from time import sleep

from constants import *
from utils import *
import numpy as np
import Adafruit_BBIO.ADC as ADC
from actuators import butter_lowpass,butter_lowpass_filter,moving_average_filter
#import pysnooper

from scipy import signal
import numpy as np


# @pysnooper.snoop('./log/file.log')
class Controller(object):

    def __init__(self, bike):

        self.bike = bike
        self.variable_init()

    def startup(self):

        self.initial_Estop_Check() # Check if the Estop Engaged
        self.descr = raw_input('type the description if necessary, Press ENTER to start,')
        print "start in %i secs" % start_up_interval
        time.sleep(start_up_interval)
        self.states[0] =self.bike.get_imu_data()[0] # Read the IMU complementary filter Phi as the initial phi estimation


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

            self.time_get_states = time.time()
            self.states_and_extra_data = self.get_states()
            self.time_get_states = time.time() - self.time_get_states

            self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
            self.extra_data = self.states_and_extra_data[3][:]  # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
            self.sensor_reading_time = time.time() - start_time_current_loop
            self.state_calculate()
            # Get y position on the roller
            # print "reading Laser at %d" %(time.time() - self.gaining_speed_start)
            if (time.time() - self.time_laserranger) > 1.1 * self.bike.laser_ranger.timing:
                self.y_laser_ranger = self.bike.laser_ranger.get_y()
                self.time_laserranger = time.time()
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
            self.calculation_time = time.time() - start_time_current_loop # The time elapsed
            self.log_regular()  #self.time_count, self.calculation_time
            self.calculation_time = time.time() - start_time_current_loop  # The time elapsed
            if self.calculation_time < self.TIME_CONSTANT:
                time.sleep((self.TIME_CONSTANT - self.calculation_time))


            # Log data

        print '\nGaining speed phase over\n'

        self.states[0] = self.bike.get_imu_data()[0]  # Reset State as Arduino Roll data again
        self.controller_active = True
        self.bike.steering_motor.enable()
        # self.thread = Thread(target=lambda: self.worker())
        # self.thread.start()

        self.pid_balance.clear()
        self.pid_steeringangle.clear()

    # @pysnooper.snoop()
    def run(self):
        self.pid_steeringangle.clear()
        while self.controller_active:
            start_time_current_loop = time.time()

            try:


                # Get states and calculate state_references
                self.velocity = self.bike.get_velocity()
                self.states_and_extra_data=self.get_states()
                self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
                self.extra_data = self.states_and_extra_data[3][:] # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
                self.sensor_reading_time = time.time() - start_time_current_loop
                self.state_calculate()
                # self.gps_read()
                # Find Global Angles and Coordinates
                if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                    self.x, self.y, self.psi, self.nu = global_angles_and_coordinates(self.velocity, self.TIME_CONSTANT,
                                                                                  LENGTH_A, LENGTH_B, self.states[1],
                                                                                  self.psi, self.x, self.y)
                self.state_references = self.get_state_references(self.velocity)

                # print "reading Laser at %f" % (time.time() - self.gaining_speed_start)
                # Get y position on the roller
                if (time.time()-self.time_laserranger) > 1.1*self.bike.laser_ranger.timing:
                    self.y_laser_ranger = self.bike.laser_ranger.get_y()
                    self.time_laserranger = time.time()

                # PID Velocity Control
                if pid_velocity_active:
                    self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                    self.bike.set_velocity(self.pid_velocity_control_signal)

                if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                    self.distance_travelled += self.velocity * self.TIME_CONSTANT
                self.sensor_reading_time = time.time() - start_time_current_loop

                # Balance Control
                self.keep_handlebar_angle_within_safety_margins(self.states[1])

                # self.keep_the_bike_stable_2states(self.velocity, [self.x1ref, self.x2ref], [self.x1, self.x2])
                self.keep_the_bike_stable_2states(self.velocity, self.state_references, self.states)

                # self.keep_the_bike_stable(self.velocity, self.state_references, self.states)

                # self.keep_the_bike_stable_predictor_control(self.velocity, [self.x1ref, self.x2ref], [self.x1, self.x2])
                # self.keep_the_bike_stable_predictor_control()


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
            self.time_count = time.time() - self.gaining_speed_start
            # Log data
            self.log_regular() #self.time_count, self.calculation_time

            self.calculation_time = time.time() - start_time_current_loop
            # print 'Time spent', self.calculation_time
            if self.calculation_time < self.TIME_CONSTANT:
                time.sleep((self.TIME_CONSTANT - self.calculation_time))


    def stop(self):
        self.controller_active = False
        self.bike.stop_all()

    def get_states(self):
        self.potential = ((self.bike.potent.read_pot_value() / 0.29) * 5 - 2.5) * deg2rad * 2
        delta_state = self.bike.get_handlebar_angle()
        # try:
        #     if abs(delta_state - self.delta_state_previous) > abs(2*sample_time*self.lqr_balance_control_signal) and abs(delta_state - self.delta_state_previous) > 0.01:
        #         print 'Time = ' + str(self.time_count) + ' ; delta_state = ' + str(delta_state) + ' ; delta_state_previous = ' + str(self.delta_state_previous)
        #         delta_state = self.delta_state_previous + sample_time*self.lqr_balance_control_signal
        # except:
        #     pass
        # self.delta_state_previous = delta_state

        ##self.delta_state_list.append(delta_state)
        #self.delta_state_list = np.append(self.delta_state_list,delta_state)
        ##self.delta_state_list = butter_lowpass_filter(self.delta_state_list, cutoff, fs, order)
        #self.delta_state_list = moving_average_filter(self.delta_state_list, windowSize)
        #delta_state_filt = self.delta_state_list[-1]
        #print 'delta_state = ' + str(delta_state) +  ' ; delta_state_filt = ' + str(delta_state_filt)
        ##delta_state = delta_state_filt

        imu_data = self.bike.get_imu_data()

        # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
        phi_d_state = imu_data[2]
        ay=imu_data[5-self.roll_comp]  # 4=roll compensated, 5=roll uncompensated
        az=imu_data[6]

        b = 1.095 # length between wheel centers [m]
        CP_acc_g=self.CP_comp*((self.velocity*self.velocity)/b)*math.tan(delta_state*0.94)*(1/9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
        phi_acc=math.atan2(ay - CP_acc_g*math.cos(self.states[0]),  az + CP_acc_g*math.sin(self.states[0])) # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        ## phi_acc=math.atan2(ay,  az)
        #phi_state = self.complementary_coef * (self.states[0]+phi_d_state*self.TIME_CONSTANT) + (1 - self.complementary_coef) * phi_acc
        ## phi_state = self.complementary_coef * (imu_data[0]) + (
        ##             1 - self.complementary_coef) * phi_acc
        ## phi_state = (imu_data[0])
        ## phi_state = imu_data[0] # for only roll compensated phi
        phi_state = imu_data[1] # for uncompensated phi
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
            #self.pid_lateral_position_control_signal = self.pid_lateral_position.update(
            #    lateral_error)  # Negative input due to PID code. See initiliazation for more info.

            # PID Control Direction Block in Simulink
            self.pid_direction_control_signal = self.pid_direction.update(
                -(-self.pid_lateral_position_control_signal + psi_ref - self.nu))  # Negative input due to PID code.
            # self.pid_direction_control_signal = self.pid_direction.update(
            #    (-self.pid_lateral_position_control_signal + psi_ref - self.nu))  # Negative input due to PID code.

            # PID Control Steering Block in Simulink
            self.pid_steering_control_signal = self.pid_steering.update(
                -(self.pid_direction_control_signal - self.states[1]))
            #self.pid_steering_control_signal = self.pid_steering.update(
            #    (self.pid_direction_control_signal - self.states[1]))

            delta_ref = self.pid_steering_control_signal
            phi_ref = -math.atan(((np.power(velocity, 2) / (GRAVITY * LENGTH_B)) * delta_ref))

            # print 'lateral_error = ' + str(lateral_error) + ' ; angular_error = ' + str(angular_error) + ' ; psi_ref = ' + str(delta_ref) + ' ; psi_ref = ' + str(delta_ref) + ' ; pid_lateral_position_control_signal = ' + str(self.pid_lateral_position_control_signal) + str(psi_ref) + ' ; pid_direction_control_signal = ' + str(self.pid_direction_control_signal) + ' ; pid_steering_control_signal = ' + str(self.pid_steering_control_signal )
        else:
            # delta_ref = ((self.bike.potent.read_pot_value()/0.29)*5 - 2.5) * deg2rad *2
            delta_ref = 0.0
            phi_ref = 0.0
        # v = self.velocity
        # phi = phi_ref
        # delta = delta_ref
        # Phi_dot = 0.0
        # if v < 2:
        #     v = 2
        # self.x1ref = (num11 * v ** 2 * delta + num12 * phi + num13 * v * Phi_dot) / (den11 * v + den12 * v ** 3)
        # self.x2ref = (num21 * v * delta + num22 * v * phi - num23 * Phi_dot) / (den21 * v + den22 * v ** 3)
        self.delta_ref = delta_ref
        return [phi_ref, delta_ref, 0.0]

    # def get_feedback_gains(self, velocity):
    #
    #     if velocity>4.0:
    #         velocity_saturated=4.0
    #     elif velocity<1.5:
    #         velocity_saturated=1.5
    #     else:
    #         velocity_saturated=velocity
    #
    #     vel=velocity_saturated
    #
    #     if fixed_gains == 0:
    #         vel3 = vel*vel*vel
    #         vel2 = vel*vel
    #         gain1 = P1[0]*vel3+P1[1]*vel2+P1[2]*vel+P1[3]
    #         gain2 = P2[0]*vel3+P2[1]*vel2+P2[2]*vel+P2[3]
    #         gain3 = P3[0]*vel3+P3[1]*vel2+P3[2]*vel+P3[3]
    #         return [gain1, gain2, gain3]
    #         # return [gain1/2, gain2/2, gain3/2]
    #         # return [gain1*2, gain2*2, gain3*2]
    #         #return [gain1*1, gain2*2, gain3*0.5]
    #     else:
    #         return gains

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
            print 'Exceeded MAX_HANDLEBAR_ANGLE of %f deg' %(max_handlebar_angle * 57.3)
            self.upperSaturation = True
            self.bike.stop_all()
        elif handlebar_angle < min_handlebar_angle:
            print 'Exceeded MIN_HANDLEBAR_ANGLE of %f deg' %(max_handlebar_angle * 57.3)
            self.lowerSaturation = True
            self.bike.stop_all()
        else:
            self.upperSaturation = False
            self.lowerSaturation = False

    def keep_the_bike_stable(self, velocity, references, states):

        # feedback_gains = self.get_feedback_gains(velocity)
        #
        # self.lqr_balance_control_signal = sum([(r - x) * k for r, x, k in zip(
        #     references, states, feedback_gains
        # )])

        #print 'velocity = ' + str(velocity) + ' ; references = ' + str(references) + ' ; states = ' + str(states) + ' ; feedback_gains = ' + str(feedback_gains) + ' ; lqr_balance_control_signal = ' + str(self.lqr_balance_control_signal)
        ##print 'velocity = %.3f ; references = %.3f ; states = %.3f ; feedback_gains = %.3f ; lqr_balance_control_signal = %.3f' % (velocity,57.29577*references,57.29577*states,feedback_gains,57.29577*self.lqr_balance_control_signal)
        ##print 'velocity = %.3f ; references = %.3f ; states = %.3f' % (velocity,references,57.29577*states)

        self.pid_balance_control_signal = self.pid_balance.update(states[0]) # Delta
        # print 'phi(deg) = ' + str(57.29577*states[0]) + ' ; pid_signal(deg) = ' + str(57.29577*self.pid_balance_control_signal)
        self.lqr_balance_control_signal = self.pos2vel(self.pid_balance_control_signal) # Delta_dot
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
        self.x1 = 0.0
        self.x2 = 0.0
        self.x1ref = 0.0
        self.x2ref = 0.0
        self.AngRef = 0.0
        self.AngRefOld = 0.0
        self.AngVel = 0.0
        self.AngVel_prec = 0.0
        self.AngVel_filt = 0.0
        self.AngVel_filt_prec = 0.0
        self.ref_diff_old = np.zeros((2,1))
        self.steering_rate = 0.0
        self.steering_rate_previous = 0.0
        self.steering_rate_filt = 0.0
        self.steering_rate_filt_previous = 0.0

        # Controller
        self.controller_active = False
        self.TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
        self.calculation_time = 0.0
        self.complementary_coef=comp_coef
        self.CP_comp=centripetal_compensation
        self.roll_comp=roll_compensation

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

        # PID Balance Controller Outer Loop
        self.pid_balance_outerloop = PID(pid_balance_outerloop_P, pid_balance_outerloop_I, pid_balance_outerloop_D)
        self.pid_balance_outerloop.setSampleTime(pid_balance_outerloop_sample_time)
        self.pid_balance_outerloop.setReference(pid_balance_outerloop_reference)
        self.pid_balance__outerloop_control_signal = 0.0  # control signal calculated by PID

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

        #PID Steering ANGLE Controller
        self.pid_steeringangle = PID(pid_steeringangle_P, pid_steeringangle_I, pid_steeringangle_D)
        self.pid_steeringangle.setSampleTime(pid_steeringangle_sample_time)
        self.pid_steeringangle.setReference(0.0)
        self.pid_steeringangle_control_signal = 0.0  # This signal is called "Delta Reference" in Simulink.
        # Global Angles and Coordinates
        self.psi = 0.0
        self.nu = 0.0
        if PATH_TYPE == 'CIRCLE':
            self.x = PATH_RADIUS
        else:
            self.x = 0.0
        self.y = 0.0

        #self.delta_state_previous = 0
        self.delta_state_list = list()

        self.descr = 'NoComments'
        self.smith_x = np.zeros([2, smith_delay])
        self.smith_delta = np.zeros([1, smith_delay * 2])
        self.delta_k = 0

        # Laser Ranger
        self.time_laserranger = 0


    def initial_Estop_Check(self):
        self.ESTOP = self.bike.emergency_stop_check()
        if self.ESTOP:
            print 'WARNING: EMERGENCY STOP ACTIVE. TEST WILL ABORT IF NOT ACTIVATED NOW'
            input_estop = raw_input('Press ENTER to continue')
            if input_estop:
                self.stop()
                print 'Experiment Terminated'

    # def log_headerline(self):
    #     # Data logging setup
    #     timestr = time.strftime("%Y%m%d-%H%M%S")
    #     RESULTS = open('./ExpData/BikeData-%s.csv' % timestr, 'wb')
    #     self.writer = csv.writer(RESULTS)
    #     if pid_velocity_active:
    #         self.writer.writerow(('P = %f' % pid_velocity_P, 'I = %f' % pid_velocity_I, 'D = %f' % pid_velocity_D))
    #     self.writer.writerow(
    #         ('R=' + str(Rparam),' Q=' + str(Qparam),' MassCenterhAndb='+ str(Box), 'Description = ' + str(self.descr)))
    #
    #     self.writer.writerow(
    #         ('Time', 'Calculation Time', 'Reference Velocity', 'Measured Velocity', 'Phi', 'Delta', 'Phi Dot',
    #          'Control Input', 'Velocity Control Input', 'phi_roll_comp', 'phi_uncomp', 'phi_dot',
    #          'a_x', 'ay_roll_comp', 'ay', 'az', 'phi_Gyro_intArd','x','y','psi','nu','phi_ref','delta_ref','delta_ctrl_ref','x1_ref','x2_ref','x1','x2','delta_k','delta_stack','x_stack'))
    #          # ,'GPS_timestamp','GPS_x','GPS_y','latitude','longitude'))
    #          #,'laserranger'))  # headers

    # def log_regular(self):
    #     # Log data
    #     self.time_log = time.time()
    #     self.writer.writerow(
    #         (
    #         self.time_count, self.calculation_time, pid_velocity_reference, self.velocity, self.states[0],
    #         self.states[1],
    #         self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
    #         self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
    #         self.extra_data[4], self.extra_data[5], self.extra_data[6], self.extra_data[7],self.x,self.y,self.psi,self.nu,
    #         self.state_references[0],self.state_references[1], self.AngRef, self.x1ref, self.x2ref, self.x1, self.x2, self.delta_k, self.smith_delta, self.smith_x))
    #         # self.latest_gps_time,self.gpspos[0],self.gpspos[1],self.bike.gps.latitude,self.bike.gps.longitude))
    #         #,self.y_laser_ranger))
    #
    #     self.time_log = time.time() - self.time_log
    #     print "sensor_reading_time, control calculation, status_check   IMU   log  = %g \t %g \t %g \t %g \t %g" % (
    #     self.sensor_reading_time, self.control_cal_time, self.status_check_time,self.time_get_states,self.time_log)
    #
    #     # Check the calculation time
    #     if self.calculation_time > sample_time:
    #         print "Warning: The calculation time exceeds the sampling time!"
    #         self.exceedscount += 1
    #         print "sensor_reading_time, control calculation, status_check = %g \t %g \t %g" % (self.sensor_reading_time, self.control_cal_time, self.status_check_time)
    #         # if self.exceedscount > 10:
    #         #     print "Error: Too long calculation time, experiment aborted"
    #         #     self.stop()
    #
    #     # print out data
    #
    #     print 'Time=%f\tCalc Time = %f\tVel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f\tx = %f \ty = %f\tpsi = %f\tnu = %f\t ' % (
    #         self.time_count, self.calculation_time, self.velocity, 57.29577 * self.states[0],
    #         57.29577 * self.extra_data[0], 57.29577 * self.states[1],self.x,self.y,self.psi,self.nu)


    def log_headerline(self):
        # Data logging setup
        timestr = time.strftime("%Y%m%d-%H%M%S")
        RESULTS = open('./ExpData/BikeData-%s.csv' % timestr, 'wb')
        self.writer = csv.writer(RESULTS)
        if pid_velocity_active:
            self.writer.writerow(('P = %f' % pid_velocity_P, 'I = %f' % pid_velocity_I, 'D = %f' % pid_velocity_D))
        self.writer.writerow(
            ('R=' + str(Rparam),' Q=' + str(Qparam),' MassCenterhAndb='+ str(Box), 'Description = ' + str(self.descr)))

        self.writer.writerow(
            ('Time', 'Calculation Time',  'Measured Velocity', 'Phi', 'Delta', 'Phi Dot',
             'Control Input',
             'a_x',  'ay', 'az', 'x','y','psi','nu','phi_ref','delta_ref','delta_ctrl_ref','x1_ref','x2_ref','x1','x2','delta_k','potential'
             # ,'GPS_timestamp','GPS_x','GPS_y','latitude','longitude'
            ,'laserranger'  # headers
        ))

    def log_regular(self):
        # Log data
        self.time_log = time.time()
        self.writer.writerow(
            (
                "{0:.5f}".format(self.time_count),
                "{0:.5f}".format(self.calculation_time),
                "{0:.5f}".format(self.velocity),
                "{0:.5f}".format(self.states[0]),
                "{0:.5f}".format(self.states[1]),
                "{0:.5f}".format(self.states[2]),
                "{0:.5f}".format(self.pid_balance_control_signal),
                "{0:.5f}".format(self.extra_data[3]),
                "{0:.5f}".format(self.extra_data[5]),
                "{0:.5f}".format(self.extra_data[6]),
                "{0:.5f}".format(self.x),
                "{0:.5f}".format(self.y),
                "{0:.5f}".format(self.psi),
                "{0:.5f}".format(self.nu),
                "{0:.5f}".format(self.state_references[0]),
                "{0:.5f}".format(self.state_references[1]),
                "{0:.5f}".format(self.AngRef),
                "{0:.5f}".format(self.x1ref),
                "{0:.5f}".format(self.x2ref),
                "{0:.5f}".format(self.x1),
                "{0:.5f}".format(self.x2),
                "{0:.5f}".format(self.delta_k),
                "{0:.5f}".format(self.potential)
            # self.latest_gps_time,self.gpspos[0],self.gpspos[1],self.bike.gps.latitude,self.bike.gps.longitude))
            ,self.y_laser_ranger
        ))

        self.time_log = time.time() - self.time_log
        #print "sensor_reading_time, control calculation, status_check   IMU   log  = %g \t %g \t %g \t %g \t %g \t" % (
        #self.sensor_reading_time, self.control_cal_time, self.status_check_time,self.time_get_states,self.time_log)

        # Check the calculation time
        if self.calculation_time > sample_time:
            #print "Warning: The calculation time exceeds the sampling time!"
            self.exceedscount += 1
            #print "sensor_reading_time, control calculation, status_check = %g \t %g \t %g" % (self.sensor_reading_time, self.control_cal_time, self.status_check_time)
            # if self.exceedscount > 10:
            #     print "Error: Too long calculation time, experiment aborted"
            #     self.stop()

        # print out data
        #print 'Time = %f\t Calculation time = %f' %(self.time_count, self.calculation_time)
        #print 'Time=%f\tCalc Time = %f\t Potentio = %g \t Vel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f\tx = %f \ty = %f\tpsi = %f\tnu = %f\t ' % (
            #self.time_count, self.calculation_time, self.potential, self.velocity, 57.29577 * self.states[0],
            #57.29577 * self.extra_data[0], 57.29577 * self.states[1],self.x,self.y,self.psi,self.nu)

    #def status_print(self):
        #print 'Time=%f\tCalc Time = %f\tVel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f\tx = %f \ty = %f\tpsi = %f\tnu = %f\t ' % (
            #self.time_count, self.calculation_time, self.velocity, 57.29577 * self.states[0],
            #57.29577 * self.extra_data[0], 57.29577 * self.states[1],self.x,self.y,self.psi,self.nu)

    # @pysnooper.snoop()
    def gps_read(self):
        # Get GPS position
        # self.gpspos = self.bike.gps.get_position()
        x_measured_GPS = self.gpspos[0]
        y_measured_GPS = self.gpspos[1]
        self.latest_gps_time = self.bike.gps.lastread - self.gaining_speed_start # The gps timestamp
        # Update Kalman filter
        # position_kalman = self.bike.kf.update(x_measured_GPS, y_measured_GPS, self.states[1], self.velocity)
        # print 'x_measured_GPS = %f ; y_measured_GPS = %f ; x_kalman = %f ; y_kalman = %f' % (x_measured_GPS, y_measured_GPS, position_kalman[0], position_kalman[1])

        #print 'x_measured_GPS = %f ; y_measured_GPS = %f ;' % (x_measured_GPS, y_measured_GPS)

    def state_calculate(self):
        v = self.velocity
        phi = self.states[0]
        delta = self.states[1]
        Phi_dot = self.states[2]
        if v > 2:
            self.x1 = (num11 * v ** 2 * delta + num12 * phi + num13 * v * Phi_dot) / (den11 * v + den12 * v ** 3)
            self.x2 = (num21 * v * delta + num22 * v * phi + num23 * Phi_dot) / (den21 * v + den22 * v ** 3)
        else:
            v = 2
            self.x1 = (num11 * v ** 2 * delta + num12 * phi + num13 * v * Phi_dot) / (den11 * v + den12 * v ** 3)
            self.x2 = (num21 * v * delta + num22 * v * phi + num23 * Phi_dot) / (den21 * v + den22 * v ** 3)
        self.xk = np.array([[self.x1], [self.x2]])

        C_bike = C_bike_V0 + C_bike_V1 * v + C_bike_V2 * v**2
        O_bike = np.vstack((C_bike,C_bike*A_bike))
        self.xk = np.linalg.inv(O_bike)*(np.array([[phi],[Phi_dot]]) - np.vstack((D_bike,D_bike+C_bike.dot(B_bike)))*delta)

    def get_2state_feedback_gains(self, velocity):

        if velocity>4.0:
            velocity_saturated=4.0
        elif velocity<1.5:
            velocity_saturated=1.5
        else:
            velocity_saturated=velocity

        vel=velocity_saturated

        if fixed_gains == 0:
            vel3 = vel*vel*vel
            vel2 = vel*vel
            gain1 = P_2state1[0]*vel3+P_2state1[1]*vel2+P_2state1[2]*vel+P_2state1[3]
            gain2 = P_2state2[0]*vel3+P_2state2[1]*vel2+P_2state2[2]*vel+P_2state2[3]
            return [gain1, gain2]
            # return [gain1/2, gain2/2, gain3/2]
            # return [gain1*2, gain2*2, gain3*2]
            #return [gain1*1, gain2*2, gain3*0.5]
        else:
            return gains

    def keep_the_bike_stable_predictor_control(self):

        # self.state_memory_shift()
        self.delta_k = self.delta_ref - LAd.dot(self.xk) + L.dot(self.xk) - lw.dot(
            self.smith_delta[0, 0:smith_delay] + self.smith_delta[0, smith_delay:2 * smith_delay]) - LAd.dot(
            self.smith_x[0:2, 0])
        self.delta_k = self.delta_k.item()
        self.lqr_balance_control_signal = self.pos2vel(self.delta_k)
        self.state_memory_shift()

        if self.lqr_balance_control_signal > deadband or self.lqr_balance_control_signal < -deadband:
            self.controller_set_handlebar_angular_velocity(self.lqr_balance_control_signal)
        else:
            self.controller_set_handlebar_angular_velocity(0)

    def keep_the_bike_stable_2states(self, velocity, references, states):
        # Chalmers Controller structure : deltadot = PID(phidot)
        # self.pid_balance_control_signal = self.pid_balance.update(states[2])
        # self.steering_rate = self.pid_balance_control_signal

        # Chalmers Controller Structure 2 : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
        # # self.potential = -((self.bike.potent.read_pot_value() / 0.29) * 5 - 2.5) * deg2rad * 2 # Potentiometer gives a position reference between -5deg and 5deg
        # self.potential = -((self.bike.potent.read_pot_value() / 0.29) * 2.5 - 1.25) * deg2rad * 2 # Potentiometer gives a position reference between -2.5deg and 2.5deg
        # self.pid_balance_outerloop.setReference(self.potential)
        # self.pid_balance.setReference(self.pid_balance_outerloop.update(states[0]))
        # self.pid_balance_control_signal = self.pid_balance.update(states[2])
        # self.steering_rate = self.pid_balance_control_signal

        # PATH TRACKING Chalmers Controller Structure 2 : yref = potentiometer ; phiref = PID (y) ; phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
        self.potential = ((self.bike.potent.read_pot_value() / 0.29) * 0.2 - 0.1) # Potentiometer gives a position reference between -0.1m and 0.1m
        self.pid_lateral_position.setReference(self.potential)
        self.pid_balance_outerloop.setReference(self.pid_lateral_position.update(self.y_laser_ranger))
        self.pid_balance.setReference(self.pid_balance_outerloop.update(states[0]))
        self.pid_balance_control_signal = self.pid_balance.update(states[2])
        self.steering_rate = self.pid_balance_control_signal



        # MDH Controller structure : delta = PID(phi) ; deltadot = PID(delta)
        # self.pid_balance_control_signal = self.pid_balance.update(states[0])
        # self.steering_rate = self.pos2vel(self.pid_balance_control_signal)

        # Low-pass filter
        # Butterworth 1st order 1Hz cutoff
        self.steering_rate_filt = 0.0305 * self.steering_rate + 0.0305 * self.steering_rate_previous + 0.9391 * self.steering_rate_filt_previous

        self.steering_rate_filt_previous = self.steering_rate_filt
        self.steering_rate_previous = self.steering_rate
        # self.steering_rate = self.steering_rate_filt

        # Send Steering Rate Reference value to steering motor controller
        if self.pid_balance_control_signal > deadband or self.pid_balance_control_signal < -deadband:
            self.controller_set_handlebar_angular_velocity(self.steering_rate)
        else:
            self.controller_set_handlebar_angular_velocity(0)


    def pos2vel(self, AngRef):
        self.pid_steeringangle.setReference(AngRef)
        # self.AngVel = self.pid_steeringangle.update(-self.states[1])
        self.AngVel = self.pid_steeringangle.update(self.states[1])
        return self.AngVel

        # ref_diff = AngRef - self.AngRefOld
        # self.ref_diff_old = np.append(self.ref_diff_old[1:-1], ref_diff)
        # ref_diff = np.mean(self.ref_diff_old)
        # self.pid_steeringangle.setReference(AngRef)
        # self.AngRefOld = AngRef
        # self.AngVel = self.pid_steeringangle.update(self.states[1]) + D_param * ref_diff / pid_steeringangle_sample_time
        #
        # # Low-pass filter
        # # 1s order low-pass Butterworth filter, 3Hz cutoff
        # self.AngVel_filt = 0.0592* self.AngVel + 0.0592* self.AngVel_prec + 0.8816 * self.AngVel_filt_prec
        # # self.AngVel_filt = 0.0864 * self.AngVel + 0.0864 * self.AngVel_prec + 0.8273 * self.AngVel_filt_prec
        # # self.AngVel_filt = 0.2452 * self.AngVel + 0.2452 * self.AngVel_prec + 0.5095 * self.AngVel_filt_prec
        # # self.AngVel_filt = 0.1367* self.AngVel + 0.1367 * self.AngVel_prec + 0.7265 * self.AngVel_filt_prec
        # # self.AngVel_filt = 0.2452 * self.AngVel + 0.2452 * self.AngVel_prec + 0.5095 * self.AngVel_filt_prec
        #
        # self.AngVel_prec = self.AngVel
        # self.AngVel_filt_prec = self.AngVel_filt
        #
        # # self.AngVel = self.AngVel_filt
        # return self.AngVel

    # def smithFilter_init(self):
    #
    #     num = [18.91]
    #     den = [1, 17.9, 0]
    #     dis_tf = signal.cont2discrete([num, den], self.TIME_CONSTANT)
    #     dis_tf = signal.TransferFunction(dis_tf1[0], dis_tf1[1], dt=dis_tf1[2])
    #     self.dis_ss = dis_tf.to_ss()
    #
    #     smith_delay = 3
    #     self.smith_x_list = np.zeros((2, smith_delay))
    #
    #     xnew = (np.dot(dis_ss.A, np.asarray([[0.0], [0.0]])) + dis_ss.B * np.asarray([0.0]))
    #     ynew = np.dot(dis_ss.C, xnew)
    #     xnew1 = (np.dot(dis_ss.A, xnew) + np.dot(dis_ss.B, 0.5))
    #     print 'xnew = ', xnew
    #     ynew1 = np.dot(dis_ss.C, xnew1)
    #     print ynew, ynew1
    #
    #     xnew2 = (np.dot(dis_ss.A, xnew1) + dis_ss.B * np.asarray([1.0]))
    #     ynew2 = np.dot(dis_ss.C, xnew2)
    #
    #     print ynew2

    def state_memory_shift(self):
        self.smith_x[:, 0:smith_delay-1] = self.smith_x[:, 1:smith_delay ]
        self.smith_x[:, smith_delay-1:smith_delay] = self.xk
        self.smith_delta[0, 0: 2 * smith_delay-1] = self.smith_delta[0, 1: 2 * smith_delay]
        self.smith_delta[0, 2 * smith_delay-1:2 * smith_delay] = self.delta_k

