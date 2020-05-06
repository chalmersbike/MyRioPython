INITIAL_SPEED = 2.5  # m/s
fixed_gains = 0 # 1 for velocity independent LQR gains, 0 for velocity dependent LQR gains
gains= [-20.8204,    5.0703,   -5.1136]  # for 2m/s 25hz
# gains= [-47.9208,    4.9023,   -9.1986]  # for 2m/s 25hz different too aggressive!

# P1 = [2.4486,  -25.4536,   91.9354, -125.0781]
# P2 = [0.1273,   -1.3272,    5.6469,   -2.2087]
# P3 = [0.5993,   -6.2280,   22.4694,  -30.5709]

P1 = [ 1.2183,  -12.7460,   47.7502,  -83.5453]
P2 = [0.0358,   -0.4217,    3.2164,   -0.2920]
P3 = [0.3092,   -3.2593,   12.3393,  -20.5164]
# gains= [-14.8850,    5.7641,   -3.6725]  # for 2.5m/s 25hz
# gains = [-11.4480, 6.3461, -2.8422]  # for 3m/s 25h# z
# gains= [-11.4480,    6.3461,   -2.8422]  # for 3.5m/s 25hz
# gains= [  -22.4049,    5.0941,   -5.3617] #manual input
TEST_TIME = 80.0  # seconds
speed_up_time = 0.5 # seconds
deadband = 0.0  # rad/s deadband for steering

import csv
import math
import time
import numpy
from math import pi as PI
from time import sleep

from constants import *
from utils import *

# Controller Parameters
CONTROLLER_FREQUENCY = 25 # 25  #50  # Hz

# Test Parameters
MAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad
MIN_HANDLEBAR_ANGLE = - MAX_HANDLEBAR_ANGLE  # rad
MAX_LEAN_ANGLE = 0.6  # rad (35 deg)
MIN_LEAN_ANGLE = - MAX_LEAN_ANGLE
comp_coef= 0.985 # 0.985 # coefficient of complementary filter
centripetal_compensation=1 # 1 to turn centripetal acceleration compensation on, 0 to turn it off
roll_compensation = 1 # 1 to turn roll acceleration compensation on, 0 to turn it off


# PID Velocity Controller Parameters
pid_velocity_active = False  # Boolean to control if PID controller is used
pid_velocity_reference = INITIAL_SPEED  # m/s
pid_velocity_P = 3.0
pid_velocity_I = 0.5
pid_velocity_D = 0.0
pid_velocity_sample_time = 1.0 / CONTROLLER_FREQUENCY

# Path Tracking
PATH_TYPE = 'NONE'  # [NONE, CIRCLE, STRAIGHT]
PATH_RADIUS = 20.0  # m

# PID Lateral Position Controller Parameters
pid_lateral_position_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
pid_lateral_position_P = 1.0
pid_lateral_position_I = 0.01
pid_lateral_position_D = 0.0
pid_lateral_position_sample_time = 1.0 / CONTROLLER_FREQUENCY

# PID Direction Controller Parameters
pid_direction_reference = 0.0  # PID code uses constant reference setpoint and feedback. Since this controller doesn't have a constant setpoint, we only use the feedback value.
pid_direction_P = 0.8
pid_direction_I = 0.4
pid_direction_D = 0.0
pid_direction_sample_time = 1.0 / CONTROLLER_FREQUENCY

# PID Steering Controller Parameters
pid_steering_reference = 0.0  # PID code uses constant reference setpoint and feedback. Since this controller doesn't have a constant setpoint, we only use the feedback value.
pid_steering_P = 0.8
pid_steering_I = 0.4
pid_steering_D = 0.0
pid_steering_sample_time = 1.0 / CONTROLLER_FREQUENCY

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('BikeData-%s.csv' % timestr, 'wb')


class Controller(object):

    def __init__(self, bike):

        self.bike = bike
        self.writer = csv.writer(RESULTS)

        # Bike
        self.time_count = 0.0
        self.states = [0.0, 0.0, 0.0]
        self.velocity = 0.0
        self.distance_travelled = 0.0
        self.ESTOP = False

        # Controller
        self.active = False
        self.TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
        self.calculation_time = 0.0
        self.complementary_coef=comp_coef
        self.CP_comp=centripetal_compensation
        self.roll_comp=roll_compensation

        # LQR Balance Controller
        self.state_references = []
        self.state_references = [state['default_reference'] for state in K_MATRIX]
        self.lqr_balance_input = 0.0

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

    def worker(self):
        while self.active:
            start_time = time.time()
            try:


                # Get states and calculate state_references
                self.velocity = self.bike.get_velocity()
                self.states_and_extra_data=self.get_states()
                self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
                self.extra_data = self.states_and_extra_data[3][:] # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]

                # Find Global Angles and Coordinates
                if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                    self.x, self.y, self.psi, self.nu = global_angles_and_coordinates(self.velocity, self.TIME_CONSTANT,
                                                                                  LENGTH_A, LENGTH_B, self.states[1],
                                                                                  self.psi, self.x, self.y)

                self.state_references = self.get_state_references(self.velocity)

                # PID Velocity Control
                if pid_velocity_active:
                    self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                    self.bike.set_velocity(self.pid_velocity_control_signal)

                if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
                    self.distance_travelled += self.velocity * self.TIME_CONSTANT

                # Balance Control
                self.keep_handlebar_angle_within_safety_margins(self.states[1])
                self.keep_the_bike_stable(self.velocity, self.state_references, self.states)




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

            # Calculation Time
            calculation_time = time.time() - start_time
            self.time_count += calculation_time
            time_0=time.time()

            # Log data
            self.writer.writerow(
                (self.time_count, calculation_time, pid_velocity_reference, self.velocity, self.states[0],
                 self.states[1],
                 self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
                 self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
                 self.extra_data[4], self.extra_data[5], self.extra_data[6]))
            print 'Time=%f\tCalc Time = %f\tVel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f' % (
                self.time_count, calculation_time, self.velocity, 57.29577*self.states[0],57.29577*self.extra_data[0],57.29577*self.states[1])

            # Control Frequency
            calculation_time = time.time() - start_time
            time_1=time.time()
            self.time_count += time_1 - time_0
            if calculation_time < self.TIME_CONSTANT:
                time.sleep(self.TIME_CONSTANT - calculation_time)
            time_2=time.time()
            self.time_count += time_2 - time_1


    def start(self):
        self.ESTOP = self.bike.emergency_stop_check()
        if self.ESTOP:
            print 'WARNING: EMERGENCY STOP ACTIVE. TEST WILL ABORT IF NOT ACTIVATED NOW'
            raw_input('Press ENTER to continue')

        raw_input('Press ENTER to start')

        if pid_velocity_active:
            self.writer.writerow(('P = %f' % pid_velocity_P, 'I = %f' % pid_velocity_I, 'D = %f' % pid_velocity_D))

        self.writer.writerow(
            ('Time', 'Calculation Time', 'Reference Velocity', 'Measured Velocity', 'Phi', 'Delta', 'Phi Dot',
             'Control Input', 'Velocity Control Input', 'phi_roll_comp', 'phi_uncomp', 'phi_dot',
             'a_x', 'ay_roll_comp', 'ay', 'az'))  # headers


        # Let the bike get to speed
        start_time = 0.0
        self.bike.set_velocity(INITIAL_SPEED)
        while start_time < speed_up_time:
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print 'EMERGENCY STOP'
                break
            sleep(self.TIME_CONSTANT)
            start_time += self.TIME_CONSTANT
            print 'GAINING SPEED'

        self.active = True
        self.worker()
        # self.thread = Thread(target=lambda: self.worker())
        # self.thread.start()

    def stop(self):
        self.active = False
        self.bike.stop_all()

    def get_states(self):

        delta_state = self.bike.get_handlebar_angle()
        imu_data = self.bike.get_imu_data()

        # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
        phi_d_state = imu_data[2]
        ay=imu_data[5-self.roll_comp]  # 4=roll compensated, 5=roll uncompensated
        az=imu_data[6]

        b = 1.095 # length between wheel centers [m]
        CP_acc_g=self.CP_comp*((self.velocity*self.velocity)/b)*math.tan(delta_state*0.94)*(1/9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
        phi_acc=math.atan2(ay+CP_acc_g*math.cos(self.states[0]),  az-CP_acc_g*math.sin(self.states[0]))
        # phi_acc=math.atan2(ay,  az)
        phi_state = self.complementary_coef * (self.states[0]+phi_d_state*self.TIME_CONSTANT) + (1 - self.complementary_coef) * phi_acc
        # phi_state = imu_data[0] # for only roll compensated phi
        # phi_state = imu_data[1] for uncompensated phi

        return [phi_state, delta_state, phi_d_state, imu_data]

    def get_state_references(self, velocity):

        if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
            lateral_error, angular_error, psi_ref = calculate__path_error(PATH_TYPE, self.distance_travelled, self.x,
                                                                          self.y, self.psi, PATH_RADIUS)

            # PID Control Lateral Position Block in Simulink
            self.pid_lateral_position_control_signal = self.pid_lateral_position.update(
                -lateral_error)  # Negative input due to PID code. See initiliazation for more info.

            # PID Control Direction Block in Simulink
            self.pid_direction_control_signal = self.pid_direction.update(
                -(-self.pid_lateral_position_control_signal + psi_ref - self.nu))  # Negative input due to PID code.

            # PID Control Steering Block in Simulink
            self.pid_steering_control_signal = self.pid_steering.update(
                -(self.pid_direction_control_signal - self.states[1]))
            delta_ref = self.pid_steering_control_signal
            phi_ref = math.atan(((numpy.power(velocity, 2) / (GRAVITY * LENGTH_B)) * delta_ref))
        else:
            delta_ref = 0.0
            phi_ref = 0.0

        return [phi_ref, delta_ref, 0.0]

    def get_feedback_gains(self, velocity):

        if velocity>4.0:
            velocity_saturated=4.0
        elif velocity<1.5:
            velocity_saturated=1.5
        else:
            velocity_saturated=velocity

        vel=velocity_saturated

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
            vel3 = vel*vel*vel
            vel2 = vel*vel
            gain1 = P1[0]*vel3+P1[1]*vel2+P1[2]*vel+P1[3]
            gain2 = P2[0]*vel3+P2[1]*vel2+P2[2]*vel+P2[3]
            gain3 = P3[0]*vel3+P3[1]*vel2+P3[2]*vel+P3[3]
            return [gain1, gain2, gain3]
        else:
            return gains


    def controller_set_handlebar_angular_velocity(self, angular_velocity):
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

    def keep_handlebar_angle_within_safety_margins(self, handlebar_angle):
        if handlebar_angle > MAX_HANDLEBAR_ANGLE:
            print 'Exceeded MAX_HANDLEBAR_ANGLE'
            self.upperSaturation = True
        elif handlebar_angle < MIN_HANDLEBAR_ANGLE:
            print 'Exceeded MIN_HANDLEBAR_ANGLE'
            self.lowerSaturation = True
        else:
            self.upperSaturation = False
            self.lowerSaturation = False

    def keep_the_bike_stable(self, velocity, references, states):

        feedback_gains = self.get_feedback_gains(velocity)

        self.lqr_balance_control_signal = sum([(r - x) * k for r, x, k in zip(
            references, states, feedback_gains
        )])

        if self.lqr_balance_control_signal > deadband or self.lqr_balance_control_signal < -deadband:
            self.controller_set_handlebar_angular_velocity(self.lqr_balance_control_signal)
        else:
            self.controller_set_handlebar_angular_velocity(0)