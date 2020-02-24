INITIAL_SPEED = 3.9  # m/s

#################################################################################################
## TEST
INITIAL_SPEED = 0  # m/s
#################################################################################################
fixed_gains = 0 # 1 for velocity independent LQR gains, 0 for velocity dependent LQR gains
gains= [-20.8204,    5.0703,   -5.1136]  # for 2m/s 25hz
# gains= [-47.9208,    4.9023,   -9.1986]  # for 2m/s 25hz different too aggressive!

# P1 = [2.4486,  -25.4536,   91.9354, -125.0781]
# P2 = [0.1273,   -1.3272,    5.6469,   -2.2087]
# P3 = [0.5993,   -6.2280,   22.4694,  -30.5709]
#Original Parameters From Umur
# P1 = [ 1.2183,  -12.7460,   47.7502,  -83.5453]
# P2 = [0.0358,   -0.4217,    3.2164,   -0.2920]
# P3 = [0.3092,   -3.2593,   12.3393,  -20.5164]

# # New Parameters R 42 Q diag 100 100 10
# P1 = [-1.9517,  20.3042,   -73.5315,  100.1292]
# P2 = [0.1005,   -1.0573,    4.1598,   -1.0215]
# P3 = [-0.4772,   4.9647,   -17.9717,  24.4728]

# New Parameters R 2000 Q diag 2000 10 100
P1 = [-1.4395,  14.9692,   -54.4004,  75.2457]
P2 = [0.0651,   -0.7031,    3.2496,   0.2177]
P3 = [-0.3518,   3.6584,   -13.2961,  18.3907]

# gains= [-14.8850,    5.7641,   -3.6725]  # for 2.5m/s 25hz
# gains = [-11.4480, 6.3461, -2.8422]  # for 3m/s 25h# z
# gains= [-11.4480,    6.3461,   -2.8422]  # for 3.5m/s 25hz
# gains= [  -22.4049,    5.0941,   -5.3617] #manual input
TEST_TIME = 25.0  # seconds
speed_up_time = 3.0 # seconds
deadband = 0.0/57.29577  # rad/s deadband for steering

CURRENT_SENSE_PORT = 'P9_38'
RPM_SENSE_PORT = 'P9_40'
# Controller Parameters
CONTROLLER_FREQUENCY = 25 # 25  #50  # Hz
sample_time = 1.0/CONTROLLER_FREQUENCY

import csv
import math
import time
import numpy
from math import pi as PI
from time import sleep

from constants import *
from utils import *
import Adafruit_BBIO.ADC as ADC
import kalman_filter #as kf

from actuators import butter_lowpass,butter_lowpass_filter,moving_average_filter


# Test Parameters
# MAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad, 50 deg
LowSpeedMAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad, 50 deg
HighSpeedMAX_HANDLEBAR_ANGLE = ((1.0/ 3.0) * PI) / 6.0   # rad 10 deg
HIGHSPEEDBOUND = 3  # m/s
MAX_HANDLEBAR_ANGLE = LowSpeedMAX_HANDLEBAR_ANGLE
MIN_HANDLEBAR_ANGLE = - MAX_HANDLEBAR_ANGLE
MAX_HANDLEBAR_ANGLE_ADJUST = 0  # Switch on/off the handle bar MAX angle range dynamic adjustment
HANDLEBAR_CORRECTION_ANGVEL = 0  # When the handlebar is out of tolerance range, drag it back to the region

MAX_LEAN_ANGLE = 0.6  # rad (35 deg)
MIN_LEAN_ANGLE = - MAX_LEAN_ANGLE
comp_coef= 0.985 # 0.985 # coefficient of complementary filter
centripetal_compensation=1 # 1 to turn centripetal acceleration compensation on, 0 to turn it off
roll_compensation = 1 # 1 to turn roll acceleration compensation on, 0 to turn it off


# PID Velocity Controller Parameters
pid_velocity_active = False  # Boolean to control if PID controller is used
pid_velocity_reference = INITIAL_SPEED  # m/s
pid_velocity_P = 3.0
pid_velocity_I = 0.01
pid_velocity_D = 0.0
pid_velocity_sample_time = 1.0 / CONTROLLER_FREQUENCY

# Path Tracking
PATH_TYPE = 'NONE'  # [NONE, CIRCLE, STRAIGHT]
#PATH_TYPE = 'STRAIGHT'  # [NONE, CIRCLE, STRAIGHT]
PATH_RADIUS = 20.0  # m

# PID Lateral Position Controller Parameters
pid_lateral_position_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
pid_lateral_position_P = 0.1
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
pid_steering_P = 1.0
pid_steering_I = 0.1
pid_steering_D = 0.0
pid_steering_sample_time = 1.0 / CONTROLLER_FREQUENCY

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('BikeData-%s.csv' % timestr, 'wb')

# Filter requirements.
order = 2
fs = CONTROLLER_FREQUENCY       # sample rate, Hz
cutoff = 8# desired cutoff frequency of the filter, Hz
windowSize = 10

# Get the filter coefficients so we can check its frequency response.
b, a = butter_lowpass(cutoff, fs, order)
# !!! TEST !!!
# PID Balance Controller Parameters
pid_balance_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
pid_balance_P = 0.7
pid_balance_I = 0.0
pid_balance_D = 0.0
pid_balance_sample_time = 1.0 / CONTROLLER_FREQUENCY

class Controller(object):

    def __init__(self, bike):

        self.bike = bike
        self.writer = csv.writer(RESULTS)

        # Bike
        self.time_count = 0.0
        self.time_count_gaining_speed = 0.0
        self.gaining_speed_start = 0.0
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

        #self.delta_state_previous = 0
        self.delta_state_list = list()

        # Kalman filter
        #self.kf = Kalman_filter()


    def worker(self):
        self.time_count = self.time_count_gaining_speed

        while self.active:
            start_time = time.time()

            try:


                # Get states and calculate state_references
                self.velocity = self.bike.get_velocity()
                self.states_and_extra_data=self.get_states()
                self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
                self.extra_data = self.states_and_extra_data[3][:] # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]

                # Get GPS position
                self.gpspos = gps.get_position()
                x_measured_GPS = self.gpspos[0]
                y_measured_GPS = self.gpspos[1]

                # Update Kalman filter
                position_kalman = self.kf.update(x_measured_GPS, y_measured_GPS, steering_angle, velocity)

                print 'x_measured_GPS = %f ; y_measured_GPS = %f ; x_kalman = %f ; y_kalman = %f' % (x_measured_GPS,y_measured_GPS,position_kalman[0],position_kalman[1])

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

            # Do a speed test
            # self.bike.set_velocity(INITIAL_SPEED)
            pid_velocity_reference = INITIAL_SPEED
            # if self.time_count > 70:
            #     self.bike.set_velocity(INITIAL_SPEED)
            #     pid_velocity_reference = INITIAL_SPEED
            #     # print 'Exceeded TEST TIME'
            # elif self.time_count > 50:
            #     self.bike.set_velocity(4)
            #     pid_velocity_reference = 4
            # elif self.time_count > 30:
            #     self.bike.set_velocity(2)
            #     pid_velocity_reference = 2
            # else:
            #     pid_velocity_reference = INITIAL_SPEED

            # End test time condition
            if self.time_count > TEST_TIME:
                self.stop()
                print 'Exceeded TEST TIME'

            # Calculation Time
            calculation_time = time.time() - start_time
            self.time_count += calculation_time
            time_0=time.time()

            # Log data
            current_reading = round(float(ADC.read(CURRENT_SENSE_PORT) * 10.0), 2)
            rpm_reading = round(float(ADC.read(RPM_SENSE_PORT) * 18000.0), 2)
            # Log data
            self.writer.writerow(
                (self.time_count, calculation_time, pid_velocity_reference, self.velocity, self.states[0],
                 self.states[1],
                     self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
                 self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
                 self.extra_data[4], self.extra_data[5], self.extra_data[6],current_reading,rpm_reading,self.extra_data[7]))
            print 'Time=%f\tCalc Time = %f\tVel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f\tCurrent = %f\tRPM = %f\t' % (
                self.time_count, calculation_time, self.velocity, 57.29577*self.states[0],57.29577*self.extra_data[0],57.29577*self.states[1],current_reading,rpm_reading)

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
        print "start in 5 secs"
        time.sleep(5)

        imu_data = self.bike.get_imu_data()
        self.states[0] = math.atan2(imu_data[1]/imu_data[2])
        self.phi_state = self.states[0]

        if pid_velocity_active:
            self.writer.writerow(('P = %f' % pid_velocity_P, 'I = %f' % pid_velocity_I, 'D = %f' % pid_velocity_D))

        self.writer.writerow(
            ('Time', 'Calculation Time', 'Reference Velocity', 'Measured Velocity', 'Phi', 'Delta', 'Phi Dot',
             'Control Input', 'Velocity Control Input', 'phi_roll_comp', 'phi_uncomp', 'phi_dot',
             'a_x', 'ay_roll_comp', 'ay', 'az', 'CurrentReading', 'MotorRPM', 'phi_Gyro_intArd' ))  # headers


        # Let the bike get to speed
        start_time = 0.0
        self.bike.set_velocity(INITIAL_SPEED)
        start_time_gaining_speed = time.time()
        self.gaining_speed_start = start_time_gaining_speed
        while start_time < speed_up_time:
            start_time_gaining_speed = time.time()
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print 'EMERGENCY STOP'
                break


            print 'GAINING SPEED'

            # Get states and calculate state_references
            self.velocity = self.bike.get_velocity()
            self.states_and_extra_data = self.get_states()
            self.states = self.states_and_extra_data[0:3]  # [roll_angle, handlebar_angle, roll_angular_velocity]
            self.extra_data = self.states_and_extra_data[3][:]  # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]

            # Find Global Angles and Coordinates
            # if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
            #     self.x, self.y, self.psi, self.nu = global_angles_and_coordinates(self.velocity, self.TIME_CONSTANT,
            #                                                                       LENGTH_A, LENGTH_B, self.states[1],
            #                                                                       self.psi, self.x, self.y)

            self.state_references = self.get_state_references(self.velocity)

            # PID Velocity Control
            if pid_velocity_active:
                self.pid_velocity_control_signal = self.pid_velocity.update(self.velocity)
                self.bike.set_velocity(self.pid_velocity_control_signal)

            # if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
            #     self.distance_travelled += self.velocity * self.TIME_CONSTANT

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

            # Do a speed test
            # self.bike.set_velocity(INITIAL_SPEED)
            pid_velocity_reference = INITIAL_SPEED
            self.lqr_balance_control_signal = 0

            # Calculation Time
            calculation_time = time.time() - start_time_gaining_speed
            self.time_count_gaining_speed += calculation_time
            time_0=time.time()
            current_reading = round(float(ADC.read(CURRENT_SENSE_PORT) * 10.0), 2)
            rpm_reading = round(float(ADC.read(RPM_SENSE_PORT) * 18000.0), 2)
            # Log data
            self.writer.writerow(
                (self.time_count_gaining_speed, calculation_time, pid_velocity_reference, self.velocity, self.states[0],
                 self.states[1],
                     self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
                 self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
                 self.extra_data[4], self.extra_data[5], self.extra_data[6],current_reading,rpm_reading, self.extra_data[7]))
            print 'Time=%f\tCalc Time = %f\tVel = %f\tphi = %f\tphi_ardu = %f\tdelta = %f\tCurrent = %f\tRPM = %f\t' % (
                self.time_count_gaining_speed, calculation_time, self.velocity, 57.29577*self.states[0],57.29577*self.extra_data[0],57.29577*self.states[1],current_reading,rpm_reading)
            start_time = self.time_count_gaining_speed
            # Control Frequency
            calculation_time = time.time() - start_time_gaining_speed
            time_1=time.time()
            self.time_count_gaining_speed += time_1 - time_0
            if calculation_time < self.TIME_CONSTANT:
                time.sleep(self.TIME_CONSTANT - calculation_time)
            time_2=time.time()
            self.time_count_gaining_speed += time_2 - time_1


        imu_data = self.bike.get_imu_data()
        self.states[0] = math.atan2(imu_data[1]/imu_data[2])

        self.active = True
        self.worker()
        # self.thread = Thread(target=lambda: self.worker())
        # self.thread.start()

    def stop(self):
        self.active = False
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
        #self.delta_state_list = np.append(self.delta_state_list,delta_state)
        ##self.delta_state_list = butter_lowpass_filter(self.delta_state_list, cutoff, fs, order)
        #self.delta_state_list = moving_average_filter(self.delta_state_list, windowSize)
        #delta_state_filt = self.delta_state_list[-1]
        #print 'delta_state = ' + str(delta_state) +  ' ; delta_state_filt = ' + str(delta_state_filt)
        ##delta_state = delta_state_filt

        imu_data = self.bike.get_imu_data()

        # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
        # imu_data = [ax ay az gx gy gz]

        ay = imu_data[1]
        az = imu_data[2]
        gx = imu_data[3]

        phi_d_state = gx

        phi_acc = atan2(ay, az)
        self.phi_state = self.complementary_coef * (self.phi_state + gx * sample_time) + (1 - self.complementary_coef) * phi_acc

        return [self.phi_state, delta_state, phi_d_state, imu_data]

    def get_state_references(self, velocity):

        if PATH_TYPE == 'CIRCLE' or PATH_TYPE == 'STRAIGHT':
            lateral_error, angular_error, psi_ref = calculate__path_error(PATH_TYPE, self.distance_travelled, self.x,
                                                                          self.y, self.psi, PATH_RADIUS)

            # PID Control Lateral Position Block in Simulink
            self.pid_lateral_position_control_signal = self.pid_lateral_position.update(
                -lateral_error)  # Negative input due to PID code. See initiliazation for more info.
            #self.pid_lateral_position_control_signal = self.pid_lateral_position.update(
            #    lateral_error)  # Negative input due to PID code. See initiliazation for more info.

            # PID Control Direction Block in Simulink
            self.pid_direction_control_signal = self.pid_direction.update(
                -(-self.pid_lateral_position_control_signal + psi_ref - self.nu))  # Negative input due to PID code.
            #self.pid_direction_control_signal = self.pid_direction.update(
            #    (-self.pid_lateral_position_control_signal + psi_ref - self.nu))  # Negative input due to PID code.

            # PID Control Steering Block in Simulink
            self.pid_steering_control_signal = self.pid_steering.update(
                -(self.pid_direction_control_signal - self.states[1]))
            #self.pid_steering_control_signal = self.pid_steering.update(
            #    (self.pid_direction_control_signal - self.states[1]))

            delta_ref = self.pid_steering_control_signal
            phi_ref = math.atan(((numpy.power(velocity, 2) / (GRAVITY * LENGTH_B)) * delta_ref))
            #phi_ref = -math.atan(((numpy.power(velocity, 2) / (GRAVITY * LENGTH_B)) * delta_ref))

            print 'lateral_error = ' + str(lateral_error) + ' ; angular_error = ' + str(angular_error) + ' ; psi_ref = ' + str(delta_ref) + ' ; psi_ref = ' + str(delta_ref) + ' ; pid_lateral_position_control_signal = ' + str(self.pid_lateral_position_control_signal) + str(psi_ref) + ' ; pid_direction_control_signal = ' + str(self.pid_direction_control_signal) + ' ; pid_steering_control_signal = ' + str(self.pid_steering_control_signal )
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
            # return [gain1, gain2, gain3]
            # return [gain1/5, gain2/5, gain3/5]
            return [gain1*2, gain2*2, gain3*2]
            #return [gain1*1, gain2*2, gain3*0.5]
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
            print 'Exceeded MAX_HANDLEBAR_ANGLE of %f deg' %(max_handlebar_angle * 57.3)
            self.upperSaturation = True
        elif handlebar_angle < min_handlebar_angle:
            print 'Exceeded MIN_HANDLEBAR_ANGLE of %f deg' %(max_handlebar_angle * 57.3)
            self.lowerSaturation = True
        else:
            self.upperSaturation = False
            self.lowerSaturation = False

    def keep_the_bike_stable(self, velocity, references, states):

        feedback_gains = self.get_feedback_gains(velocity)

        self.lqr_balance_control_signal = sum([(r - x) * k for r, x, k in zip(
            references, states, feedback_gains
        )])

        #print 'velocity = ' + str(velocity) + ' ; references = ' + str(references) + ' ; states = ' + str(states) + ' ; feedback_gains = ' + str(feedback_gains) + ' ; lqr_balance_control_signal = ' + str(self.lqr_balance_control_signal)
        ##print 'velocity = %.3f ; references = %.3f ; states = %.3f ; feedback_gains = %.3f ; lqr_balance_control_signal = %.3f' % (velocity,57.29577*references,57.29577*states,feedback_gains,57.29577*self.lqr_balance_control_signal)
        ##print 'velocity = %.3f ; references = %.3f ; states = %.3f' % (velocity,references,57.29577*states)

        # self.pid_balance_control_signal = self.pid_balance.update(states[0])
        # print 'phi(deg) = ' + str(57.29577*states[0]) + ' ; pid_signal(deg) = ' + str(57.29577*self.pid_balance_control_signal)

        if self.lqr_balance_control_signal > deadband or self.lqr_balance_control_signal < -deadband:
            self.controller_set_handlebar_angular_velocity(self.lqr_balance_control_signal)
            # self.controller_set_handlebar_angular_velocity(self.pid_balance_control_signal)
        else:
            self.controller_set_handlebar_angular_velocity(0)
