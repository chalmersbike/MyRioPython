import csv
import math
import time
from math import pi as PI
from time import sleep
import numpy as np

import sys

sys.path.append('../../../')
from constants import *

# Controller Parameters
CONTROLLER_FREQUENCY = 50  # Hz

# Bike Parameters
MAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad
MIN_HANDLEBAR_ANGLE = -MAX_HANDLEBAR_ANGLE  # rad

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('CLdelta-%s.csv' % timestr, 'wb')

# Simulation parameters
REFERENCE_VELOCITY = 2.0
DELTA_STEP = -PI / 6.0


class Controller(object):

    def __init__(self, bike):
        self.bike = bike
        self.TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
        self.references = [state['default_reference'] for state in K_MATRIX]
        self.time_count = 0.0
        self.active = False
        self.references = []
        self.states = []
        self.upperSaturation = False
        self.lowerSaturation = False
        self.writer = csv.writer(RESULTS)
        self.u = 0.0
        self.vel_controller = 0.0
        self.velocity = 0.0
        self.calculation_time = 0.0
        self.ESTOP = False

    def worker(self):
        while self.active and self.time_count < 10.0:
            try:
                start_time = time.time()

                # get states and calculate references
                self.velocity = REFERENCE_VELOCITY  # fixed velocity
                self.states = self.get_states()  # [roll_angle, handlebar_angle, roll_angular_velocity]
                self.references = self.get_references(self.velocity)

                # control loop
                self.keep_handlebar_angle_within_safety_margins(self.states[1])
                self.keep_the_bike_stable(self.velocity, self.references, self.states)

                # control frequency
                calculation_time = time.time() - start_time
                if calculation_time < self.TIME_CONSTANT:
                    time.sleep(self.TIME_CONSTANT - calculation_time)
                self.time_count += time.time() - start_time

                # log data
                phi, delta, phi_dot = self.states  # save states as individual variables for data analysis
                phi_ref, delta_ref, phi_dot_ref = self.references

                self.writer.writerow(
                    (self.time_count, calculation_time, self.velocity, phi_ref, delta_ref, delta, self.u))
                print 'Time = %f' % (self.time_count)

            except (ValueError, KeyboardInterrupt):
                self.stop()
                print 'BREAK'

            # Check for ESTOP
            self.ESTOP = self.bike.emergency_stop_check()
            if self.ESTOP:
                self.stop()
                print 'EMERGENCY STOP'
        self.stop()

    def start(self):
        self.ESTOP = self.bike.emergency_stop_check()
        if self.ESTOP:
            print 'ERROR: EMERGENCY STOP ACTIVE'
            raw_input('Press ENTER to continue')

        raw_input('Press ENTER to start')
        self.writer.writerow((['Delta Step = %f' % DELTA_STEP]))
        self.writer.writerow(
            ('Time', 'Calculation Time', 'Reference Velocity', 'Reference Phi', 'Reference Delta', 'Measured Delta',
             'Control Input'))  # headers

        # Time to get into position to start test
        self.time_count = 0.0
        while self.time_count < 10.0:
            time.sleep(self.TIME_CONSTANT)
            self.time_count += self.TIME_CONSTANT
            print self.time_count

        self.time_count = 0.0
        print 'STARTING NOW'

        self.active = True
        self.worker()

    def stop(self):
        self.active = False
        self.bike.stop_all()

    def get_states(self):
        delta_state = self.bike.get_handlebar_angle()
        # imu_data = self.bike.get_imu_data()
        phi_state = 0  # imu_data[0]
        phi_d_state = 0  # imu_data[1]
        return [phi_state, delta_state, phi_d_state]

    def get_references(self, velocity):
        delta_ref = 0.0
        # DELTA STEP INPUT
        if self.time_count >= 5.0:
            delta_ref = DELTA_STEP
            print 'STEP ACTIVATED'

        phi_ref = 0.0  # math.atan(((velocity * velocity) / (GRAVITY * WHEELBASE)) * delta_ref)
        return [phi_ref, delta_ref, 0.0]

    def get_feedback_gains(self, velocity):
        # return [np.interp(velocity, K_VELOCITY_LOOKUP, gain['LQR_gain_multipliers']) for gain in K_MATRIX]
        return [-22.1251, 5.2128, -5.8463]  # gains corresponding to v = 2 where Q = diag([100,100,10]) and R = 10
        # return [-4.6897, 12.1343, -1.2607]  # gains corresponding to v = 8 where Q = diag([100,100,10]) and R = 10

    def controller_set_handlebar_angular_velocity(self, angular_velocity):
        if self.upperSaturation:
            if angular_velocity > 0.0:  # we have exceeded the limit and are trying to move beyond it
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

        self.u = sum([(r - x) * k for r, x, k in zip(
            references, states, feedback_gains
        )])

        self.controller_set_handlebar_angular_velocity(self.u)
