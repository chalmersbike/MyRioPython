import time
import math
import numpy as np
# from scipy.signal import butter, lfilter, freqz, filtfilt,lfilter_zi


########################################################################################################################
# PID
class PID(object):
    """
    PID Controller adapted from https://github.com/ivmech/ivPID
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.Reference = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.last_output = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 6.0/57.29577

        self.output = 0.0

        self.current_time = time.time()
        self.last_time = self.current_time

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            lqr_balance_input(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.Reference - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            self.last_output = self.output
            return self.output
        else:
            return self.last_output

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in LENGTH_A PID feedback controller where
        LENGTH_A large change in reference occurs (say LENGTH_A positive change)
        and the integral terms accumulates LENGTH_A significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at LENGTH_A regular interval.
        Based on LENGTH_A pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

    def setReference(self, reference):
        """Change PID reference"""
        self.Reference = reference


########################################################################################################################
# Path tracking
def global_angles_and_coordinates(velocity, time_constant, length_a, length_b, steering_angle, psi, x, y):
    beta = math.atan((length_a / length_b) * math.tan(steering_angle))
    psi_dot = (velocity / length_a) * math.sin(beta)
    psi += psi_dot * time_constant
    nu = psi + beta
    x_dot = velocity * math.cos(nu)
    x += x_dot * time_constant
    y_dot = velocity * math.sin(nu)
    y += y_dot * time_constant

    return x, y, psi, nu




def generate_circular_path(path_radius):
    theta = np.linspace(0.0, math.pi, 5000)
    x_ref = path_radius * math.cos(theta)
    y_ref = path_radius * math.sin(theta)
    return x_ref, y_ref


def generate_straight_path():
    x_ref = np.linspace(0.0, 0.0, 5000)
    y_ref = np.linspace(0.0, 250, 5000)
    return x_ref, y_ref


def calculate_path_error(path_type, distance_travelled, x, y, psi, path_radius):
    """
    This function corresponds to the following blocks in Simulink model:
        Reference Position and Direction in Global CS with variable Look Ahead
        Lateral Error
    """
    if path_type == 'CIRCLE':
        circular_path_angle_ref = np.mod(distance_travelled / (2 * math.pi * path_radius), 1) * (2 * math.pi)
        x_ref = math.cos(circular_path_angle_ref) * path_radius
        y_ref = math.sin(circular_path_angle_ref) * path_radius
        psi_ref = np.mod(circular_path_angle_ref + math.pi / 2, 2 * math.pi)

        lateral_error = math.cos(psi_ref) * (y - y_ref) - math.sin(psi_ref) * (x - x_ref)
        angular_error = psi - psi_ref
        return lateral_error, angular_error, psi_ref

    if path_type == 'STRAIGHT':
        lateral_error = y
        angular_error = psi
        return lateral_error, angular_error, 0.0

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
    elif path_choice == 'file':
        lateral_error = 0.0
        angular_error = 0.0
        psi_ref = 0.0
        return lateral_error, angular_error, psi_ref
    else:
        lateral_error = 0.0
        angular_error = 0.0
        psi_ref = 0.0
        return lateral_error, angular_error, psi_ref




########################################################################################################################
# Filter
# def butter_lowpass(cutoff, fs, order=5):
#     nyq = 0.5 * fs
#     normal_cutoff = cutoff / nyq
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return b, a
#
#
# def butter_lowpass_filter(data, cutoff, fs, order=5):
#     b, a = butter_lowpass(cutoff, fs, order=order)
#     # zi = np.ones(max(len(a), len(b)) - 1) * data[0]
#     zi = lfilter_zi(b, a) * data[0]
#     y,_ = lfilter(b, a, data,zi = zi)
#     return y
#
#
# def moving_average_filter(data, windowSize=5):
#     b = (1/windowSize)*(np.ones((windowSize,)))
#     a = 1
#     y,_ = lfilter(b, a, data)
#     return y