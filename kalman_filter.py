import math
import time
import numpy as np
from time import sleep

from constants import *
from utils import *
import Adafruit_BBIO.ADC as ADC
from controller import *


# Create states-space matrices for the Kalman filter
A_kalman = np.array([[1, 0, sample_time, 0], [0, 1, 0, sample_time], [0, 0, 1, 0], [0, 0, 0, 1]])
B_kalman = np.zeros((4,4))
C_kalman = np.eye(4)
D_kalman = np.zeros((4,4))

# Noise matrices
Q_kalman = np.array([[0.0001, 0, 0, 0], [0, 0.0001, 0, 0], [0, 0, 0, 0.1], [0, 0, 0, 0.1]])
R_kalman = np.array([[0.51, 0, 0, 0], [0, 0.51, 0, 0], [0, 0, 0.0031, 0], [0, 0, 0, 0.0031]])

# Initial conditions for the Kalman filter
x0_kalman = np.zeros((4,1)) # TO CHECK IN SIMULINK
P0_kalman = np.zeros((4,4)) # TO CHECK IN SIMULINK
K0_kalman = np.zeros((4,4)) # TO CHECK IN SIMULINK


class Kalman_filter(object):

    def __init__(self, bike):

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        # Initialize a-priori and a-posteriori estimates
        self.x_kalman_prio = x0_kalman
        self.x_kalman_post = x0_kalman
        self.P_kalman_prio = P0_kalman
        self.P_kalman_post = P0_kalman
        self.K_kalman = K0_kalman


    def update(self, x_measured_GPS, y_measured_GPS, steering_angle, velocity):
        # Update the Kalman filter's estimation
        # See Cecile Savoye's MS thesis (https://drive.google.com/open?id=1WbTN7so_zdlYyP0GJKAHQubhny6ofth8)

        # x_measured_GPS and y_measured_GPS are the positions in x and y measured by the GPS
        # velx_measured and vely_measured are the velocity measured by the Hall sensor and projected to the x and y axis

        # Compute velx_measured and vely_measured from Hall sensor (forward velocity) and encoder (steering angle) measurements
        beta = math.atan((LENGTH_A / LENGTH_B) * math.tan(steering_angle))
        psi_dot = (velocity / LENGTH_A) * math.sin(beta)
        psi += psi_dot * sample_time
        nu = psi + beta
        velx_measured = velocity * math.cos(nu)
        vely_measured = velocity * math.sin(nu)

        # Create measurements (outputs) vector
        y = np.array(x_measured_GPS, velx_measured, y_measured_GPS, vely_measured)

        # Compute a-priori estimates
        self.x_kalman_prio = A_kalman.dot(self.x_kalman_post)
        self.P_kalman_prio = A_kalman.dot(self.P_kalman_post).dot(np.transpose(A_kalman))

        # Compute a-posteriori estimates
        self.K_kalman = self.P_kalman_prio.dot(np.transpose(C_kalman)).dot(np.linalg.inv(C_kalman.dot(self.P_kalman_prio).dot(np.transpose(C_kalman)) + R_kalman))
        self.x_kalman_post = self.x_kalman_prio + self.K_kalman.dot(y - self.x_kalman_prio)
        self.P_kalman_post = (eye(4) - self.K_kalman.dot(C_kalman)).dot(self.P_kalman_prio)

        # Return the estimated states
        # return self.x_kalman_post # Return as a numpy array
        return self.x_kalman_post.tolist() # Return as a Python list