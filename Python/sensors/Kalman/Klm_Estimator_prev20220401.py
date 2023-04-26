#!/usr/bin/python3
"""This code is for EKF Implementation in Python, hopefully can be integrated into BeagleBone or MyRIO """

from .KalmanData import KalmanData as data
from .kalmanProfile import *

from .stateestimator_setting import black_bike_parameters
import numpy as np
from math import cos, sin, tan, atan2, atan, asin, sqrt, isnan
from numpy import deg2rad
from .kf_lin import *
import os

# import .ekf

class Klm_Estimator(object):
    def __init__(self, lat_0, lon_0):
        # self.n = len(kalmanProfile['P0'])    # Nr of states
        n = 8
        self.stationary_kalman_ON = True
        self.ts = np.zeros(shape=(0,1), dtype='float64')
        self.Xs = np.zeros(shape=(0,n,1), dtype='float64')
        self.X_est = np.zeros(shape=(n,1), dtype='float64')
        self.P_est = np.ones(shape=(n,n), dtype='float64')
        self.X_pred = np.zeros(shape=(n,1), dtype='float64')
        self.P_pred = np.ones(shape=(n,n), dtype='float64')
        self.X_cfs = np.zeros(shape=(0,n,1), dtype='float64')

        self.params = black_bike_parameters().astype(np.float64)
        self.sensors = kalmanProfile['sensors']
        self.model = kalmanProfile['model']
        self.data = data(lat_0, lon_0)  # Data container
        # self.ekf = ekf()


        self.Psi_prev = 0
        self.Global_X_prev_GPS = 0
        self.Global_Y_prev_GPS = 0
        self.Psi_prev_GPS = 0

        self.dv = 0
        self.a_const = 0.6687
        self.b_const = 1.15
        self.c_const = 0.06
        self.d_const = 0.4
        self.g_const = 9.81
        self.r_o_const = 0.694 / 2
        self.N_rps_const = 3
        self.R_e_const = 6357000
        self.alpha_0_radian = deg2rad(lat_0) # Radian
        self.gamma_0_radian = deg2rad(lon_0)
        self.inilon_const = lon_0  # Degrees
        self.inilat_const = lat_0
        self.lambda__const = deg2rad(66)

        self.inix_GPS = self.R_e_const * deg2rad(self.inilon_const * cos(deg2rad(self.inilat_const)))
        self.iniy_GPS = self.R_e_const * deg2rad(self.inilat_const)
        self.C_imuenc = np.array([
        [ 0,    0,     0,     0,     1,     0,     0,     0,],
        [ 0,    0,     0,     0,     1,     0,     0,     0,],
        [ 0,    0,     0,     0,     1,     0,     0,     0,],
        [ 0,    0,     0,     0,     1,     0,     0,     0,],
        [ 0,    0,     0,     0,     0,     0,     1,     0,],
        [ 0,    0,     0,     0,     0,     1,     0,     1,],])
        self.C_rps = np.array([
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     0,     0,     1,     0,],
        [0,     0,     0,     0,     0,     1,     0,     1,],
        [0,     0,     1,     0,     0,     0,     0,     0,],])
        self.C_gps = np.array([
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     0,     0,     1,     0,],
        [0,     0,     0,     0,     0,     1,     0,     1,],
        [1,     0,     0,     0,     0,     0,     0,     0,],
        [0,     1,     0,     0,     0,     0,     0,     0,],
        [0,     0,     1,     0,     0,     0,     0,     0,],
        [0,     0,     0,     1,     0,     0,     0,     0,],
        [0,     0,     0,     0,     0,     0,     0,     1,],])
        self.C_rps_gps = np.array([
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     0,     0,     1,     0,],
        [0,     0,     0,     0,     0,     1,     0,     1,],
        [0,     0,     1,     0,     0,     0,     0,     0,],
        [1,     0,     0,     0,     0,     0,     0,     0,],
        [0,     1,     0,     0,     0,     0,     0,     0,],
        [0,     0,     1,     0,     0,     0,     0,     0,],
        [0,     0,     0,     1,     0,     0,     0,     0,],
        [0,     0,     0,     0,     0,     0,     0,     1,],])
        self.Ppred_matrix = np.array(
        [   [ 1.5e-3,        0, 1.5e-7,       0,        0,        0,       0,       0],
            [      0,   1.5e-3,      0,  2.8e-8, -1.1e-10,   4.2e-9,  1.0e-9, -4.2e-9],
            [ 1.5e-7,        0, 1.3e-6,       0,        0,        0,       0,       0],
            [      0,   2.8e-8,      0,  4.5e-6,  -3.5e-9,   1.3e-7,  3.2e-8, -1.3e-7],
            [      0, -1.1e-10,      0, -3.5e-9,   1.6e-6, -5.2e-10,  1.8e-6, 5.2e-10],
            [      0,   4.2e-9,      0,  1.3e-7, -5.2e-10,   2.1e-8,  4.8e-9, -2.0e-8],
            [      0,   1.0e-9,      0,  3.2e-8,   1.8e-6,   4.8e-9,  6.4e-5, -4.8e-9],
            [      0,  -4.2e-9,      0, -1.3e-7,  5.2e-10,  -2.0e-8, -4.8e-9,  2.0e-8],])
        self.K_rps_gps_final = np.array([
            [       0,       0,        0,       0,       0,       0, 4.6e-3,   0.55,       0, 9.5e-5,       0,       0],
            [ -6.3e-9, -6.3e-7, -2.5e-10, -6.3e-7,  2.7e-6,  1.9e-4,      0,      0,    0.55,      0,  1.6e-4, -2.4e-8],
            [       0,       0,        0,       0,       0,       0,  0.069, 5.3e-5,       0, 1.4e-3,       0,       0],
            [ -3.5e-7, -3.5e-5,  -1.4e-8, -3.5e-5,  1.5e-4,  6.0e-4,      0,      0,  9.8e-6,      0,   0.043, -1.3e-6],
            [  1.6e-4,   0.016,   6.4e-6,   0.016,  8.4e-3, -2.7e-5,      0,      0, -3.9e-8,      0, -3.5e-5,  5.3e-9],
            [ -5.3e-8, -5.3e-6,  -2.1e-9, -5.3e-6,  2.3e-5,    0.97,      0,      0,  1.5e-6,      0,  1.3e-3, -2.1e-7],
            [  1.7e-4,   0.017,   6.7e-6,   0.017,    0.35,  2.1e-3,      0,      0,  3.4e-7,      0,  3.0e-4, -4.6e-8],
            [  5.3e-8,  5.3e-6,   2.1e-9,  5.3e-6, -2.3e-5,  9.8e-5,      0,      0, -1.5e-6,      0, -1.3e-3,  2.1e-7],
        ])
        self.K_gps_final = np.array([
            [       0,       0,        0,       0,       0,       0,   0.67,       0, 1.1e-4,       0,       0],
            [ -7.6e-9, -7.6e-7, -3.0e-10, -7.6e-7,  3.1e-6,  5.4e-5,      0,    0.67,      0,  1.9e-4, -2.9e-8],
            [       0,       0,        0,       0,       0,       0, 6.4e-5,       0, 1.4e-3,       0,       0],
            [ -3.5e-7, -3.5e-5,  -1.4e-8, -3.5e-5,  1.4e-4,  2.3e-4,      0,  1.2e-5,      0,   0.044, -1.3e-6],
            [  1.6e-4,   0.016,   6.5e-6,   0.016,  7.7e-3, -9.6e-6,      0, -4.7e-8,      0, -3.5e-5,  5.2e-9],
            [ -5.2e-8, -5.2e-6,  -2.1e-9, -5.2e-6,  2.1e-5,    0.99,      0,  1.8e-6,      0,  1.3e-3, -2.0e-7],
            [  1.5e-4,   0.015,   6.1e-6,   0.015,    0.42,  7.3e-4,      0,  3.9e-7,      0,  2.9e-4, -4.3e-8],
            [  5.2e-8,  5.2e-6,   2.1e-9,  5.2e-6, -2.1e-5,  9.9e-5,      0, -1.8e-6,      0, -1.3e-3,  2.0e-7],
        ])
        self.K_rps_final = np.array([
                          [       0,       0,        0,       0,       0,       0, 0.016],
                          [ -2.3e-8, -2.3e-6, -9.1e-10, -2.3e-6,  1.1e-5,  4.5e-4,     0],
                          [       0,       0,        0,       0,       0,       0, 0.068],
                          [ -3.7e-7, -3.7e-5,  -1.5e-8, -3.7e-5,  1.7e-4,  6.7e-4,     0],
                          [  1.6e-4,   0.016,   6.3e-6,   0.016,  9.0e-3, -3.1e-5,     0],
                          [ -5.3e-8, -5.3e-6,  -2.1e-9, -5.3e-6,  2.5e-5,    0.97,     0],
                          [  1.8e-4,   0.018,   7.2e-6,   0.018,    0.32,  2.3e-3,     0],
                          [  5.3e-8,  5.3e-6,   2.1e-9,  5.3e-6, -2.5e-5,  9.7e-5,     0],])
        self.K_imu_enc_final = np.array([
            [       0,       0,        0,       0,       0,       0],
            [ -1.1e-8, -1.1e-6, -4.3e-10, -1.1e-6,  5.0e-6,  4.1e-4],
            [       0,       0,        0,       0,       0,       0],
            [ -3.5e-7, -3.5e-5,  -1.4e-8, -3.5e-5,  1.6e-4,  6.2e-4],
            [  1.6e-4,   0.016,   6.4e-6,   0.016,  8.9e-3, -2.8e-5],
            [ -5.2e-8, -5.2e-6,  -2.1e-9, -5.2e-6,  2.4e-5,    0.97],
            [  1.8e-4,   0.018,   7.1e-6,   0.018,    0.32,  2.1e-3],
            [  5.2e-8,  5.2e-6,   2.1e-9,  5.2e-6, -2.4e-5,  9.7e-5],
        ])
        self.P_est = self.Ppred_matrix
        self.R_roll = np.diag([0.1, 0.01, 1, 1e-1]) ** 2
        self.Q = np.diag([ 1, 1, 0.01, 0.01, deg2rad(1), deg2rad(1), deg2rad(30), deg2rad(1)*sqrt(0.01) ]) ** 2
        self.imu_R = np.diag([ 0.0062, 0.0062, 0.0175, 2e-4, 0.0035, 0.0022 ])

        self.enc_R = 9.1935e-10

        self.rps_R = 1e-3
        # self.gps_R_normalconfidence = np.diag([1.7572e-13, 1.2505e-13, 2.1567e-5, 9.5228e-5])
        # self.gps_R_highconfidence = np.diag([1.7572e-16, 1.2505e-16, 2.1567e-8, 9.5228e-10]) * 100
        self.gps_R_highconfidence = np.diag([4e-2, 4e-2, 3e-2, 0.01]) ** 2
        self.gps_R = self.gps_R_highconfidence

        self.Global_X_prev = 0
        self.Global_Y_prev = 0

        self.X_h = 0
        self.Y_h = 0
        self.v_h = 3
        self.psi_h = 0
        self.phi_h = 0
        self.delta_h = 0
        self.phidot_h = 0
        self.delta0_h = 0
        self.Uk = 0
        self.dt = 0.01
        self.A_m = np.array([[ 1.0,   0, self.dt /(0.337*tan(0.914*self.delta_h) ** 2 + 1.0) ** (1/2) ,   0,       0,          -(0.337* self.dt *self.v_h*tan(0.914*self.delta_h)*(0.914*tan(0.914*self.delta_h) ** 2 + 0.914))/(0.337*tan(0.914*self.delta_h) ** 2 + 1.0) ** (3/2),   0,   0],
                    [   0, 1.0, (0.581* self.dt *tan(0.914*self.delta_h))/(0.337*tan(0.914*self.delta_h) ** 2 + 1.0) ** (1/2) ,   0,       0, (0.581* self.dt *self.v_h*(0.914*tan(0.914*self.delta_h) ** 2 + 0.914))/(0.337*tan(0.914*self.delta_h) ** 2 + 1.0) ** (1/2)  - (0.196* self.dt *self.v_h*tan(0.914*self.delta_h) ** 2*(0.914*tan(0.914*self.delta_h) ** 2 + 0.914))/(0.337*tan(0.914*self.delta_h) ** 2 + 1.0) ** (3/2),   0,   0],
                    [   0,   0,          1.0,   0,       0,     0,   0,   0],
                    [   0,   0,               0.868* self.dt *tan(0.914*self.delta_h), 1.0,       0,               0.868* self.dt *self.v_h*(0.914*tan(0.914*self.delta_h) ** 2 + 0.914),   0,   0],
                    [   0,   0,            0,   0,     1.0,     0,   self.dt ,   0],
                    [   0,   0,            0,   0,       0,   1.0,   0,   0],
                    [   0,   0,     0.993* self.dt * self.Uk +  self.dt *(0.993* self.Uk + 2.97*self.delta_h*self.v_h),   0, 18.4* self.dt ,              self.dt *(1.49*self.v_h ** 2 - 1.09), 1.0,   0],
                    [   0,   0,     0,   0,       0,     0,   0, 1.0],])
        self.B_m = np.array([[0, 0, 0 ,0 ,0 , 2*self.dt, (2* self.dt * self.v_h), 0]]).T

        print('IMU : Searching for calibration file %s/sensors/Acc_Cali.txt' %(os.getcwd()))
        with open('./sensors/Acc_Cali.txt', 'r') as f:
            self.acc_roll_offset = float(f.read())

    def estimate(self, controllerOBJ):

        (UpdateFlg, Yks) = self.data.update_check(controllerOBJ)
        self.Uk = controllerOBJ.steering_rate
        self.dt = max([controllerOBJ.loop_time, 0.01])

        start_time = time.time()


        # (self.X_est, self.P_est) = filter(self.params, UpdateFlg, dt, self.model, self.X_est, self.P_est, Uk, self.sensors, Yks)
        Xk = self.X_est
        X_h = Xk[0]
        Y_h = Xk[1]
        v_h = Xk[2]
        psi_h = Xk[3]
        phi_h = Xk[4]
        delta_h = Xk[5]
        phidot_h = Xk[6]
        delta0_h = Xk[7]


        deltadot_ref = self.Uk
        # Speed Sensor
        Yk_rps = Yks['rps']
        v_rps = Yk_rps
        update_rps = UpdateFlg['rps']

        # Enc
        Yk_enc = Yks['enc']
        delta_enc = Yk_enc
        update_enc = UpdateFlg['enc']

        # GPS
        Yk_gps = Yks['gps']
        lat_gps = Yk_gps[0]
        lon_gps = Yk_gps[1]
        vel_gps = Yk_gps[2]

        update_gps = UpdateFlg['gps']
        print(Yk_gps[3])
        if isnan(Yk_gps[3]):
            print('Yk3 is NAN!')
            print(Yk_gps[3])
            Yk_gps[3] = psi_h
            print('Overwritten as %f' %phi_h)
        heading_gps = Yk_gps[3]

        # IMU
        Yk_imu = Yks['imu']
        gx_imu = Yk_imu[3]
        gy_imu = Yk_imu[4]
        gz_imu = Yk_imu[5] # Make the Z-axis Upwards
        ax_imu = Yk_imu[0]
        ay_imu = Yk_imu[1]
        az_imu = Yk_imu[2]
        update_imu = UpdateFlg['imu']



        # roll_virtual_sensor = np.zeros(shape=(0, 4), dtype='float64')
        roll_virtual_sensor = np.zeros(shape=(4), dtype='float64')
        Centrip = v_h ** 2 * tan(delta_h * sin(1.1519)) / self.b_const / self.g_const
        roll_virtual_sensor[0] = atan2(ay_imu - Centrip * cos(phi_h), az_imu + Centrip * sin(phi_h))  - self.acc_roll_offset
        roll_virtual_sensor[1] = -atan(gz_imu * v_h / self.g_const)
        roll_virtual_sensor[2] = np.sign(gz_imu) * asin( gy_imu / sqrt(gy_imu ** 2  + gz_imu ** 2 ))
        roll_virtual_sensor[3] = - v_h ** 2 / (self.g_const * self.b_const) * delta_h
        # print(roll_virtual_sensor)
        # psidot_virtual_sensor[0] = gy_imu * sin(phi_h) + gz_imu * cos(phi_h)
        phi_dot_sensor = gx_imu

        x_GPS_NED = self.R_e_const * deg2rad(lon_gps * cos(deg2rad(self.inilat_const))) - self.inix_GPS
        y_GPS_NED = self.R_e_const * deg2rad(lat_gps) - self.iniy_GPS

        x_GPS_glb = x_GPS_NED
        y_GPS_glb = y_GPS_NED

        dX_global = x_GPS_glb - self.Global_X_prev_GPS
        dY_global = y_GPS_glb - self.Global_Y_prev_GPS

        dX_local = dX_global * cos(self.Psi_prev_GPS) + dY_global * sin(self.Psi_prev_GPS)
        dY_local = -dX_global * sin(self.Psi_prev_GPS) + dY_global * cos(self.Psi_prev_GPS)

        Yk_gps[0:2] = np.array([dX_local, dY_local])
        Yk_gps = Yk_gps.reshape((4,))


        delta_0_sensor = delta0_h - 0.01 * (Yk_gps[1] - Y_h)

        if update_rps and update_gps:
            sensor_out = np.hstack((roll_virtual_sensor,
                                    phi_dot_sensor,
                                    Yk_enc,
                                    Yk_rps,
                                    Yk_gps,
                                    delta_0_sensor,
                                    )).T
            C_obsv = self.C_rps_gps
            R = np.diag(np.hstack((np.diag(self.R_roll), self.imu_R[3][3], self.enc_R, self.rps_R, np.diag(self.gps_R), 1e-1 )))
        elif not update_rps and update_gps:
            sensor_out = np.hstack((roll_virtual_sensor,
                                    phi_dot_sensor,
                                    Yk_enc,
                                    Yk_gps,
                                    delta_0_sensor,
                                    )).T
            C_obsv = self.C_gps
            R = np.diag(np.hstack((np.diag(self.R_roll), self.imu_R[3][3], self.enc_R, np.diag(self.gps_R), 1e-1 )))
        elif update_rps and not update_gps:
            sensor_out = np.hstack((roll_virtual_sensor,
                                    phi_dot_sensor,
                                    Yk_enc,
                                    Yk_rps,
                                    )).T
            C_obsv = self.C_rps
            R = np.diag(np.hstack((np.diag(self.R_roll), self.imu_R[3][3], self.enc_R, self.rps_R)))
        else:
            sensor_out = np.hstack((roll_virtual_sensor, phi_dot_sensor, Yk_enc )).T
            C_obsv = self.C_imuenc
            R = np.diag(np.hstack((np.diag(self.R_roll), self.imu_R[3][3], self.enc_R)))

        if not self.stationary_kalman_ON:
            # Kalman Update:
            Yk = sensor_out
            Pk = self.P_est
            Xk = self.X_est
            Hx = C_obsv
            hx = C_obsv.dot(Xk)
            S = Hx.dot(Pk).dot(Hx.T) + R
            # invS = np.diag(1./np.diag(S))  # Approximation of S by only keeping diagnal terms
            # K = Pk.dot(Hx.T).dot(invS)
            K = Pk.dot(Hx.T).dot(np.linalg.inv(S))
            v = K.dot((Yk - hx.T).T)

            Xk = Xk + v
            Pk = Pk - K.dot(S).dot(K.T)

            Pk = (Pk + np.transpose(Pk)) / 2

            self.X_est = Xk
            self.P_est = Pk
            # Kalman Prediction:
            # params = params.astype(np.float64)
            # try:
            X_h = Xk[0]
            Y_h = Xk[1]
            v_h = Xk[2]
            psi_h = Xk[3]
            phi_h = Xk[4]
            delta_h = Xk[5]
            phidot_h = Xk[6]
            delta0_h = Xk[7]
            dv = 0
            dt = 0.01

            # f_m = np.array([[X_h + (dt*v_h)/(0.34*tan(0.91*delta_h) ** 2 + 1.0) ** (1/2) ,Y_h + (0.58*dt*v_h*tan(0.91*delta_h))/(0.34*tan(0.91*delta_h) ** 2 + 1.0) ** (1/2) ,v_h + dt*dv ,psi_h + 0.87*dt*v_h*tan(0.91*delta_h) ,phi_h + dt*phidot_h ,delta_h + 2.0*dt*self.Uk ,phidot_h + dt*(18.0*phi_h + 0.99*self.Uk*v_h + delta_h*(1.5*v_h ** 2 - 1.1)) + 0.99*dt*self.Uk*v_h ,delta0_h]])
            f_m = self.A_m * Xk + self.B_m * self.Uk
            Xkp = f_m
            Pkp = np.dot(self.A_m, np.dot(Pk, self.A_m.T)) + np.dot(dt**2, self.Q)

            # Ensure Covariance matrix is symmetrical
            # https://stackoverflow.com/a/30010778/4255176
            Pkp = (Pkp + np.transpose(Pkp)) / 2
        else:
            if update_rps and update_gps:
                K = self.K_rps_gps_final
            elif not update_rps and update_gps:
                K = self.K_gps_final
            elif update_rps and not update_gps:
                K = self.K_rps_final
            else:
                K = self.K_imu_enc_final
            hx = C_obsv.dot(Xk)
            Xk = Xk + K.dot((sensor_out - hx.T).T)
            self.X_est = Xk

            f_m = self.A_m * Xk + self.B_m * self.Uk
            Xkp = f_m
            Pkp = self.P_est

        if update_gps:

            Global_X = self.Global_X_prev_GPS + Xk[0] * cos(self.Psi_prev_GPS) - Xk[1] * sin(self.Psi_prev_GPS)
            Global_Y = self.Global_Y_prev_GPS + Xk[0] * sin(self.Psi_prev_GPS) + Xk[1] * cos(self.Psi_prev_GPS)

            # Xkp[0][0:2] = Xkp[0][0:2] - np.expand_dims(Xk[0:2], axis=0)
            # Xkp[0][0:2] = Xkp[0][0:2] - np.expand_dims(Xk[0:2], axis=0)
            Xkp[0:2] = Xkp[0:2] - Xk[0:2]

            Xk[0:2] = 0

            self.Global_X_prev_GPS = Global_X
            self.Global_Y_prev_GPS = Global_Y
            self.Psi_prev_GPS = Xk[3]
        else:

            Global_X = self.Global_X_prev_GPS + Xk[0] * cos(self.Psi_prev_GPS) - Xk[1] * sin(self.Psi_prev_GPS)
            Global_Y = self.Global_Y_prev_GPS + Xk[0] * sin(self.Psi_prev_GPS) + Xk[1] * cos(self.Psi_prev_GPS)


        Xk_f = Xk
        Xk_f[0:2] = np.array([Global_X, Global_Y])

        self.X_est = Xk

        self.X_pred = Xkp
        self.P_pred = Pkp



        # print('Kalman Time : \n')
        # print(time.time() - start_time)



        return Xk_f, self.P_est
