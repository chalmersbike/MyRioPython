#!/usr/bin/python3
"""This code is for EKF Implementation in Python, hopefully can be integrated into BeagleBone or MyRIO """
import cython
# from cython.cimports.libc
# from cython.libc.math cimport cos, sin, tan, atan2, atan, asin, sqrt, isnan
from libc.math cimport cos, sin, tan, atan2, atan, asin, sqrt, isnan, abs
from .KalmanData import KalmanData as data
import numpy as np
cimport numpy as np
from numpy import deg2rad, sign, unwrap
import time

@cython.wraparound(False)
@cython.boundscheck(False)
cdef class Klm_Estimator:
    cdef int n, N_rps_const
    cdef bint stationary_kalman_ON
    cdef double Psi_prev, Global_X_prev_GPS, dv ,a_const ,b_const ,c_const ,d_const ,g_const ,r_o_const
    cdef double R_e_const, alpha_0_radian, gamma_0_radian, inilon_const, inilat_const, lambda__const
    cdef double Global_Y_prev_GPS, Psi_prev_GPS, inix_GPS, iniy_GPS
    cdef double Global_X_prev, Global_Y_prev, X_h, Y_h, v_h, psi_h, phi_h, delta_h, phidot_h, delta0_h, Uk, dt, prev_klm_time
    cdef readonly double[:] ts, X_est, X_pred, Xs, X_cfs
    cdef double[:,:] P_est, P_pred, C_imuenc, C_rps, C_gps, C_rps_gps, Ppred_matrix, K_rps_gps_final, K_gps_final, K_rps_final, K_imu_enc_final, A_m, B_m
    cdef double[:,:] gps_R_highconfidence, gps_R, R_roll, Q,
    cdef public object data

    def __init__(self, lat_0, lon_0):
        # self.n = len(kalmanProfile['P0'])    # Nr of states
        cdef int n
        n = 8
        self.stationary_kalman_ON = True
        self.ts = np.zeros(shape=(1), dtype='double')
        self.Xs = np.zeros(shape=(n), dtype='double')
        self.X_est = np.zeros(shape=(n), dtype='double')
        self.P_est = np.ones(shape=(n, n), dtype='double')
        self.X_pred = np.zeros(shape=(n), dtype='double')
        self.P_pred = np.ones(shape=(n, n), dtype='double')
        self.X_cfs = np.zeros(shape=(n), dtype='double')

        self.data = data(lat_0, lon_0)  # Data container


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
        [ 0,    0,     0,     0,     0,     1,     0,     1,],], dtype = 'double')
        self.C_rps = np.array([
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     1,     0,     0,     0,],
        [0,     0,     0,     0,     0,     0,     1,     0,],
        [0,     0,     0,     0,     0,     1,     0,     1,],
        [0,     0,     1,     0,     0,     0,     0,     0,],] , dtype = 'double')
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
        [0,     0,     0,     0,     0,     0,     0,     1,],] , dtype = 'double')
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
        [0,     0,     0,     0,     0,     0,     0,     1,],] , dtype = 'double')
        self.Ppred_matrix = np.array(
        [   [ 1.5e-3,        0, 1.5e-7,       0,        0,        0,       0,       0],
            [      0,   1.5e-3,      0,  2.8e-8, -1.1e-10,   4.2e-9,  1.0e-9, -4.2e-9],
            [ 1.5e-7,        0, 1.3e-6,       0,        0,        0,       0,       0],
            [      0,   2.8e-8,      0,  4.5e-6,  -3.5e-9,   1.3e-7,  3.2e-8, -1.3e-7],
            [      0, -1.1e-10,      0, -3.5e-9,   1.6e-6, -5.2e-10,  1.8e-6, 5.2e-10],
            [      0,   4.2e-9,      0,  1.3e-7, -5.2e-10,   2.1e-8,  4.8e-9, -2.0e-8],
            [      0,   1.0e-9,      0,  3.2e-8,   1.8e-6,   4.8e-9,  6.4e-5, -4.8e-9],
            [      0,  -4.2e-9,      0, -1.3e-7,  5.2e-10,  -2.0e-8, -4.8e-9,  2.0e-8],], dtype = 'double')
        self.K_rps_gps_final = np.array([
            [       0,        0,       0,        0,        0, -0.00014,   0.0065, 0.998,        0, 0.00001,        0,      0],
            [ 0.00001,  0.00024,       0,  0.00024, -0.00003,  0.01758, -0.00187,     0,  0.99796,       0, -0.00062,      0],
            [       0,        0,       0,        0,        0,   0.0003,  0.65093,     0,        0, 0.00121, -0.00001,      0],
            [       0,        0,       0,        0,        0, -0.20239, -0.00002,     0,        0,       0,  0.38196,      0],
            [ 0.01279,  0.31971,  0.0032,  0.31971,  0.00095, -0.00022,  0.00002,     0,        0,       0,  0.00001,      0],
            [       0, -0.00007,       0, -0.00007,  0.00001,  0.99444,  0.00058,     0,  0.00001,       0,  0.00022,      0],
            [ 0.01208,  0.30206, 0.00302,  0.30206,  0.96725,  0.00835, -0.00089,     0, -0.00001,       0, -0.00029,      0],
            [       0,        0,       0,        0,        0,        0,        0,     0,        0,       0,        0, 0.0098],
        ], dtype = 'double')
        self.K_gps_final = np.array([
            [       0,        0,       0,        0,        0, -0.00014, 0.998,        0,   0.0001,        0,      0],
            [ 0.00001,  0.00024,       0,  0.00024, -0.00003,  0.01767,     0,  0.99796, -0.00001, -0.00062,      0],
            [       0,        0,       0,        0,        0,  0.00011,     0,        0,   0.0098,        0,      0],
            [       0,        0,       0,        0,        0, -0.20239,     0,        0,        0,  0.38196,      0],
            [ 0.01279,  0.31971,  0.0032,  0.31971,  0.00095, -0.00022,     0,        0,        0,  0.00001,      0],
            [       0, -0.00007,       0, -0.00007,  0.00001,  0.99442,     0,  0.00001,        0,  0.00022,      0],
            [ 0.01208,  0.30206, 0.00302,  0.30206,  0.96725,  0.00838,     0, -0.00001,        0,  -0.0003,      0],
            [       0,        0,       0,        0,        0,        0,     0,        0,        0,        0, 0.0098],
        ], dtype = 'double')
        self.K_rps_final = np.array([
                          [       0,        0,       0,        0,       0,        0,  0.00227],
                          [       0,        0,       0,        0,       0,  0.00013, -0.00001],
                          [       0,        0,       0,        0,       0,  0.00029,  0.65188],
                          [       0,        0,       0,        0,       0,   0.0001, -0.00001],
                          [ 0.01279,  0.31971,  0.0032,  0.31971, 0.00095, -0.00021,  0.00002],
                          [       0, -0.00007,       0, -0.00007, 0.00001,  0.99455,  0.00058],
                          [ 0.01208,  0.30206, 0.00302,  0.30206, 0.96725,  0.00822, -0.00089],
                          [       0,        0,       0,        0,       0,        0,        0],
                                    ], dtype = 'double')
        self.K_imu_enc_final = np.array([
            [       0,        0,       0,        0,       0,        0],
            [       0,        0,       0,        0,       0,  0.00013],
            [       0,        0,       0,        0,       0,   0.0001],
            [       0,        0,       0,        0,       0,   0.0001],
            [ 0.01279,  0.31971,  0.0032,  0.31971, 0.00095, -0.00021],
            [       0, -0.00007,       0, -0.00007, 0.00001,  0.99453],
            [ 0.01208,  0.30206, 0.00302,  0.30206, 0.96725,  0.00824],
            [       0,        0,       0,        0,       0,        0],
        ], dtype = 'double')

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
                    [   0,   0,     0,   0,       0,     0,   0, 1.0],], dtype = 'double')
        self.B_m = np.array([[0, 0, 0 ,0 ,0 , 2*self.dt, (2* self.dt * self.v_h), 0]], dtype = 'double').T

        # print('Kalman : Searching for calibration file %s/sensors/Acc_Cali.txt' %(os.getcwd()))
        # with open('./sensors/Acc_Cali.txt', 'r') as f:
        #     self.acc_roll_offset = float(f.read())
        self.prev_klm_time = time.time()

    cpdef estimate(self, controllerOBJ, int time_count):
        cdef double start_time, X_h ,Y_h ,v_h ,psi_h ,phi_h ,delta_h ,phidot_h ,delta0_h, deltadot_ref, Yk_rps, Yk_enc,
        cdef double[:]  Yk_imu,
        cdef double v_rps, delta_enc, lat_gps, lon_gps, vel_gps, heading_gps
        cdef double gx_imu, gy_imu, gz_imu, ax_imu, ay_imu, az_imu, Centrip, phi_dot_sensor
        cdef double x_GPS_NED, y_GPS_NED, x_GPS_glb, y_GPS_glb, dX_global, dY_global, dX_local , dY_local
        cdef double dv, time_now, dt
        cdef bint update_rps, update_enc, update_gps, update_imu
        cdef double[:] Xk, roll_virtual_sensor, Yk_gps, sensor_out, hx, Xk_f, f_m, Xkp
        cdef double[:, :] C_obsv, K
        # cdef list  , #Xk,
        cdef dict UpdateFlg, Yks,
        cdef int ind




        (UpdateFlg, Yks) = self.data.update_check(controllerOBJ)
        self.Uk = controllerOBJ.steering_rate
        self.dt = max([controllerOBJ.loop_time, 0.01])

        start_time = time.time()


        # (self.X_est, self.P_est) = filter(self.params, UpdateFlg, dt, self.model, self.X_est, self.P_est, Uk, self.sensors, Yks)
        # Xk = self.X_est
        Xk = self.X_pred
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
        # print(Yk_gps[3])
        if isnan(Yk_gps[3]):
            print('Yk3 is NAN!')
            print(Yk_gps[3])
            Yk_gps[3] = psi_h
            print('Overwritten as %f' %phi_h)
        # else:
            # print(unwrap([psi_h, Yk_gps[3]]))
            # Yk_gps[3] = unwrap([psi_h, Yk_gps[3]])[1]
            # print(Yk_gps[3])
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



        # roll_virtual_sensor = np.zeros(shape=(0, 4), dtype='double')
        roll_virtual_sensor = np.zeros(shape=(4), dtype='double')
        Centrip = v_h ** 2 * tan(delta_h * sin(1.1519)) / self.b_const / self.g_const
        roll_virtual_sensor[0] = atan2(ay_imu - Centrip * cos(phi_h), az_imu + Centrip * sin(phi_h))
        roll_virtual_sensor[1] = -atan(gz_imu * v_h / self.g_const)
        roll_virtual_sensor[2] = np.sign(gz_imu) * asin( gy_imu / sqrt(gy_imu ** 2  + gz_imu ** 2 ))
        roll_virtual_sensor[3] = - v_h ** 2 / (self.g_const * self.b_const) * delta_h
        # print(roll_virtual_sensor)
        # psidot_virtual_sensor[0] = gy_imu * sin(phi_h) + gz_imu * cos(phi_h)
        phi_dot_sensor = gx_imu
        if update_gps:
            x_GPS_NED = self.R_e_const * deg2rad(lon_gps * cos(deg2rad(self.inilat_const))) - self.inix_GPS
            y_GPS_NED = self.R_e_const * deg2rad(lat_gps) - self.iniy_GPS

            x_GPS_glb = x_GPS_NED
            y_GPS_glb = y_GPS_NED

            dX_global = x_GPS_glb - self.Global_X_prev_GPS
            dY_global = y_GPS_glb - self.Global_Y_prev_GPS

            dX_local = dX_global * cos(self.Psi_prev_GPS) + dY_global * sin(self.Psi_prev_GPS)
            dY_local = -dX_global * sin(self.Psi_prev_GPS) + dY_global * cos(self.Psi_prev_GPS)

            # Yk_gps[0:2] = np.array([dX_local, dY_local])
            Yk_gps[0] = dX_local
            Yk_gps[1] = dY_local
            # Yk_gps = Yk_gps.reshape((4,))
            if dX_local > 10:
                print('Warning: dX_Local > 10 m, something is wrong in GPS reading!')


        delta_0_sensor = delta0_h - 0.01 * (Yk_gps[1] - Y_h)

        if update_rps and update_gps:
            sensor_out = np.hstack((roll_virtual_sensor,
                                    phi_dot_sensor,
                                    Yk_enc,
                                    Yk_rps,
                                    Yk_gps,
                                    delta_0_sensor,
                                    )).T.astype('double')
            C_obsv = self.C_rps_gps
        elif not update_rps and update_gps:
            sensor_out = np.hstack((roll_virtual_sensor,
                                    phi_dot_sensor,
                                    Yk_enc,
                                    Yk_gps,
                                    delta_0_sensor,
                                    )).T.astype('double')
            C_obsv = self.C_gps
        elif update_rps and not update_gps:
            sensor_out = np.hstack((roll_virtual_sensor,
                                    phi_dot_sensor,
                                    Yk_enc,
                                    Yk_rps,
                                    )).T.astype('double')
            C_obsv = self.C_rps
        else:
            sensor_out = np.hstack((roll_virtual_sensor, phi_dot_sensor, Yk_enc )).T.astype('double')
            C_obsv = self.C_imuenc

        if update_rps and update_gps:
            K = self.K_rps_gps_final
            # if time_count < 20:
            #     K[5][11] = 0
        elif not update_rps and update_gps:
            K = self.K_gps_final
            # if time_count < 20:
            #     K[5][10] = 0
        elif update_rps and not update_gps:
            K = self.K_rps_final
        else:
            K = self.K_imu_enc_final
        # hx = C_obsv.dot(Xk)
        # Xk = Xk + K.dot((np.asarray(sensor_out) - np.asarray(hx.T)).T)
        hx = np.dot(C_obsv,Xk)
        Xk = Xk + np.dot(K,(np.asarray(sensor_out) - np.asarray(hx.T)).T)
        # Xk[7] = 0
        if abs(Xk[7]) > 0.087: # Satuation at 5 degs
            Xk[7] = np.sign(Xk[7]) * 0.087
        Xk[6] = phi_dot_sensor  # Overwrite the rollrate
        # Nonlinear Prediction
        X_h = Xk[0]
        Y_h = Xk[1]
        v_h = Xk[2]
        psi_h = Xk[3]
        phi_h = Xk[4]
        delta_h = Xk[5]
        phidot_h = Xk[6]
        delta0_h = Xk[7]
        dv = 0
        time_now = time.time()
        dt = min(0.02,time_now - self.prev_klm_time)
        self.prev_klm_time = time_now
        f_m = np.array([X_h + (dt*v_h)/(0.34*tan(0.91*delta_h) ** 2 + 1.0) ** (1/2),
                        Y_h + (0.58*dt*v_h*tan(0.91*delta_h))/(0.34*tan(0.91*delta_h) ** 2 + 1.0) ** (1/2),
                        v_h + dt*dv,
                        psi_h + 0.87*dt*v_h*tan(0.91*delta_h),
                        phi_h + dt*phidot_h ,delta_h + 2.0*dt*self.Uk,
                        phidot_h + dt*(18.0*phi_h + 0.99*self.Uk*v_h + delta_h*(1.5*v_h ** 2 - 1.1)) + 0.99*dt*self.Uk*v_h,
                        delta0_h], dtype = 'double')
        # f_m = self.A_m * Xk + self.B_m * self.Uk
        Xkp = f_m
        Xkp[6] = phi_dot_sensor  # Overwrite the rollrate


        if update_gps:

            Global_X = self.Global_X_prev_GPS + Xk[0] * cos(self.Psi_prev_GPS) - Xk[1] * sin(self.Psi_prev_GPS)
            Global_Y = self.Global_Y_prev_GPS + Xk[0] * sin(self.Psi_prev_GPS) + Xk[1] * cos(self.Psi_prev_GPS)

            # Xkp[0:2] = np.asarray(Xkp[0:2]) - np.asarray(Xk[0:2])
            Xkp[0] = Xkp[0]-Xk[0]
            Xkp[1] = Xkp[1]-Xk[1]

            # Xk[0:2] = 0
            Xk[0] = 0
            Xk[1] = 0
            self.Global_X_prev_GPS = Global_X
            self.Global_Y_prev_GPS = Global_Y
            self.Psi_prev_GPS = Xk[3]
        else:

            Global_X = self.Global_X_prev_GPS + Xk[0] * cos(self.Psi_prev_GPS) - Xk[1] * sin(self.Psi_prev_GPS)
            Global_Y = self.Global_Y_prev_GPS + Xk[0] * sin(self.Psi_prev_GPS) + Xk[1] * cos(self.Psi_prev_GPS)


        Xk_f = Xk
        # Xk_f[0:2] = np.array([Global_X, Global_Y], dtype='double')
        Xk_f[0] = Global_X
        Xk_f[1] = Global_Y

        for ind in range(0,8):
            # if abs(Xk[ind]) < 1e-5:
            if isnan(Xk[ind]):
                Xk[ind] = 0.0

        if sum(np.isnan(Xk_f)):
            print(Yks)

        self.X_est = Xk

        self.X_pred = Xkp

        # print(self.X_est[0:2])

        # print('Kalman Time : \n')
        # print(time.time() - start_time)



        return Xk_f
