cimport numpy as cnp
import numpy as np
from numpy import deg2rad, rad2deg
# from .kalman_param import *
# from kalman_param import *
# from libc.math import abs
from libc.math cimport abs
import cython

@cython.wraparound(False)
@cython.boundscheck(False)
cdef class  KalmanData:

    # cdef cnp.float32_t[:] gx_imu, gy_imu, gz_imu, ax_imu, ay_imu, az_imu
    # cdef cnp.float32_t[:] Beta, Odo_Psi, Odo_X, Odo_Y, lat_gps, lon_gps, vel_gps, heading_gps
    # cdef cnp.float32_t[:] deltadot_ref
    # cdef cnp.float32_t[:] v_rps,
    # cdef cnp.float32_t[:] delta_enc,
    cdef readonly double [:] gx_imu, gy_imu, gz_imu, ax_imu, ay_imu, az_imu
    cdef readonly double [:] Beta, Odo_Psi, Odo_X, Odo_Y, lat_gps, lon_gps, vel_gps, heading_gps
    cdef readonly double [:] deltadot_ref
    cdef readonly double [:] v_rps,
    cdef readonly double [:] delta_enc,
    cdef readonly double alpha_0, gamma_0, last_valid_gx
    cdef double sMag, sMag9, vMA
    cdef list dT_rps
    cdef double gx_threshold


    def __init__(self, double lat_ini, double lon_ini):


        self.Beta = np.zeros(shape=(2,), dtype='double')
        self.Odo_Psi = np.zeros(shape=(2,), dtype='double')
        self.Odo_X = np.zeros(shape=(2,), dtype='double')
        self.Odo_Y = np.zeros(shape=(2,), dtype='double')

        self.deltadot_ref = np.zeros(shape=(2,), dtype='double')
        # Speed Sensor
        self.v_rps = np.zeros(shape=(2,), dtype='double')
        self.dT_rps = [1] * 9
        self.sMag = 0.6
        self.sMag9 = 0.6 * 9
        self.vMA = 0.0

        # Enc
        self.delta_enc = np.zeros(shape=(2,), dtype='double')

        # GPS
        self.lat_gps = np.array([lat_ini, lat_ini], dtype='double')
        self.lon_gps = np.array([lon_ini, lon_ini], dtype='double')
        self.vel_gps = np.zeros(shape=(2,), dtype='double')
        self.heading_gps = np.zeros(shape=(2,), dtype='double')
        # self.last_valid_gps = np.zeros(shape=(2,), dtype='double')
        self.alpha_0 = lat_ini # Degrees
        self.gamma_0 = lon_ini
        # IMU
        self.gx_imu = np.zeros(shape=(2,), dtype='double')
        self.gy_imu = np.zeros(shape=(2,), dtype='double')
        self.gz_imu = np.zeros(shape=(2,), dtype='double')
        self.ax_imu = np.zeros(shape=(2,), dtype='double')
        self.ay_imu = np.zeros(shape=(2,), dtype='double')
        self.az_imu = np.ones(shape=(2,), dtype='double')
        self.last_valid_gx = 0.0
        self.gx_threshold = 20




    cpdef tuple update_check(self, controllerOBJ):
        cdef int IMU_update, ENC_update, RPS_update, GPS_update
        """ Check the validity of latest reading, for IMU, ENC, RPS and GPS """
        # Speed Sensor
        self.v_rps[0] = self.v_rps[1]
        self.v_rps[1] = controllerOBJ.velocity_rec
        # Enc
        self.delta_enc[0] = self.delta_enc[1]
        self.delta_enc[1] = controllerOBJ.steeringAngle
         # GPS
        self.lat_gps[0] = self.lat_gps[1]
        self.lon_gps[0] = self.lon_gps[1]
        self.vel_gps[0] = self.vel_gps[1]
        self.heading_gps[0] = self.heading_gps[1]
        # self.last_valid_gps[0] = self.last_valid_gps[1]
        self.lat_gps[1] = controllerOBJ.lat_measured_GPS
        self.lon_gps[1] = controllerOBJ.lon_measured_GPS
        # self.vel_gps[1] = controllerOBJ.v_estimated
        # self.heading_gps[1] = controllerOBJ.theta_measured_GPS
        self.vel_gps[1] = controllerOBJ.speed_m_s
        # self.heading_gps[1] = controllerOBJ.course
        self.heading_gps[1] = controllerOBJ.theta_measured_GPS
        # print(self.heading_gps[1])
        # self.last_valid_gps[1] =
        # IMU
        self.gx_imu[0] = self.gx_imu[1]
        self.gy_imu[0] = self.gy_imu[1]
        self.gz_imu[0] = self.gz_imu[1]
        self.ax_imu[0] = self.ax_imu[1]
        self.ay_imu[0] = self.ay_imu[1]
        self.az_imu[0] = self.az_imu[1]
        self.gx_imu[1] = controllerOBJ.rollRate_rec
        self.gy_imu[1] = controllerOBJ.gy
        self.gz_imu[1] = controllerOBJ.gz
        self.ax_imu[1] = controllerOBJ.ax
        self.ay_imu[1] = controllerOBJ.ay
        self.az_imu[1] = controllerOBJ.az

        """Update IMU reading"""
        if abs(self.gx_imu[1] - self.last_valid_gx) > self.gx_threshold * deg2rad(1):
            IMU_update = 0
        else:
            IMU_update = 1
            self.last_valid_gx = self.gx_imu[1]

        """ Encoder """
        ENC_update = 1  # Always update encoder reading

        """Check RPS reading """
        if self.differentRead(self.v_rps, 1):
            RPS_update = 1
            if self.v_rps[1] > 0.6:
                self.dT_rps.insert(0, self.sMag / self.v_rps[1])
                self.dT_rps.pop()

            self.vMA = self.sMag9 / sum(self.dT_rps)
        else:
            RPS_update = 0
        # print(self.vMA)

        """Check GPS validity"""
        if ((self.differentRead(self.lat_gps, 1) or
             self.differentRead(self.lon_gps, 1) or
             self.differentRead(self.heading_gps, 1)) and
                (abs(deg2rad( self.lat_gps[1] - self.alpha_0)) < 0.1) and  # No position jump between samples
                (abs(deg2rad( self.lon_gps[1] - self.gamma_0)) < 0.1)):
            GPS_update = 1
        else:
            GPS_update = 0

        return ({'imu': IMU_update, 'enc': ENC_update, 'rps': RPS_update, 'gps': GPS_update}, {
                'imu': np.array([self.ax_imu[1],
                                 self.ay_imu[1],
                                 self.az_imu[1],
                                 self.gx_imu[1],
                                 self.gy_imu[1],
                                 self.gz_imu[1]], dtype = 'double').T,
                'enc': self.delta_enc[1],
                'rps': self.vMA,
                'gps': (np.array([self.lat_gps[1],
                                   self.lon_gps[1],
                                   self.vel_gps[1],
                                   self.heading_gps[1],
                                        ], dtype = 'double').T)
            })

    cdef bint differentRead(self, double[:] var, int ind):
        if ind > 0:
            tol = 1e-7
            return abs(var[ind] - var[ind - 1]) > tol
        else:
            return True
        # tol = 1e-4
        # return abs(var[ind] - var[ind - 1]) < tol

