import numpy as np
from numpy import deg2rad, rad2deg
from .kalman_param import *

class KalmanData(object):
    def __init__(self, lat_ini, lon_ini):


        self.Beta = [0,0]
        self.Odo_Psi = [0,0]
        self.Odo_X = [0,0]
        self.Odo_Y = [0,0]

        self.deltadot_ref = [0,0]
        # Speed Sensor
        self.v_rps = [0,0]

        # Enc
        self.delta_enc = [0,0]

        # GPS
        self.lat_gps = [lat_ini, lat_ini]
        self.lon_gps = [lon_ini, lon_ini]
        self.vel_gps = [0,0]
        self.heading_gps = [0,0]
        # self.last_valid_gps = [0,0]
        self.alpha_0 = lat_ini # Degrees
        self.gamma_0 = lon_ini
        # IMU
        self.gx_imu = [0,0]
        self.gy_imu = [0,0]
        self.gz_imu = [0,0]
        self.ax_imu = [0,0]
        self.ay_imu = [0,0]
        self.az_imu = [1,1]
        self.last_valid_gx = 0




    def update_check(self, controllerOBJ):
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
        if abs(self.gx_imu[1] - self.last_valid_gx) > gx_threshold * deg2rad(1):
            IMU_update = 0
        else:
            IMU_update = 1
            self.last_valid_gx = self.gx_imu[1]

        """ Encoder """
        ENC_update = 1  # Always update encoder reading

        """Check RPS reading """
        if self.differentRead(self.v_rps, 1):
            RPS_update = 1
        else:
            RPS_update = 0

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
                'imu': np.array([[float(self.ax_imu[1]),
                                 float(self.ay_imu[1]),
                                 float(self.az_imu[1]),
                                 float(self.gx_imu[1]),
                                 float(self.gy_imu[1]),
                                 float(self.gz_imu[1])]]).T,
                'enc': float(self.delta_enc[1]),
                'rps': float(self.v_rps[1]),
                'gps': (np.array([[float(self.lat_gps[1]),
                                   float(self.lon_gps[1]),
                                   float(self.vel_gps[1]),
                                   float(self.heading_gps[1]),
                                        ]]).T)
            })

    def differentRead(self, var, ind):
        if ind > 0:
            tol = 1e-7
            return abs(var[ind] - var[ind - 1]) > tol
        else:
            return True
        # tol = 1e-4
        # return abs(var[ind] - var[ind - 1]) < tol

