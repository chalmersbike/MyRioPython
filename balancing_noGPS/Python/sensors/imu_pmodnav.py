#!/usr/bin/python
# -*- coding: UTF-8 -*-
from param import *
import time
import math
import warnings
import os
import time
import numpy as np
from numpy import sin, cos, arctan2
# PmodNAV is of LEFT handed Frame, sadly..... Therefore only Y needs to be inverted.
# When standing still, the sensor prints an upright acceleration!
########################################################################################################################
########################################################################################################################
# IMU registers and acceptable values
# DO NOT MODIFY
print('START OF PROGRAM')
WHO_AM_I_RG = 0x0F
WHO_AM_I = 0x68
CTRL_REG1_G = 0x10
CTRL_REG6_XL = 0x20
OUT_X_L_G = 0x18
OUT_X_L_XL = 0x28

print('START GYRO')
SCALE_GYROSCOPE_SWITCHER = {
    245: 0b00,
    500: 0b01,
    1000: 0b11,
}
print('START ACCELERO')
SCALE_ACCELEROMETER_SWITCHER = {
    2: 0b00,
    4: 0b01,
    8: 0b10,
    16: 0b11,
}
print('START OF SENS GYRO')
SENSITIVITY_GYROSCOPE_SWITCHER = {
    245: 0.00875,
    500: 0.0175,
    1000: 0.07,
}
print('START OF SENS ACCELO')
SENSITIVITY_ACCELEROMETER_SWITCHER = {
    2: 0.000061,
    4: 0.000122,
    8: 0.000244,
    16: 0.000732,
}
print('START OF ODR GYRO')
ODR_GYROSCOPE_SWITCHER = {
    0: 0,
    1: 14.9,
    2: 59.5,
    3: 119,
    4: 238,
    5: 476,
    6: 952,
}
print('START OF ACCELERO SWITCH')
ODR_ACCELEROMETER_SWITCHER = {
    0: 0,
    1: 10,
    2: 50,
    3: 119,
    4: 238,
    5: 476,
    6: 952,
}
########################################################################################################################
########################################################################################################################

print('START OF CLASS IMU')
class IMU(object):
    print('PRE INIT')
    def __init__(self,session, horizontal=False, Kalman = False, pitch_horizontal=False):
        print('START OF START OF INIT')
        self.fpga_IMUGyroData = session.registers['SPI Read Gyro Data'] #accesing controls and indicators
        self.fpga_IMUAccData = session.registers['SPI Read Acc Data']#accesing controls and indicators
        print('START OF FPGA SENSOR REGISTERY')
        self.gyro_sensitivity = SENSITIVITY_GYROSCOPE_SWITCHER.get(imu_gyroscopeRange, 0.00875)
        self.accel_sensitivity = SENSITIVITY_ACCELEROMETER_SWITCHER.get(imu_accelerometerRange, 0.000061)

        self.gx_prev = 0.0
        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0
        self.acc_roll_offset = 0.0
        self.acc_pitch_offset = 0.0
        self.prev_reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.calibrate(horizontal, pitch_horizontal)
        if not horizontal:
            print('IMU : Searching for calibration file %s/sensors/Acc_Cali.txt' %(os.getcwd()))
            with open('./sensors/Acc_Cali.txt', 'r') as f:
                self.acc_roll_offset = float(f.readline())
                self.acc_pitch_offset = float(f.readline())
                print('The Roll Angle Offset in X axis is:')
                print(self.acc_roll_offset)
                print('The Pitch Angle Offset in Y axis is:')
                print(self.acc_pitch_offset)
        read = self.get_reading()
        ax = read[1]
        ay = read[2]
        az = read[3]
        self.phi = 0.0
        self.phi_gyro = self.phi
        self.last_read = 0.0
        self.first_read = True
        self.first_read_time = 0.0
        self.Kalman_switch = False
        if Kalman:
            self.X_k = np.zeros((2,1))
            self.P_k = np.zeros((2,2))
            self.Sigma_P = np.diag([5e-7, 1e-8])
            self.Sigma_S = 0.0025
            self.W_Rollest = 0.25
            self.Phi_Bar_factorSQRE = 0.15
            self.Kalman_switch = True

    def calibrate(self, horizontal=False, pitch_horizontal=False):
        gx_offset = 0.0
        gy_offset = 0.0
        gz_offset = 0.0
        ax_offset = 0.0
        ay_offset = 0.0
        az_offset = 0.0

        # More samples for accelerometer calibration
        if horizontal is not False:
            calibration_samples = imu_calibrationSamples * 4
        else:
            calibration_samples = imu_calibrationSamples

        # Gyroscope calibration
        print('IMU : Waiting for calibration data')
        for i in range(1, calibration_samples + 1):
            read = self.get_reading()
            ang_vel = read[4:7]
            acc = read[1:4]
            # print(acc)  # Print out the readings
            gx_offset += ang_vel[0] / calibration_samples
            gy_offset += ang_vel[1] / calibration_samples
            gz_offset += ang_vel[2] / calibration_samples
            ax_offset += acc[0] / calibration_samples
            ay_offset += acc[1] / calibration_samples
            az_offset += acc[2] / calibration_samples
            time.sleep(0.01)
            if abs(acc[2]) < 0.8:
                print(['acc Z is TOO LOW at %.3f' %acc[2]]) # Print the wrong reading
            # if abs(ang_vel[0]) < 0.001:
            #     print(['Gyro X is TOO LOW at %.3f' % ang_vel[0]])  # Print the wrong reading
        self.gx_offset = gx_offset
        self.gy_offset = gy_offset
        self.gz_offset = gz_offset
        print("IMU : Gyroscope calibration finished")
        print(horizontal)
        print(pitch_horizontal)
        # Accelerometer calibration
        if horizontal is True:
            self.acc_roll_offset = math.atan2(ay_offset, math.sqrt(ax_offset**2 + az_offset**2))
            print('Accelerometer calibrated, new roll angle offset is %.5f\n' %(self.acc_roll_offset))

            if pitch_horizontal is True:
                self.acc_pitch_offset = math.atan2(ax_offset, math.sqrt(ay_offset**2 + az_offset**2))
        elif horizontal == -8 and pitch_horizontal == -7:
            self.acc_pitch_offset = arctan2(-ax_offset,az_offset)
            self.acc_roll_offset = math.atan2(ay_offset, (-ax_offset / sin(self.acc_pitch_offset) + az_offset / cos(self.acc_pitch_offset) )/2 )
            self.acc_roll_offset -= np.deg2rad(horizontal)

            print(self.acc_roll_offset)
            print(self.acc_pitch_offset)
            with open('./sensors/Acc_Cali.txt', 'w') as f:
                f.write(str(self.acc_roll_offset))
                f.write('\r\n')
                f.write(str(self.acc_pitch_offset))
        # else:
        #     print('IMU : Accelerometer not calibrated, using %.5f as roll angle offset\n' %(self.acc_roll_offset))


    def get_imu_data(self, velocity, delta_state, phi):
        read = self.get_reading()
        if self.first_read is True:
            dT = 0.01
            self.first_read = False
            self.first_read_time = time.time()
            self.last_read = time.time()
        else:
            dT = time.time() - self.last_read
            # dT = 0.01
            self.last_read = time.time()

        ax = read[1]
        ay = read[2]
        az = read[3]
        gx = read[4]
        gy = read[5]
        gz = read[6]
        if az < 0.8:
            print(['acc Z is TOO LOW at %.4f' %az]) # Print the wrong reading
        # if abs(gx) < 0.0005:
        #     print(['Gyro X is TOO LOW at %.4f' %gx]) # Print the wrong reading
        # if gx > 20 * deg2rad:
        #     print('WARNING : Measured roll rate larger than 20deg/s, at %g deg/s' % ( self.rollRate * rad2deg))
        #     gx = self.prev_reading[2]


        # # The switch approach needs to be checked carefully.
        # Estimators = {
        #     0 : self.complementary_filter(dT, gx, ay, az, phi, delta_state, velocity),
        #     1 : self.kalman_schwab(dT, gx, gy, gz, velocity),
        #     2 : self.constant_coef_kalman_schwab(dT, gx, gy, gz, velocity, 0.99),
        #     3 : self.complementary_filter_gz_acc(dT, gx, gz, ay, az, delta_state, self.phi, velocity, 0.99),
        #     4 : self.gyro_xz(dT, gx, gz, velocity, phi)
        # }
        #
        # self.phi = Estimators.get(Estimator_option, None)
        self.phi = self.complementary_filter(dT, gx, ay, az, phi, delta_state, velocity, gz)
        # self.phi = self.complementary_filter_gz_acc(dT, gx, gz, ay, az, delta_state, self.phi, velocity, 0.99)
        self.phi_gyro += dT * gx
        self.prev_reading = [self.phi, self.phi_gyro, gx, gy, gz, ax, ay, az]
        self.gx_prev = gx
        # print(read) # Printout all readings
        # [phi_comp, phi_gyro, gx (phidot), gy, gz, ax, ay, az]


        return [self.phi, self.phi_gyro, gx, gy, gz, ax, ay, az, self.last_read - self.first_read_time]

    def get_reading(self):
        # Read data
        # for i in range(0,255):
        # for i in range(0, 20):
        # Get time
        time_loop = time.time()

        # Read gyro to buffer
        buf = self.fpga_IMUGyroData.read()
        buf = buf[:7]
        # Calculate gyro values from buffer
        gx = (buf[2] << 8) | buf[1]
        gy = (buf[4] << 8) | buf[3]
        gz = (buf[6] << 8) | buf[5]
        # Apply two's complement
        if (gx & (1 << (16 - 1))) != 0:
            gx -= (1 << 16)
        if (gy & (1 << (16 - 1))) != 0:
            gy -= (1 << 16)
        if (gz & (1 << (16 - 1))) != 0:
            gz -= (1 << 16)
        # the PmodNav has y pointing to the right of x (left axised) but the bike has y pointing to the left of x (right axised)
        gy  = -gy
        # gz  = -gz # PmodNAV is of LEFT handed Frame, sadly..... Therefore only Y needs to be inverted.
        gx *= self.gyro_sensitivity * deg2rad
        gy *= self.gyro_sensitivity * deg2rad
        gz *= self.gyro_sensitivity * deg2rad
        gx -= self.gx_offset
        gy -= self.gy_offset
        gz -= self.gz_offset

        # gx *= -cos(self.acc_roll_offset) * sin(self.acc_pitch_offset)
        # gy *= sin(self.acc_roll_offset)
        # gz *= cos(self.acc_roll_offset) * cos(self.acc_pitch_offset)
        gx_raw = gx
        gy_raw = gy
        gz_raw = gz
        gx = cos(self.acc_pitch_offset) * gx_raw+ sin(self.acc_pitch_offset) * gz_raw
        gy = sin(self.acc_roll_offset)*sin(self.acc_pitch_offset) * gx_raw + cos(self.acc_roll_offset) * gy_raw - cos(self.acc_pitch_offset) * sin(self.acc_roll_offset) * gz_raw
        gz = -cos(self.acc_roll_offset)*sin(self.acc_pitch_offset) * gx_raw + sin(self.acc_roll_offset) * gy_raw +  cos(self.acc_roll_offset) * cos(self.acc_pitch_offset) * gz_raw

        # Read accel to buffer
        buf = self.fpga_IMUAccData.read()
        buf = buf[:7]
        # Calculate accel values from buffer
        ax = (buf[2] << 8) | buf[1]
        ay = (buf[4] << 8) | buf[3]
        az = (buf[6] << 8) | buf[5]
        # Apply two's complement
        if (ax & (1 << (16 - 1))) != 0:
            ax -= (1 << 16)
        if (ay & (1 << (16 - 1))) != 0:
            ay -= (1 << 16)
        if (az & (1 << (16 - 1))) != 0:
            az -= (1 << 16)
        # the PmodNav has y pointing to the right of x (left axised) but the bike has y pointing to the left of x (right axised)
        ay = -ay

        ax *= self.accel_sensitivity
        ay *= self.accel_sensitivity
        az *= self.accel_sensitivity

        ax_raw = ax
        ay_raw = ay
        az_raw = az
        ax = cos(self.acc_pitch_offset) * ax_raw+ sin(self.acc_pitch_offset) * az_raw
        ay = sin(self.acc_roll_offset)*sin(self.acc_pitch_offset) * ax_raw + cos(self.acc_roll_offset) * ay_raw - cos(self.acc_pitch_offset) * sin(self.acc_roll_offset) * az_raw
        az = -cos(self.acc_roll_offset)*sin(self.acc_pitch_offset) * ax_raw + sin(self.acc_roll_offset) * ay_raw +  cos(self.acc_roll_offset) * cos(self.acc_pitch_offset) * az_raw

        # Print data
        # print 'i = %d ; t = %f ; ax = %f ; ay = %f ; az = %f ; gx = %f ; gy = %f ; gz = %f' % (
        # i, time.time() - time_loop, ax, ay, az, gx, gy, gz)
        #print('END OF PROGRAM')
        return time.time() - time_loop, ax, ay, az, gx, gy, gz

    def complementary_filter(self, dT, gx, ay, az, phi, delta_state, velocity, gz):
        # CP_acc_g = ((velocity ** 2) / LENGTH_B) * math.tan(delta_state * 0.94) * (1 / 9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
        CP_acc_g = (velocity * gz) / 9.81  # Unit (g)
        # dT = self.dT_reading
        # CP_acc_g = 0
        # Roll acceleration
        # phiddot = (gx - self.gx_prev) / dT

        # self.phi_acc = math.atan2(ay - CP_acc_g * math.cos(phi) + imu_height*phiddot, az + CP_acc_g * math.sin(phi)) - self.acc_roll_offset  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        # self.phi_acc = math.atan2(ay - CP_acc_g * math.cos(phi), az + CP_acc_g * math.sin(phi)) - self.acc_roll_offset  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        # self.phi_acc = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)) - self.acc_roll_offset
        self.phi_acc = math.atan2(ay - CP_acc_g * math.cos(phi), az + CP_acc_g * math.sin(phi))  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        
        if abs(self.phi_acc) >  80 * deg2rad:
            print('WARNING : Spike in accelerometer detected, calculated acc roll is  %g deg. \n using previous reading' % (self.phi_acc * rad2deg))
            self.phi_acc = self.prev_reading[0]

        if velocity < 1.0:
            Phi_m = self.phi_acc * (1-imu_complementaryFilterRatio) + (phi + gx * (dT)) * imu_complementaryFilterRatio
        else:
            Phi_gz = -math.atan(gz * velocity / 9.81)
            Phi_m = Phi_gz * (1 - imu_complementaryFilterRatio) + (phi + gx * (dT)) * imu_complementaryFilterRatio
        return Phi_m

    def kalman_schwab(self, dT, gx, gy, gz, velocity):

        Phi_d = np.arctan(-gz * velocity/ 9.81)
        Phi_omg = np.sign(-gz) * np.arcsin(-gy / np.sqrt(gy**2 + gz**2))

        Fk = np.array(([1, -dT], [0, 1]))
        self.X_k = Fk * self.X_k + np.array((dT, 0)) * gx
        self.P_k = Fk * self.P_k * Fk.T + self.Sigma_P
        H = np.array([1, 0])
        ymis_k = Phi_m - H * self.X_k
        S_k = H * self.P_k * H.T + self.Sigma_S
        K_k = self.P_k * H.T * np.linalg.inv(S_k)
        self.X_k = self.X_k + K_k * ymis_k
        self.P_k = (np.eye(2) - K_k*H) * self.P_k
        self.W_Rollest = np.exp(-self.X_k[0,0] ** 2/self.Phi_Bar_factorSQRE)
        Phi_m = self.W_Rollest * Phi_d + (1-self.W_Rollest) * Phi_omg
        return Phi_m
        # print(self.X_k)

    def constant_coef_kalman_schwab(self, dT, gx, gy, gz, velocity, W_Rollest):
        Phi_d = np.arctan(-gz * velocity/ 9.81)
        Phi_omg = np.sign(-gz) * np.arcsin(-gy / np.sqrt(gy**2 + gz**2))
        Phi_m = W_Rollest * Phi_d + (1-W_Rollest) * Phi_omg
        print('Constant Coef Kalman Schwab ran successfully!!!!!')
        return Phi_m
        # print(self.X_k)

    def complementary_filter_gz_acc(self, dT, gx, gz, ay, az, delta_state, phi, velocity, W_Rollest):
        # CP_acc_g = ((velocity ** 2) / LENGTH_B) * math.tan(delta_state * 0.94) * (1 / 9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
        CP_acc_g = ( velocity * gz / 9.81 )  # Use Gyro-Z
        # dT = self.dT_reading
        # CP_acc_g = 0
        # Roll acceleration
        # phiddot = (gx - self.gx_prev) / dT

        # self.phi_acc = math.atan2(ay - CP_acc_g * math.cos(phi) + imu_height*phiddot, az + CP_acc_g * math.sin(phi)) - self.acc_roll_offset  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        # self.phi_acc = math.atan2(ay - CP_acc_g * math.cos(phi), az + CP_acc_g * math.sin(phi)) - self.acc_roll_offset  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        # self.phi_acc = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)) - self.acc_roll_offset
        self.phi_acc = math.atan2(ay - CP_acc_g * math.cos(phi), az + CP_acc_g * math.sin(phi))  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        if abs(self.phi_acc) >  80 * deg2rad:
            print('WARNING : Spike in accelerometer detected, calculated acc roll is  %g deg. \n using previous reading' % (self.phi_acc * rad2deg))
            self.phi_acc = self.prev_reading[0]

        Phi_d = np.arctan(gz * velocity/ 9.81)
        Phi_m = W_Rollest * Phi_d + (1-W_Rollest) * self.phi_acc
        return Phi_m

    def gyro_xz(self, dT, gx, gz, velocity, phi):
        Phi_d = np.arctan(-gz * velocity/ 9.81)
        Phi_gyro = (phi + gx * (dT))
        Phi_m = W_Rollest * Phi_d + (1-W_Rollest) * Phi_gyro
        return Phi_m