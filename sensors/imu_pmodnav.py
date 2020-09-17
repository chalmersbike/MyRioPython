#!/usr/bin/python
# -*- coding: UTF-8 -*-
from param import *
import time
import spidev
import math
import warnings
import os
import time


########################################################################################################################
########################################################################################################################
# IMU registers and acceptable values
# DO NOT MODIFY
WHO_AM_I_RG = 0x0F
WHO_AM_I = 0x68
CTRL_REG1_G = 0x10
CTRL_REG6_XL = 0x20
OUT_X_L_G = 0x18
OUT_X_L_XL = 0x28
SCALE_GYROSCOPE_SWITCHER = {
    245: 0b00,
    500: 0b01,
    1000: 0b11,
}
SCALE_ACCELEROMETER_SWITCHER = {
    2: 0b00,
    4: 0b01,
    8: 0b10,
    16: 0b11,
}
SENSITIVITY_GYROSCOPE_SWITCHER = {
    245: 0.00875,
    500: 0.0175,
    1000: 0.07,
}
SENSITIVITY_ACCELEROMETER_SWITCHER = {
    2: 0.000061,
    4: 0.000122,
    8: 0.000244,
    16: 0.000732,
}
ODR_GYROSCOPE_SWITCHER = {
    0: 0,
    1: 14.9,
    2: 59.5,
    3: 119,
    4: 238,
    5: 476,
    6: 952,
}
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


class IMU(object):
    def __init__(self, horizontal=False):
        self.spi = spidev.SpiDev()
        self.spi.open(1, 0)
        #self.spi.max_speed_hz = 8000000
        # Get gyro seetings
        self.gyro_sensitivity = SENSITIVITY_GYROSCOPE_SWITCHER.get(imu_gyroscopeRange, 0.00875)
        print('IMU : Gyro range : %d deg/s ; Gyro sensitivity : %f (deg/s)/LSB ; Gyro ODR : %.1f Hz' % (
            imu_gyroscopeRange, self.gyro_sensitivity, ODR_GYROSCOPE_SWITCHER.get(imu_gyroscopeODR, 0)))
        # Get accel setting
        self.accel_sensitivity = SENSITIVITY_ACCELEROMETER_SWITCHER.get(imu_accelerometerRange, 0.000061)
        print('IMU : Accel range : %d g ; Accel sensitivity : %f g/LSB ; Accel ODR : %.1f Hz' % (
            imu_accelerometerRange, self.accel_sensitivity, ODR_ACCELEROMETER_SWITCHER.get(imu_accelerometerODR, 0)))

        # Read WHO_AM_I
        who_am_i = self.spi.xfer2([0x80 | WHO_AM_I_RG, 0x00])
        who_am_i = who_am_i[1]
        if who_am_i == WHO_AM_I:
            if debug:
                print('IMU : WHO_AM_I correctly read !')
        else:
            #warnings.warn('IMU : WHO_AM_I incorrect !')
            print('IMU : WHO_AM_I incorrect ! Read %s, expected %s' %(hex(who_am_i),hex(WHO_AM_I)))
            raw_input('IMU : SPI failure?')

        # Initialize gyro
        gyro_init = 0b00000000
        gyro_init = gyro_init | (imu_gyroscopeODR << 5)
        gyro_init = gyro_init | (SCALE_GYROSCOPE_SWITCHER.get(imu_gyroscopeRange, 0b00) << 3)
        self.spi.xfer2([CTRL_REG1_G, gyro_init])
        # Verify gyro initialization
        buf = self.spi.xfer2([0x80 | CTRL_REG1_G, 0x00])
        if buf[1] == gyro_init:
            if debug:
                print('IMU : Gyro correctly initialized !')
        else:
            print('IMU : Gyro incorrectly initialized !')
            # print buf[1]
            # raw_input('IMU : SPI failure?')

        # Initialize accel
        accel_init = 0b00000000
        accel_init = accel_init | (imu_accelerometerODR << 5)
        accel_init = accel_init | (SCALE_ACCELEROMETER_SWITCHER.get(imu_accelerometerRange, 0b00) << 3)
        self.spi.xfer2([CTRL_REG6_XL, accel_init])
        # Verify accel initialization
        buf = self.spi.xfer2([0x80 | CTRL_REG6_XL, 0x00])
        if buf[1] == accel_init:
            if debug:
                print('IMU : Accel correctly initialized !')
        else:
            print('IMU : Accel incorrectly initialized !')
            # print buf[1]
            # raw_input('IMU : SPI failure?')


        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0
        self.acc_roll_offset = 0.0
        self.prev_reading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if not horizontal:
            print('IMU : Searching for calibration file %s/sensors/Acc_Cali.txt' %(os.getcwd()))
            with open('./sensors/Acc_Cali.txt', 'r') as f:
                self.acc_roll_offset = float(f.read())

        self.calibrate(horizontal)
        read = self.get_reading()
        ax = read[1]
        ay = read[2]
        az = read[3]
        self.phi = 0.0
        self.phi_gyro = self.phi
        self.last_read = 0.0
        self.first_read = True
        self.first_read_time = 0.0


    def calibrate(self, horizontal=False, pitch_horizontal=False):
        gx_offset = 0.0
        gy_offset = 0.0
        gz_offset = 0.0
        ax_offset = 0.0
        ay_offset = 0.0
        az_offset = 0.0

        # More samples for accelerometer calibration
        if horizontal is True:
            calibration_samples = imu_calibrationSamples * 4
        else:
            calibration_samples = imu_calibrationSamples

        # Gyroscope calibration
        print('IMU : Waiting for calibration data')
        for i in range(1, calibration_samples + 1):
            read = self.get_reading()
            ang_vel = read[4:7]
            acc = read[1:4]

            gx_offset += ang_vel[0] / calibration_samples
            gy_offset += ang_vel[1] / calibration_samples
            gz_offset += ang_vel[2] / calibration_samples
            ax_offset += acc[0] / calibration_samples
            ay_offset += acc[1] / calibration_samples
            az_offset += acc[2] / calibration_samples
            time.sleep(0.01)
        self.gx_offset = gx_offset
        self.gy_offset = gy_offset
        self.gz_offset = gz_offset
        print("IMU : Gyroscope calibration finished")

        # Accelerometer calibration
        if horizontal:
            self.acc_roll_offset = math.atan2(ay_offset, math.sqrt(ax_offset**2 + az_offset**2))
            print('Accelerometer calibrated, new roll angle offset is %.5f\n' %(self.acc_roll_offset))

            if pitch_horizontal is True:
                self.acc_pitch_offset = math.atan2(ax_offset, math.sqrt(ay_offset**2 + az_offset**2))

            with open('./sensors/Acc_Cali.txt', 'w') as f:
                f.write(str(self.acc_roll_offset))
        else:
            print('IMU : Accelerometer not calibrated, using %.5f as roll angle offset\n' %(self.acc_roll_offset))

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

        # if gx > 20 * deg2rad:
        #     print('WARNING : Measured roll rate larger than 20deg/s, at %g deg/s' % ( self.rollRate * rad2deg))
        #     gx = self.prev_reading[2]


        CP_acc_g = ((velocity ** 2) / b) * math.tan(delta_state * 0.94) * (1 / 9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
        self.phi_acc = math.atan2(ay - CP_acc_g * math.cos(phi), az + CP_acc_g * math.sin(phi)) - self.acc_roll_offset  # Making the signs consistent with mathematic model, counterclockwise positive, rear to front view
        # self.phi_acc = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)) - self.acc_roll_offset

        if abs(self.phi_acc) >  80 * deg2rad:
            print('WARNING : Spike in accelerometer detected, calculated acc roll is  %g deg. \n using previous reading' % (self.phi_acc * rad2deg))
            self.phi_acc = self.prev_reading[0]


        self.phi = self.phi_acc * (1-imu_complementaryFilterRatio) + (self.phi + gx * (dT)) * imu_complementaryFilterRatio
        self.phi_gyro += dT * gx
        self.prev_reading = [self.phi, self.phi_gyro, gx, gy, gz, ax, ay, az]
        # [phi_comp, phi_gyro, gx (phidot), gy, gz, ax, ay, az]
        return [self.phi, self.phi_gyro, gx, gy, gz, ax, ay, az, self.last_read - self.first_read_time]

    def get_reading(self):
        # Read data
        # for i in range(0,255):
        # for i in range(0, 20):
        # Get time
        time_loop = time.time()

        # Read gyro to buffer
        buf = self.spi.xfer2([0x80 | OUT_X_L_G, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
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

        gx *= self.gyro_sensitivity * deg2rad
        gy *= self.gyro_sensitivity * deg2rad
        gz *= self.gyro_sensitivity * deg2rad
        gx -= self.gx_offset
        gy -= self.gy_offset
        gz -= self.gz_offset

        # Read accel to buffer
        buf = self.spi.xfer2([0x80 | OUT_X_L_XL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
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

        # Print data
        # print 'i = %d ; t = %f ; ax = %f ; ay = %f ; az = %f ; gx = %f ; gy = %f ; gz = %f' % (
        # i, time.time() - time_loop, ax, ay, az, gx, gy, gz)

        return time.time() - time_loop, ax, ay, az, gx, gy, gz