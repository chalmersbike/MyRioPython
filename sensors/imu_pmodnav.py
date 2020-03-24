#!/usr/bin/python
# -*- coding: UTF-8 -*-
import time
import spidev
import math
import warnings
import os
# from magum import Magum

import time

rad2deg = 180 / 3.1415
deg2rad = 3.1415 / 180
h = 0.5195  # IMU height
CALIBRATION_SAMPLES = 500
COMPLEMENTARY_RATIO = 0.05
# User-defined variables
# Gyro
gyro_range = 245  # [245 , 500 , 2000] deg/s
gyro_odr = 3  # [0-6]
# 0 = gyro off    3 = 119Hz    6 = 952Hz
# 1 = 14.9Hz      4 = 238Hz
# 2 = 59.5Hz      5 = 476Hz

# Accel
accel_range = 2  # [2 , 4 , 8 , 16] g
accel_odr = 3  # [1 - 6]
# 0 = accel off   3 = 119Hz    6 = 952Hz
# 1 = 10Hz        4 = 238Hz
# 2 = 50Hz        5 = 476Hz


# Define IMU constants
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


class IMU(object):

    def __init__(self, horizontal=False):
        self.spi = spidev.SpiDev()
        self.spi.open(1, 1)
        self.spi.max_speed_hz = 8000000
        # Get gyro seetings
        self.gyro_sensitivity = SENSITIVITY_GYROSCOPE_SWITCHER.get(gyro_range, 0.00875)
        print 'Gyro range : %d deg/s ; Gyro sensitivity : %f (deg/s)/LSB ; Gyro ODR : %.1f Hz' % (
            gyro_range, self.gyro_sensitivity, ODR_GYROSCOPE_SWITCHER.get(gyro_odr, 0))
        # Get accel setting
        self.accel_sensitivity = SENSITIVITY_ACCELEROMETER_SWITCHER.get(accel_range, 0.000061)
        print 'Accel range : %d g ; Accel sensitivity : %f g/LSB ; Accel ODR : %.1f Hz' % (
            accel_range, self.accel_sensitivity, ODR_ACCELEROMETER_SWITCHER.get(accel_odr, 0))

        # Read WHO_AM_I
        who_am_i = self.spi.xfer2([0x80 | WHO_AM_I_RG, 0x00])
        who_am_i = who_am_i[1]
        if who_am_i == WHO_AM_I:
            print 'WHO_AM_I correctly read !'
        else:
            #warnings.warn('WHO_AM_I incorrect !')
            print 'WHO_AM_I incorrect !'
            print who_am_i
            raw_input('SPI failure?')

        # Initialize gyro
        gyro_init = 0b00000000
        gyro_init = gyro_init | (gyro_odr << 5)
        gyro_init = gyro_init | (SCALE_GYROSCOPE_SWITCHER.get(gyro_range, 0b00) << 3)
        self.spi.xfer2([CTRL_REG1_G, gyro_init])
        # Verify gyro initialization
        buf = self.spi.xfer2([0x80 | CTRL_REG1_G, 0x00])
        if buf[1] == gyro_init:
            print 'Gyro correctly initialized !'
        else:
            #warnings.warn('Gyro incorrectly initialized !')
            print 'Gyro incorrectly initialized !'
            print buf[1]
            raw_input('SPI failure?')

        # Initialize accel
        accel_init = 0b00000000
        accel_init = accel_init | (accel_odr << 5)
        accel_init = accel_init | (SCALE_ACCELEROMETER_SWITCHER.get(accel_range, 0b00) << 3)
        self.spi.xfer2([CTRL_REG6_XL, accel_init])
        # Verify accel initialization
        buf = self.spi.xfer2([0x80 | CTRL_REG6_XL, 0x00])
        if buf[1] == accel_init:
            print 'Accel correctly initialized !'
        else:
            print 'Accel incorrectly initialized !'
        self.gx_ofst = 0.0
        self.gy_ofst = 0.0
        self.gz_ofst = 0.0
        self.acc_roll_ofst = 0.0

        self.calibrate(horizontal)
        read = self.get_reading()
        ax = read[1]
        ay = read[2]
        az = read[3]
        self.phi = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)) - self.acc_roll_ofst
        self.phi_gyro = self.phi
        self.last_read = time.time()

    def calibrate(self, horizontal=False, pitch_horizontal=False):
        gx_ofst = 0.0
        gy_ofst = 0.0
        gz_ofst = 0.0
        ax_ofst = 0.0
        ay_ofst = 0.0
        az_ofst = 0.0
        # More samples for accelermeter calibration
        if horizontal is True:
            calibration_samples = CALIBRATION_SAMPLES * 4
        else:
            calibration_samples = CALIBRATION_SAMPLES

        for i in range(1, calibration_samples + 1):
            print 'waiting for Calibration'
            read = self.get_reading()
            ang_vel = read[4:7]
            acc = read[1:4]

            gx_ofst += ang_vel[0] / calibration_samples
            gy_ofst += ang_vel[1] / calibration_samples
            gz_ofst += ang_vel[2] / calibration_samples
            ax_ofst += acc[0] / calibration_samples
            ay_ofst += acc[1] / calibration_samples
            az_ofst += acc[2] / calibration_samples
            time.sleep(0.01)
        self.gx_ofst = gx_ofst
        self.gy_ofst = gy_ofst
        self.gz_ofst = gz_ofst
        print "Gyroscope Calibration finished"

    def get_roll_angle(self):
        read = self.get_reading()
        dT = time.time() - self.last_read
        self.last_read = time.time()
        gx = read[4]
        ay = read[2]
        az = read[3]
        self.phi_acc = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)) - self.acc_roll_ofst
        self.phi = self.phi_acc * COMPLEMENTARY_RATIO + (self.phi + gx * (dT)) * (1 - COMPLEMENTARY_RATIO)
        self.phi_gyro += dT * gx

    def get_roll_angular_velocity(self):
        read = self.get_reading()
        return read[4], read[5], read[6]

    def get_imu_data(self):
        read = self.get_reading()
        dT = time.time() - self.last_read
        self.last_read = time.time()
        ax = read[1]
        ay = read[2]
        az = read[3]
        gx = read[4]
        gy = read[5]
        gz = read[6]
        self.phi_acc = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)) - self.acc_roll_ofst
        self.phi = self.phi_acc * COMPLEMENTARY_RATIO + (self.phi + gx * (dT)) * (1 - COMPLEMENTARY_RATIO)
        self.phi_gyro += dT * gx

        # return [phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z, phi_int]
        return self.phi, self.phi, gx, ax, ay, ay, az, self.phi_gyro

    def get_reading(self):
        # Read data
        # for i in range(0,255):
        for i in range(0, 20):
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
            # the PmodNav is left-axised. the IMU x,y heading has been invertedly installed
            gx = -gx

            gx *= self.gyro_sensitivity * deg2rad
            gy *= self.gyro_sensitivity * deg2rad
            gz *= self.gyro_sensitivity * deg2rad
            gx -= self.gx_ofst
            gy -= self.gy_ofst
            gz -= self.gz_ofst
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
            # the PmodNav is left-axised. the IMU x,y heading has been invertedly installed
            ax = -ax
            ax *= self.accel_sensitivity
            ay *= self.accel_sensitivity
            az *= self.accel_sensitivity

            # Print data
            # print 'i = %d ; t = %f ; ax = %f ; ay = %f ; az = %f ; gx = %f ; gy = %f ; gz = %f' % (
            # i, time.time() - time_loop, ax, ay, az, gx, gy, gz)

            return time.time() - time_loop, ax, ay, az, gx, gy, gz

