import sys
sys.path.append(sys.path[0] + '/../../')
import param
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

########################################################################################################################
# IMU
imu_calibrationSamples = 500                # IMU number of calibration samples used at startup
imu_gyroscopeRange = 245                    # [deg/s] IMU range of the gyroscope : 245, 500, 2000
imu_gyroscopeODR = 6                        # IMU Output Data Rate of the gyroscope, follows the table below
                                                # 0 = gyro off    3 = 119Hz    6 = 952Hz
                                                # 1 = 14.9Hz      4 = 238Hz
                                                # 2 = 59.5Hz      5 = 476Hz
imu_accelerometerRange = 2                  # [g] IMU range of the accelerometer : 2, 4, 8, 16
imu_accelerometerODR = 6                    # IMU Output Data Rate of the accelerometer, follows the table below
                                                # 0 = accel off   3 = 119Hz    6 = 952Hz
                                                # 1 = 10Hz        4 = 238Hz
                                                # 2 = 50Hz        5 = 476Hz
########################################################################################################################




try:
    time_count = 0.0

    spi = spidev.SpiDev()
    spi.open(1, 0)
    print(spi.max_speed_hz)
    spi.max_speed_hz = 1000000

    # while True:
    # Read WHO_AM_I
    who_am_i = spi.xfer2([0x80 | WHO_AM_I_RG, 0x00])
    who_am_i = who_am_i[1]
    if who_am_i == WHO_AM_I:
        print('IMU : WHO_AM_I correctly read !')
    else:
        #warnings.warn('IMU : WHO_AM_I incorrect !')
        print('IMU : WHO_AM_I incorrect ! Read %s, expected %s' %(hex(who_am_i),hex(WHO_AM_I)))
        raw_input('IMU : SPI failure?')


    # Initialize gyro
    gyro_init = 0b00000000
    gyro_init = gyro_init | (imu_gyroscopeODR << 5)
    gyro_init = gyro_init | (SCALE_GYROSCOPE_SWITCHER.get(imu_gyroscopeRange, 0b00) << 3)
    print('gyro_init : 0x%02x' % (gyro_init))

    # Initialize accel
    accel_init = 0b00000000
    accel_init = accel_init | (imu_accelerometerODR << 5)
    accel_init = accel_init | (SCALE_ACCELEROMETER_SWITCHER.get(imu_accelerometerRange, 0b00) << 3)
    print('accel_init : 0x%02x' % (accel_init))

    # time.sleep(0.1)

finally:
    print "done"
