import binascii
import Adafruit_GPIO.I2C as I2C
import struct
import time

BUS = 1               # I2C bus to use on the BeagleBone Blue
I2C_ADDR = 0X6B          # I2C address of the IMU

ADDR_WHO_AM_I  = 0X0F # I2C register address for the WHO_AM_I of the IMU
IDENTITY = 0X68       # Expected value of the WHO_AM_I of the IMU

ACCEL_CONFIG = 0X1F   # I2C register address for the accelerometer configuration
GYRO_CONFIG = 0X10    # I2C register address for the gyroscope configuration

ADDR_TEMP_OUT_L =   0X15 # I2C register address for the lower 8 bits of the temperature measurement
ADDR_TEMP_OUT_H =   0X16 # I2C register address for the higher 8 bits of the temperature measurement

ADDR_ACCEL_XOUT_L = 0X28 # I2C register address for the lower 8 bits of the x-axis acceleration measurement
ADDR_ACCEL_XOUT_H = 0X29 # I2C register address for the higher 8 bits of the x-axis acceleration measurement
ADDR_ACCEL_YOUT_L = 0X2A # I2C register address for the lower 8 bits of the y-axis acceleration measurement
ADDR_ACCEL_YOUT_H = 0X2B # I2C register address for the higher 8 bits of the y-axis acceleration measurement
ADDR_ACCEL_ZOUT_L = 0X2C # I2C register address for the lower 8 bits of the z-axis acceleration measurement
ADDR_ACCEL_ZOUT_H = 0X2D # I2C register address for the higher 8 bits of the z-axis acceleration measurement

ADDR_GYRO_XOUT_L =  0X18 # I2C register address for the lower 8 bits of the x-axis angular velocity measurement
ADDR_GYRO_XOUT_H =  0X19 # I2C register address for the higher 8 bits of the x-axis angular velocity measurement
ADDR_GYRO_YOUT_L =  0X1A # I2C register address for the lower 8 bits of the y-axis angular velocity measurement
ADDR_GYRO_YOUT_H =  0X1B # I2C register address for the higher 8 bits of the y-axis angular velocity measurement
ADDR_GYRO_ZOUT_L =  0X1C # I2C register address for the lower 8 bits of the z-axis angular velocity measurement
ADDR_GYRO_ZOUT_H =  0X1D # I2C register address for the higher 8 bits of the z-axis angular velocity measurement


class IMU(object):

    def __init__(self):
        self.i2c = I2C.Device(I2C_ADDR, BUS)

        # Check that WHO_AM_I matches the expected value
        who_am_i = self.i2c.readList(ADDR_WHO_AM_I, 1) # read the WHO_AM_I register
        if hex(who_am_i[0]) != str(hex(IDENTITY)):
            raise ValueError('IMU NOT IDENTIFIED! the read WHO_AM_I (' + str(who_am_i) + ') does not match the expected one (' + str(IDENTITY) + ')')

        self.i2c.write8(ACCEL_CONFIG, 0b00111000) # Set accelerometer sensitivity to +/- 2g
        self.i2c.write8(GYRO_CONFIG, 0b11100000)  # Set gyroscope sensitivity to +/- 245 dps

        self.temp_sensitivity = 16.0            # from LSM9DS1's datasheet
        self.acc_sensitivity = 4 * 1000 / 65536 # 4000mg (+/- 2g) range on 2 bytes (2^16 = 65536)
        self.gyro_sensitivity = 0.00875 / 1000  # from LSM9DS1's datasheet

    def get_imu_data(self):
        # Get data from the I2C transmission
        imu_data = (struct.unpack('<hhhhhhh', (self.i2c.readList(ADDR_TEMP_OUT_L, 14))))
        
        # Process IMU data
        temp = 25 + imu_data[0] / self.temp_sensitivity

        gx = imu_data[1] * self.gyro_sensitivity
        gy = imu_data[2] * self.gyro_sensitivity
        gz = imu_data[3] * self.gyro_sensitivity

        accx = imu_data[4] * self.acc_sensitivity
        accy = imu_data[5] * self.acc_sensitivity
        accz = imu_data[6] * self.acc_sensitivity

        return [temp, accx, accy, accz, gx, gy, gz]