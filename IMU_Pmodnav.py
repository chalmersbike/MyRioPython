#!/usr/bin/python
# -*- coding: UTF-8 -*-
BUS = 1
SMPLRT_DIV = 0X19 #陀螺仪采样率典型值为0X07 1000/(1+7)=125HZ
CONFIG = 0X12 # Filter Setting
GYRO_CONFIG = 0X10 #陀螺仪测量范围 0X18 正负2000度
ACCEL_CONFIG = 0X20 #加速度计测量范围 0X18 正负16g
ACCEL_CONFIG2 = 0X1D #加速度计低通滤波器 0x06 5hz
PWR_MGMT_1 = 0X6B#电源管理1 典型值为0x00
PWR_MGMT_3 = 0X12 #电源管理2 典型值为0X00
WHO_AM_I  = 0X0F #器件ID MPU9250默认ID为0X71
IDENTITY = 0X68
USER_CTRL = 0X23 #if 0x40, I2C disabled

# MPU9250_CS        PDout(3) #MPU9250片选信号

I2C_ADDR = 0X6B  #i2c的地址
# ACCEL_XOUT_H = 0X1D
ACCEL_XOUT_H = 0X29  #加速度计输出数据
ACCEL_XOUT_L = 0X28
ACCEL_YOUT_H = 0X2B
ACCEL_YOUT_L = 0X2A
ACCEL_ZOUT_H = 0X2D
ACCEL_ZOUT_L = 0X2C
TEMP_OUT_L =   0X15  #温度计输出数据
TEMP_OUT_H =   0X16
GYRO_XOUT_H =  0X19  #陀螺仪输出数据
GYRO_XOUT_L =  0X18
GYRO_YOUT_H =  0X1B
GYRO_YOUT_L =  0X1A
GYRO_ZOUT_H =  0X1D
GYRO_ZOUT_L =  0X1C

import binascii
import Adafruit_GPIO.I2C as I2C
import time
import struct
i2c = I2C.Device(I2C_ADDR, BUS)

# i2c.get_i2c_device(0x68)
bytes = i2c.readList(WHO_AM_I, 1) # read 1 byte
print bytes[0]
print hex(bytes[0])

# if binascii.b2a_hex(str(bytes)) == '71':
if hex(bytes[0]) == str(hex(IDENTITY)):
    print "IMU detected !"
else:
    raise ValueError('IMU NOT IDENTIFIED! the IMU bus 2 addr 0x68 reg 0x75 Who_Am_I != 0x71 !')

print('MPU9250 Initiated \n')
# # i2c.write16(PWR_MGMT_1, 0x80) #电源管理,复位MPU9250
# # i2c.write16(SMPLRT_DIV, 0x07) #陀螺仪采样率1000/(1+7)=125Hz
# # i2c.write16(CONFIG, 0X06) # 低通滤波器 0x06 5 hz
# i2c.write16(ACCEL_CONFIG, 0x00) # +- 2g
# # i2c.write16(ACCEL_CONFIG2, 0x00) # 加速度采样频率460HZ
i2c.write16(GYRO_CONFIG, 0xe0) # 245 dps

# i2c.write16()

i2c.write8(0x1F,0b00111000)

# Read the data

time.sleep(0.01)  # Wait for the refresh of IMU register, otherwise the READING is FAKE

# print hex(i2c.readU8(ACCEL_XOUT_H))

print i2c.readList(GYRO_CONFIG, 1)

# MPU9250_buf = i2c.readList(ACCEL_XOUT_H, 14) # Read all the data(acc, temp, gyro)
# print bin(MPU9250_buf[0])
# print bin(MPU9250_buf[1])
# MPU9250_ACC_LASTX = (MPU9250_buf[0]<<8)| MPU9250_buf[1] #加速度原始数据
# print bin(MPU9250_ACC_LASTX)
# print hex((MPU9250_buf[0]<<8)| MPU9250_buf[1])
# IMU_data = struct.unpack('>hhhHhhh', (MPU9250_buf[0:14]))
# print i2c.readU16(ACCEL_XOUT_L)
while 1:
    # Acc_X = (struct.unpack('>h',(i2c.readList(ACCEL_XOUT_L, 2))))
    # time.sleep(1)
    readtime = time.time()
    IMU_Data = (struct.unpack('<hhhhhhh', (i2c.readList(TEMP_OUT_L, 14))))
    # print IMU_Data
    temp = 25 + IMU_Data[0] / 16.0
    accx = IMU_Data[4] * 0.061
    accy = IMU_Data[5] * 0.061
    accz = IMU_Data[6] * 0.061
    gx = IMU_Data[1] * 0.00875
    gy = IMU_Data[2] * 0.00875
    gz = IMU_Data[3] * 0.00875


    # print type(Acc_X)
    # temp =  25 + struct.unpack('<h',(i2c.readList(TEMP_OUT_H, 2)))[0]/16.0
    # accx = struct.unpack('<h',(i2c.readList(ACCEL_XOUT_H, 2)))[0] * 0.061
    # accy = struct.unpack('<h',(i2c.readList(ACCEL_YOUT_H, 2)))[0]  * 0.061
    # accz = struct.unpack('<h',(i2c.readList(ACCEL_ZOUT_H, 2)))[0]  * 0.061
    # gx = struct.unpack('<h',(i2c.readList(GYRO_XOUT_H, 2)))[0] * 0.00875
    # gy = struct.unpack('<h',(i2c.readList(GYRO_YOUT_H, 2)))[0] * 0.00875
    # gz = struct.unpack('<h',(i2c.readList(GYRO_ZOUT_H, 2)))[0] * 0.00875
    print (time.time() - readtime), temp, accx, accy, accz, gx, gy, gz
    # # # Acc_X = i2c.readU8(ACCEL_XOUT_L) | (i2c.readU8(ACCEL_XOUT_H) << 8)
    # print Acc_X
    # print hex(i2c.readU8(ACCEL_XOUT_H))
    # print hex(i2c.readU8(ACCEL_XOUT_L))
