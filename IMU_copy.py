#!/usr/bin/python
# -*- coding: UTF-8 -*-
BUS = 2
SMPLRT_DIV = 0X19 #陀螺仪采样率典型值为0X07 1000/(1+7)=125HZ
CONFIG = 0X1A #低通滤波器  典型值0x06 5hz
GYRO_CONFIG = 0X1B #陀螺仪测量范围 0X18 正负2000度
ACCEL_CONFIG = 0X1C #加速度计测量范围 0X18 正负16g
ACCEL_CONFIG2 = 0X1D #加速度计低通滤波器 0x06 5hz
PWR_MGMT_1 = 0X6B#电源管理1 典型值为0x00
PWR_MGMT_2 = 0X6C #电源管理2 典型值为0X00
WHO_AM_I  = 0X75 #器件ID MPU9250默认ID为0X71
IDENTITY = 0X71
USER_CTRL = 0X6A #用户配置当为0X10时使用SPI模式
# MPU9250_CS        PDout(3) #MPU9250片选信号
I2C_ADDR = 0X68  #i2c的地址
ACCEL_XOUT_H = 0X3B  #加速度计输出数据
ACCEL_XOUT_L = 0X3C
ACCEL_YOUT_H = 0X3D
ACCEL_YOUT_L = 0X3E
ACCEL_ZOUT_H = 0X3F
ACCEL_ZOUT_L = 0X40
TEMP_OUT_H =   0X41  #温度计输出数据
TEMP_OUT_L =   0X42
GYRO_XOUT_H =  0X43  #陀螺仪输出数据
GYRO_XOUT_L =  0X44
GYRO_YOUT_H =  0X45
GYRO_YOUT_L =  0X46
GYRO_ZOUT_H =  0X47
GYRO_ZOUT_L =  0X48

import binascii
import Adafruit_GPIO.I2C as I2C
import time
import struct
i2c = I2C.Device(I2C_ADDR, BUS)
# i2c.get_i2c_device(0x68)
bytes = i2c.readList(WHO_AM_I, 1) # read 1 byte

# if binascii.b2a_hex(str(bytes)) == '71':
if hex(bytes[0]) == str(hex(IDENTITY)):
    print "IMU detected !"
else:
    raise ValueError('IMU NOT IDENTIFIED! the IMU bus 2 addr 0x68 reg 0x75 Who_Am_I != 0x71 !')

i2c.write8(PWR_MGMT_1, 0x80) #电源管理,复位MPU9250
print('MPU9250 Initiated \n')
i2c.write8(SMPLRT_DIV, 0x07) #陀螺仪采样率1000/(1+7)=125Hz
i2c.write8(CONFIG, 0X06) # 低通滤波器 0x06 5 hz
i2c.write8(ACCEL_CONFIG, 0x18) # 加速度计测量范围 0X18 正负16g
i2c.write8(ACCEL_CONFIG2, 0x00) # 加速度采样频率460HZ

gyro_acc_config = bin(i2c.readU16(GYRO_CONFIG))
cfg_len_offset = 16-gyro_acc_config.__len__()+2
Gyro_FS_SEL = gyro_acc_config[4+1-cfg_len_offset:6+1-cfg_len_offset]
Accel_FS_SEL = gyro_acc_config[12+1-cfg_len_offset:14+1-cfg_len_offset]
if Gyro_FS_SEL == '00':
    Gyro_Sensitivity = 16384.0
elif Gyro_FS_SEL == '01':
    Gyro_Sensitivity = 8192.0
elif Gyro_FS_SEL == '10':
    Gyro_Sensitivity = 4096.0
elif Gyro_FS_SEL == '11':
    Gyro_Sensitivity = 2048.0
print Gyro_Sensitivity

if Accel_FS_SEL == '00':
    Accel_Sensitivity = 131.0
elif Accel_FS_SEL == '01':
    Accel_Sensitivity = 65.5
elif Accel_FS_SEL == '10':
    Accel_Sensitivity = 32.8
elif Accel_FS_SEL == '11':
    Accel_Sensitivity = 16.4
print Accel_Sensitivity
# bin(gyro_acc_config)

# i2c.write16()

# Read the data

time.sleep(0.01)  # Wait for the refresh of IMU register, otherwise the READING is FAKE

# print hex(i2c.readU8(ACCEL_XOUT_H))
# print i2c.readList(GYRO_CONFIG, 1)

MPU9250_buf = i2c.readList(ACCEL_XOUT_H, 14) # Read all the data(acc, temp, gyro)
print bin(MPU9250_buf[0])
print bin(MPU9250_buf[1])
MPU9250_ACC_LASTX = (MPU9250_buf[0]<<8)| MPU9250_buf[1] #加速度原始数据
# MPU9250_ACC_LASTX = bytes(MPU9250_buf[0]<<8| MPU9250_buf[1]) #加速度原始数据
print bin(MPU9250_ACC_LASTX)
print hex((MPU9250_buf[0]<<8)| MPU9250_buf[1])
# print struct.unpack('>h', hex((MPU9250_buf[0]<<8)| MPU9250_buf[1]))

while 1:
    time.sleep(0.1)
    MPU9250_buf = i2c.readList(ACCEL_XOUT_H, 14)  # Read all the data(acc, temp, gyro)
    IMU_data = struct.unpack('>hhhHhhh', (MPU9250_buf[0:14]))
    ax = IMU_data[0] / Gyro_Sensitivity
    ay = IMU_data[1] / Gyro_Sensitivity
    az = IMU_data[2] / Gyro_Sensitivity
    Temp = ((IMU_data[3] - 21) / 333.87) + 21
    gx = IMU_data[4] / Accel_Sensitivity
    gy = IMU_data[5] / Accel_Sensitivity
    gz = IMU_data[6] / Accel_Sensitivity
    print ax, ay, az, Temp, gx, gy, gz
