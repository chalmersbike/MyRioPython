#!/usr/bin/env bash

################################################################
# P8
################################################################
config-pin p8.7 gpio # 24/04/2020 : NOT USED
config-pin p8.7 out- # 24/04/2020 : NOT USED
config-pin p8.8 gpio # 24/04/2020 : NOT USED
config-pin p8.8 out- # 24/04/2020 : NOT USED
config-pin p8.10 gpio # 24/04/2020 : NOT USED
config-pin p8.11 qep # 24/04/2020 : ENCODER A
config-pin p8.12 qep # 24/04/2020 : ENCODER B
config-pin p8.13 pwm # 24/04/2020 : STEERING PWM
config-pin p8.15 out- # 24/04/2020 : NOT USED
config-pin p8.16 qep # 24/04/2020 : NOT USED
config-pin p8.17 gpio # 24/04/2020 : STEERING ENABLE


################################################################
# P9
################################################################
config-pin p9.11 uart # 24/04/2020 : NOT USED
config-pin p9.12 gpio # 24/04/2020 : NOT USED
config-pin p9.12 out- # 24/04/2020 : NOT USED
config-pin p9.13 uart # 24/04/2020 : DRIVE MOTOR UART
config-pin p9.14 pwm # 24/04/2020 : NOT USED
config-pin p9.15 gpio # 24/04/2020 : ESTOP
config-pin P9.17 spi_cs # 24/04/2020 : IMU SPI CS
config-pin P9.18 spi # 24/04/2020 : IMU SPI MOSI
config-pin p9.19 i2c # 24/04/2020 : I2C LASER RANGER SCL
config-pin p9.20 i2c # 24/04/2020 : I2C LASER RANGER SDA
config-pin P9.21 spi # 24/04/2020 : IMU SPI MISO
config-pin P9.22 spi_sclk # 24/04/2020 : IMU SPI CLK
config-pin p9.23 gpio # 24/04/2020 : I2C LASER RANGER SHUTDOWN 1
config-pin p9.24 uart # 24/04/2020 : GPS UART BBB/TX GPS/RX
config-pin p9.26 uart # 24/04/2020 : GPS UART BBB/RX GPS/TX
config-pin p9.27 gpio # 24/04/2020 : I2C LASER RANGER SHUTDOWN 1
config-pin p9.30 gpio # 24/04/2020 : HALL SENSOR R1


echo "Pin configuration complete"