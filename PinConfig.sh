#!/usr/bin/env bash


config-pin P9.21 pwm # PWM to steering
config-pin P9.22 gpio #Enable Steering
config-pin P9.24 uart #TX
config-pin P9.26 uart #RX
config-pin P9.23 gpio # HallSensor GPIO1.25
#config-pin P9.28 gpio # Emergency stop GPIO3_17
#config-pin P9.11 gpio # Emergency stop UART_DSM_RX (P9.11)
config-pin P9.28 spi_cs # PmodNAV IMU SPI CS
config-pin P9.29 spi # PmodNAV IMU SPI MISO
config-pin P9.30 spi # PmodNAV IMU SPI MOSI
config-pin P9.31 spi_sclk # PmodNAV IMU SPI SCK
echo 113 > /sys/class/gpio/export
echo 49 > /sys/class/gpio/export
echo "Pin configuration complete"