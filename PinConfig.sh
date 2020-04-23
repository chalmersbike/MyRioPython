#!/usr/bin/env bash

config-pin p8.7 gpio
config-pin p8.7 out-
config-pin p8.8 gpio
config-pin p8.8 out-
config-pin p8.10 gpio
config-pin p9.20 i2c
config-pin p9.19 i2c
config-pin p9.24 uart
config-pin p9.26 uart
config-pin p9.12 gpio
config-pin p9.12 out-
config-pin p8.12 qep
config-pin p8.11 qep
config-pin p8.16 qep
config-pin p9.11 uart
config-pin p9.13 uart
config-pin p9.12 gpio
config-pin p9.14 pwm
config-pin p8.13 pwm
config-pin p8.17 gpio

#config-pin p9.21 uart
#config-pin p9.22 uart

config-pin P9.17 spi_cs
config-pin P9.18 spi
config-pin P9.21 spi
config-pin P9.22 spi_sclk
config-pin 8.15 out-



echo "Pin configuration complete"