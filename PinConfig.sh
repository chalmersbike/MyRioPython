#!/usr/bin/env bash


config-pin P9.21 pwm # PWM to steering
config-pin P9.22 gpio #Enable Steering
config-pin P9.24 uart #TR
config-pin P9.26 uart #RX
config-pin P9.23 gpio # HallSensor GPIO1.25
config-pin P9.28 gpio # Emergency stop GPIO3_17
echo 113 > /sys/class/gpio/export
echo 49 > /sys/class/gpio/export
echo "Pin configuration complete"