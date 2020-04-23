#!/usr/bin/python

# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import time
from ctypes import *
import smbus
import time
import Adafruit_BBIO.GPIO as GPIO
import VL53L0X
#import pysnooper

VL53L0X_GOOD_ACCURACY_MODE = 0  # Good Accuracy mode
VL53L0X_BETTER_ACCURACY_MODE = 1  # Better Accuracy mode
VL53L0X_BEST_ACCURACY_MODE = 2  # Best Accuracy mode
VL53L0X_LONG_RANGE_MODE = 3  # Longe Range mode
VL53L0X_HIGH_SPEED_MODE = 4  # High Speed mode

i2cbus = smbus.SMBus(1)
roller_width = 380  # milimeter

# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 'P9_23'
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 'P9_27'


# GPIO.setwarnings(False)

class DualLaserRanger(object):
    def __init__(self):
        # Setup GPIO for shutdown pins on each VL53L0X
        # GPIO.setmode(GPIO.BCM)
        GPIO.setup(sensor1_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(sensor2_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)

        # Set all shutdown pins low to turn off each VL53L0X
        GPIO.output(sensor1_shutdown, GPIO.LOW)
        GPIO.output(sensor2_shutdown, GPIO.LOW)

        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)

        # Create one object per VL53L0X passing the address to give to
        # each.
        self.tof1 = VL53L0X.VL53L0X(address=0x39)
        self.tof2 = VL53L0X.VL53L0X(address=0x29)

        # Set shutdown pin high for the first VL53L0X then
        # call to start ranging
        GPIO.output(sensor1_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof1.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)

        # Set shutdown pin high for the second VL53L0X then
        # call to start ranging
        GPIO.output(sensor2_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof2.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)

        self.timing = self.tof1.get_timing()
        if (self.timing < 20000):
            self.timing = 20000
        print ("Timing %d ms" % (self.timing / 1000))
        self.timing = self.timing / 1000000.00 + 0.1

        self.distance1 = 0
        self.distance2 = 0

        self.distance1_old = 0
        self.distance2_old = 0
        self.bike_y = 0
        self.last_read_time = 0

    def __del__(self):
        self.tof2.stop_ranging()
        GPIO.output(sensor2_shutdown, GPIO.LOW)
        self.tof1.stop_ranging()
        GPIO.output(sensor1_shutdown, GPIO.LOW)

    # @pysnooper.snoop()
    def get_y(self):
        if time.time() - self.last_read_time > self.timing:
            self.last_read_time = time.time()
            # print "Reading the laser data"
            dist = self.get_distance()
            # print dist
            # return (distances[1] - distances[0])/2 # TRUE if the sensor readings are reliable

            if dist[0] < 1000 and dist[1] < 1000:
                self.bike_y = (roller_width / 2 - dist[0]) / 1000.0 if dist[0] >= dist[1] else (dist[1] - roller_width / 2) / 1000.0
            else:
                print "Warning: The laser reading exceeds the roller, neglected"

            return self.bike_y
        else:
            print "Too Frequent Laser Ranger Reading"

            return self.bike_y

    def get_distance(self):
        # Sensor 1 # LEFT SENSOR
        self.distance1 = self.tof1.get_distance()
        if (self.distance1 > 0):
            self.distance1_old = self.distance1
        else:
            print "Warning: channel 1 strange reading %f" % self.distance1
            self.distance1 = self.distance1_old
        # time.sleep(0.0001)
        # Sensor 2 RIGHT SENSOR
        self.distance2 = self.tof2.get_distance()
        if (self.distance2 > 0):
            self.distance2_old = self.distance2
        else:
            print "Warning: channel 2 strange reading %f" % self.distance2
            self.distance2 = self.distance2_old
        return (self.distance1, self.distance2)

# for count in range(1,10000):
#     distance1 = self.tof1.get_distance()
#     if (distance1 > 0):
#         distance1_old = distance1
#         # print ("sensor %d - %d mm, %d cm, iteration %d" % (self.tof1.my_object_number, distance1, (distance1/10), count))
#     else:
#         distance1 = distance1_old
#         # print ("%d - Error" % self.tof1.my_object_number)
#
#     distance2 = self.tof2.get_distance()
#     if (distance2 > 0):
#         distance2_old = distance2
#         # print ("sensor %d - %d mm, %d cm, iteration %d" % (self.tof2.my_object_number, distance2, (distance2/10), count))
#     else:
#         distance2 = distance2_old
#
#
#     # time.sleep(timing/1000000.00)
#
# self.tof2.stop_ranging()
# GPIO.output(sensor2_shutdown, GPIO.LOW)
# self.tof1.stop_ranging()
# GPIO.output(sensor1_shutdown, GPIO.LOW)
#
#
#
#
#
#
#
#
#
#
#
#
#
# # i2c bus read callback
# def i2c_read(address, reg, data_p, length):
#     ret_val = 0;
#     result = []
#
#     try:
#         result = i2cbus.read_i2c_block_data(address, reg, length)
#     except IOError:
#         ret_val = -1;
#
#     if (ret_val == 0):
#         for index in range(length):
#             data_p[index] = result[index]
#
#     return ret_val
#
#
# # i2c bus write callback
# def i2c_write(address, reg, data_p, length):
#     ret_val = 0;
#     data = []
#
#     for index in range(length):
#         data.append(data_p[index])
#     try:
#         i2cbus.write_i2c_block_data(address, reg, data)
#     except IOError:
#         ret_val = -1;
#
#     return ret_val
#
#
# # Load VL53L0X shared lib
# tof_lib = CDLL("./vl53l0x_python.so")
#
# # Create read function pointer
# READFUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
# read_func = READFUNC(i2c_read)
#
# # Create write function pointer
# WRITEFUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
# write_func = WRITEFUNC(i2c_write)
#
# # pass i2c read and write function pointers to VL53L0X library
# tof_lib.VL53L0X_set_i2c(read_func, write_func)
#
#
# class VL53L0X(object):
#     """VL53L0X ToF."""
#
#     object_number = 0
#
#     def __init__(self, address=0x29, TCA9548A_Num=255, TCA9548A_Addr=0, **kwargs):
#         """Initialize the VL53L0X ToF Sensor from ST"""
#         self.device_address = address
#         self.TCA9548A_Device = TCA9548A_Num
#         self.TCA9548A_Address = TCA9548A_Addr
#         self.my_object_number = VL53L0X.object_number
#         VL53L0X.object_number += 1
#
#     def start_ranging(self, mode=VL53L0X_GOOD_ACCURACY_MODE):
#         """Start VL53L0X ToF Sensor Ranging"""
#         tof_lib.startRanging(self.my_object_number, mode, self.device_address, self.TCA9548A_Device,
#                              self.TCA9548A_Address)
#
#     def stop_ranging(self):
#         """Stop VL53L0X ToF Sensor Ranging"""
#         tof_lib.stopRanging(self.my_object_number)
#
#     def get_distance(self):
#         """Get distance from VL53L0X ToF Sensor"""
#         return tof_lib.getDistance(self.my_object_number)
#
#     # This function included to show how to access the ST library directly
#     # from python instead of through the simplified interface
#     def get_timing(self):
#         Dev = POINTER(c_void_p)
#         Dev = tof_lib.getDev(self.my_object_number)
#         budget = c_uint(0)
#         budget_p = pointer(budget)
#         Status = tof_lib.VL53L0X_GetMeasurementTimingBudgetMicroSeconds(Dev, budget_p)
#         if (Status == 0):
#             return (budget.value + 1000)
#         else:
#             return 0