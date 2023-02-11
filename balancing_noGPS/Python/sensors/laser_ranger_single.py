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

from param import *
import time
from ctypes import *
import smbus
import time
import Adafruit_BBIO.GPIO as GPIO
from .VL53L0X import VL53L0X
import pysnooper

VL53L0X_GOOD_ACCURACY_MODE = 0  # Good Accuracy mode
VL53L0X_BETTER_ACCURACY_MODE = 1  # Better Accuracy mode
VL53L0X_BEST_ACCURACY_MODE = 2  # Best Accuracy mode
VL53L0X_LONG_RANGE_MODE = 3  # Longe Range mode
VL53L0X_HIGH_SPEED_MODE = 4  # High Speed mode

i2cbus = smbus.SMBus(2)


class SingleLaserRanger(object):
    def __init__(self):
        self.tof = VL53L0X.VL53L0X()

        if laserRanger_sensor1_accuracyMode == 0:
            self.tof.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)
        elif laserRanger_sensor1_accuracyMode == 1:
            self.tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
        elif laserRanger_sensor1_accuracyMode == 2:
            self.tof.start_ranging(VL53L0X.VL53L0X_BEST_ACCURACY_MODE)
        elif laserRanger_sensor1_accuracyMode == 3:
            self.tof.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)
        elif laserRanger_sensor1_accuracyMode == 4:
            self.tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)
        else:
            print("Laser ranger 1 : Chosen accuracy mode is not valid : %d. Choosing high speed mode (4) instead" %(laserRanger_sensor1_accuracyMode))
            self.tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)
            
            
        self.timing = self.tof.get_timing()
        if (self.timing < 20000):
            self.timing = 20000
        if debug:
            print(("Timing %d ms" % (self.timing / 1000)))
        self.timing = self.timing / 1000000.00 + 0.1

        # self.distance1 = 0
        self.distance = 0

        # self.distance1_old = 0
        self.distance_old = 0
        self.bike_y = 0
        self.last_read_time = 0

    def __del__(self):
        self.tof.stop_ranging()

    # @pysnooper.snoop()
    def get_y(self):
        if time.time() - self.last_read_time > self.timing:
            self.last_read_time = time.time()
            if debug:
                print("Reading the laser data")
            dist = self.get_distance()
            print(dist)
            # return (distances[1] - distances[0])/2 # TRUE if the sensor readings are reliable

            if dist < 1000:
                self.bike_y = (dist - roller_width / 2) / 1000.0
            else:
                if debug:
                    print("Warning: The laser reading exceeds the roller, ignoring this data point")

            return self.bike_y
        else:
            if debug:
                print("Too Frequent Laser Ranger Reading")

            return self.bike_y

    def get_distance(self):
        # Sensor 2 RIGHT SENSOR
        self.distance = self.tof.get_distance()
        if (self.distance > 0):
            self.distance_old = self.distance
        else:
            if debug:
                print("Warning: channel strange reading %f" % self.distance)
            self.distance = self.distance_old
            self.tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)
        return (self.distance)