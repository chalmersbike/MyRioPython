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

VL53L0X_GOOD_ACCURACY_MODE = 0      # Good Accuracy mode
VL53L0X_BETTER_ACCURACY_MODE = 1    # Better Accuracy mode
VL53L0X_BEST_ACCURACY_MODE = 2      # Best Accuracy mode
VL53L0X_LONG_RANGE_MODE = 3         # Long Range mode
VL53L0X_HIGH_SPEED_MODE = 4         # High Speed mode

i2cbus = smbus.SMBus(1)

#@pysnooper.snoop()
class DualLaserRanger(object):
    def __init__(self):
        # Setup GPIO for shutdown pins on each VL53L0X
        # GPIO.setmode(GPIO.BCM)
        if laserRanger_sensor1_shutdown_pullUpDown == 'down':
            GPIO.setup(laserRanger_sensor1_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        elif laserRanger_sensor1_shutdown_pullUpDown == 'up':
            GPIO.setup(laserRanger_sensor1_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_up)
        else:
            print("Laser ranger 1 : Chosen GPIO shutdown edge detection type is not valid : %s. Choosing pull-down instead" %(laserRanger_sensor1_shutdown_pullUpDown))
            GPIO.setup(laserRanger_sensor1_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)

        if laserRanger_sensor2_shutdown_pullUpDown == 'down':
            GPIO.setup(laserRanger_sensor2_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        elif laserRanger_sensor2_shutdown_pullUpDown == 'up':
            GPIO.setup(laserRanger_sensor2_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_up)
        else:
            print("Laser ranger 2 : Chosen GPIO shutdown edge detection type is not valid : %s. Choosing pull-down instead" %(laserRanger_sensor2_shutdown_pullUpDown))
            GPIO.setup(laserRanger_sensor2_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)

        # Set all shutdown pins low to turn off each VL53L0X
        GPIO.output(laserRanger_sensor1_shutdown, GPIO.LOW)
        GPIO.output(laserRanger_sensor2_shutdown, GPIO.LOW)

        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)

        # Create one object per VL53L0X passing the address to give to
        # each.
        self.tof1 = VL53L0X.VL53L0X(address=laserRanger_sensor1_I2Caddress)
        self.tof2 = VL53L0X.VL53L0X(address=laserRanger_sensor2_I2Caddress)

        # Set shutdown pin high for the first VL53L0X then
        # call to start ranging
        GPIO.output(laserRanger_sensor1_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        if laserRanger_sensor1_accuracyMode == 0:
            self.tof1.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)
        elif laserRanger_sensor1_accuracyMode == 1:
            self.tof1.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
        elif laserRanger_sensor1_accuracyMode == 2:
            self.tof1.start_ranging(VL53L0X.VL53L0X_BEST_ACCURACY_MODE)
        elif laserRanger_sensor1_accuracyMode == 3:
            self.tof1.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)
        elif laserRanger_sensor1_accuracyMode == 4:
            self.tof1.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)
        else:
            print("Laser ranger 1 : Chosen accuracy mode is not valid : %d. Choosing high speed mode (4) instead" %(laserRanger_sensor1_accuracyMode))
            self.tof1.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)

        # Set shutdown pin high for the second VL53L0X then
        # call to start ranging
        GPIO.output(laserRanger_sensor2_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        if laserRanger_sensor2_accuracyMode == 0:
            self.tof2.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)
        elif laserRanger_sensor2_accuracyMode == 1:
            self.tof2.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
        elif laserRanger_sensor2_accuracyMode == 2:
            self.tof2.start_ranging(VL53L0X.VL53L0X_BEST_ACCURACY_MODE)
        elif laserRanger_sensor2_accuracyMode == 3:
            self.tof2.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)
        elif laserRanger_sensor2_accuracyMode == 4:
            self.tof2.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)
        else:
            print("Laser ranger 2 : Chosen accuracy mode is not valid : %d. Choosing high speed mode (4) instead" %(laserRanger_sensor2_accuracyMode))
            self.tof2.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)


        self.timing = self.tof1.get_timing()
        if (self.timing < 20000):
            self.timing = 20000
        if debug:
            print(("Laser ranger : Timing %d ms" % (self.timing / 1000)))
        self.timing = self.timing / 1000000.00

        self.distance1 = 0
        self.distance2 = 0

        self.distance1_old = 0
        self.distance2_old = 0
        self.bike_y = 0
        self.last_read_time = 0

    def __del__(self):
        self.tof2.stop_ranging()
        GPIO.output(laserRanger_sensor2_shutdown, GPIO.LOW)
        self.tof1.stop_ranging()
        GPIO.output(laserRanger_sensor1_shutdown, GPIO.LOW)

    # @pysnooper.snoop()
    def get_y(self):
        if time.time() - self.last_read_time > self.timing:
            self.last_read_time = time.time()
            # print("Reading the laser data"
            dist = self.get_distance()
            # print(dist
            # return (distances[1] - distances[0])/2 # TRUE if the sensor readings are reliable

            if dist[0] < 1000 and dist[1] < 1000:
                self.bike_y = (laserRanger_roller_width / 2 - dist[0]) / 1000.0 if dist[0] >= dist[1] else (dist[1] - laserRanger_roller_width / 2) / 1000.0
            else:
                if debug:
                    print("Warning: The laser reading exceeds the roller, ignoring this data point")

            return self.bike_y
        else:
            if debug:
                print("Too Frequent Laser Ranger Reading")

            return self.bike_y

    def get_distance(self):
        # Sensor 1 : LEFT SENSOR
        self.distance1 = self.tof1.get_distance()
        if (self.distance1 > 0):
            self.distance1_old = self.distance1
        else:
            if debug:
                print("Laser ranger : Sensor 1 negative reading %f" % self.distance1)
            self.distance1 = self.distance1_old
        # time.sleep(0.0001)
        # Sensor 2 : RIGHT SENSOR
        self.distance2 = self.tof2.get_distance()
        if (self.distance2 > 0):
            self.distance2_old = self.distance2
        else:
            if debug:
                print("Laser ranger : Sensor 1 negative reading %f" % self.distance1)
            self.distance2 = self.distance2_old
        return (self.distance1, self.distance2)