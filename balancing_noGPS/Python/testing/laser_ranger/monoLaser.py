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

import sys
sys.path.append(sys.path[0]+'/../../')
import time
from sensors import VL53L0X
import Adafruit_BBIO.GPIO as GPIO

# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = "GP1_3"
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = "GP1_4"

# GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
# GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor1_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_UP)
GPIO.setup(sensor2_shutdown, GPIO.OUT, pull_up_down=GPIO.PUD_UP)


# Keep all low for 500 ms or so to make sure they reset
time.sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
tof = VL53L0X.VL53L0X(address=0x2B)

# Set shutdown pin high for the first VL53L0X then
# call to start ranging
GPIO.output(sensor1_shutdown, GPIO.HIGH)
GPIO.output(sensor2_shutdown, GPIO.HIGH)
time.sleep(0.50)
tof.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

timing = tof.get_timing()
if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))
distance1_old = 0
distance2_old = 0
starttime = time.time()
for count in range(1, 10000):
    print("TIMESTEP \t %f" %(time.time()-starttime))
    distance1 = tof.get_distance()
    if (distance1 > 0):
        distance1_old = distance1
        print ("sensor %d - %d mm, %d cm, iteration %d" % (tof.my_object_number, distance1, (distance1/10), count))
    else:
        print "Warning: channel 0 strange reading %f" % distance1
        # distance1 = distance1_old
        # print ("sensor %d - %d mm, %d cm, iteration %d" % (tof.my_object_number, distance1, (distance1 / 10), count))
        # print ("%d - Error" % tof.my_object_number)
    # time.sleep(0.001)
    time.sleep(timing/1000000 + 0.1)


tof.stop_ranging()
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)