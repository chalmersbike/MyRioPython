import sys
sys.path.append(sys.path[0]+'/../../')
import param
from sensors import HallSensor
import Adafruit_BBIO.GPIO as GPIO

from actuators import DriveMotor
import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")


class Test(object):
    def __init__(self):
        self.mot = DriveMotor()
        self.mot.startLog()
        while 1:
            time_start = time.time()
            data = self.mot.getLog()
            print(time.time() - time_start)
            print(data)
            # time.sleep(0.11)

test = Test()