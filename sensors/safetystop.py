import math

import Adafruit_BBIO.GPIO as GPIO
from time import sleep, time
INPUT_PORT = 'P9_28'
# INPUT_PORT = 'GPIO3_17'
# INPUT_PORT = 'SPI1_CS0'

class SafetyStop(object):
    ESTOP = False

    def __init__(self):
        GPIO.setup(INPUT_PORT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print "GPIO P9_23 has been configed Pulled UP"

    def button_check(self):
        # If GPIO.input == 0 it means the stop button has been pressed
        if GPIO.input(INPUT_PORT):
            self.ESTOP = False
        else:
            self.ESTOP = True

        return self.ESTOP
