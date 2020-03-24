import math

import Adafruit_BBIO.GPIO as GPIO
from time import sleep, time
INPUT_PORT = 'P9_26'
# INPUT_PORT = 'GPIO3_17'
# INPUT_PORT = 'SPI1_CS0'
#INPUT_PORT = 'GP1_4'

class SafetyStop(object):
    ESTOP = False

    def __init__(self):
        GPIO.setup(INPUT_PORT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #GPIO.setup(INPUT_PORT, GPIO.IN)
        print "GPIO P3_1 has been configed Pulled UP"

    def button_check(self):
        # If GPIO.input == 0 it means the stop button has been pressed
        if GPIO.input(INPUT_PORT):
            self.ESTOP = False
        else:
            self.ESTOP = True
        #self.ESTOP = False

        return self.ESTOP
