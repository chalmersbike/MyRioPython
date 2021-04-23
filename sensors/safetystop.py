from param import *
import math
import Adafruit_BBIO.GPIO as GPIO
from time import sleep, time


class SafetyStop(object):
    ESTOP = False

    def __init__(self):
        GPIO.setup(safetyStop_port, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if safetyStop_pullUpDown == 'up':
            GPIO.setup(safetyStop_port, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        elif safetyStop_pullUpDown == 'down':
            GPIO.setup(safetyStop_port, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        else:
            print("Safety stop : Chosen safety stop pull-up or pull-down type is not valid : %s. Choosing pull-up instead" %(safetyStop_pullUpDown))
            GPIO.setup(safetyStop_port, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.ESTOP_previous = False

    def button_check(self):
        # If GPIO.input == 0 it means the stop button has been pressed
        if GPIO.input(safetyStop_port):
            self.ESTOP = False
        else:
            self.ESTOP = True

        # if not self.ESTOP == self.ESTOP_previous:
        #     self.ESTOP = not self.ESTOP
        # self.ESTOP_previous = not bool(GPIO.input(safetyStop_port))

        # print("estop_gpio = %r ; estop = %r ; estop_previous = %r" % (bool(GPIO.input(safetyStop_port)),self.ESTOP,self.ESTOP_previous))

        # FOR TESTING PURPOSE ONLY, WILL IGNORE THE EMERGENCY STOP TO FORCEFULLY RUN THE CODE
        # The motors will still be electrically disconnected if the emergency stop is pressed
        # self.ESTOP = False

        return self.ESTOP
