from param import *
import math
from time import sleep, time


class SafetyStop(object):
    ESTOP = False

    def __init__(self,session):
        self.fpga_ESTOP = session.registers['ESTOP']
        self.ESTOP_previous = False
        print ('ESTOP INIT')

    def button_check(self):
        self.ESTOP = self.fpga_ESTOP.read()

        # FOR TESTING PURPOSE ONLY, WILL IGNORE THE EMERGENCY STOP TO FORCEFULLY RUN THE CODE
        # The motors will still be electrically disconnected if the emergency stop is pressed
        self.ESTOP = False

        return self.ESTOP
