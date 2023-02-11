from sensors import SafetyStop
import Adafruit_BBIO.GPIO as GPIO
from time import sleep
import time

class Test(object):
    def __init__(self):
        # Safety stop
        self.safety_stop = SafetyStop()
        input1 = raw_input('Press the EStop and Enter, test the emergency stop is detectable!')
        ESign = self.safety_stop.button_check()
        while not ESign:
            print 'The EStop was not pressed!'
            time.sleep(0.5)
            ESign = self.safety_stop.button_check()
        if ESign:
            print 'Estop detected!'
test = Test()
