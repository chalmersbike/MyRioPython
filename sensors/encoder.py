from param import *
import math
import time

from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP2


class Encoder(object):
    encoder = None

    def __init__(self):
        self.encoder = RotaryEncoder(eQEP2)
        self.encoder.setAbsolute()
        self.calibrate()


    def calibrate(self):
        self.encoder.zero()

    def get_angle(self):
        return - -self.encoder.position * steeringEncoder_TicksToRadianRatio
