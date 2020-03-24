import math
import time

from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP1

TICKS_PER_REVOLUTION = 2 * 1000 * 111
TICKS_TO_RADIAN_RATIO = 2 * math.pi / TICKS_PER_REVOLUTION


class Encoder(object):

    encoder = None

    def __init__(self):
        self.encoder = RotaryEncoder(eQEP1)
        self.encoder.setAbsolute()
        self.calibrate()


    def calibrate(self):
        self.encoder.zero()

    def get_angle(self):
        return - self.encoder.position * TICKS_TO_RADIAN_RATIO
