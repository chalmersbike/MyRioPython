from param import *
import Adafruit_BBIO.ADC as ADC


# A low pass filter is needed for this potentiometer as the signal is very noisy. The components for this filter have
#  not been selected properly.

class Potentiometer(object):
    def __init__(self):
        ADC.setup()

    def read_pot_value(self):
        return ADC.read(potentiometer_port)
