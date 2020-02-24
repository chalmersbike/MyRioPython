import Adafruit_BBIO.ADC as ADC

OUTPUT_PORT = 'P9_39'


# A low pass filter is needed for this potentiometer as the signal is very noisy. The components for this filter have
#  not been selected properly.

class Potentiometer(object):

    def __init__(self):
        ADC.setup()

    def read_pot_value(self):
        return ADC.read(OUTPUT_PORT)
