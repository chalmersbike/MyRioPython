import Adafruit_BBIO.ADC as ADC

OUTPUT_PORT = 'P9_36'


# A low pass filter is needed for this potentiometer as the signal is very noisy. The components for this filter have
#  not been selected properly.

class Potentiometer(object):

    def __init__(self):
        ADC.setup()

    def read_pot_value(self):
        return ADC.read(OUTPUT_PORT)


from sensors import Potentiometer
import Adafruit_BBIO.GPIO as GPIO

import time
import csv

# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")
class Test(object):
    def __init__(self):
         # IMU
        number_samples_POT = raw_input('Input the number of samples or press ENTER for 50 samples for the IMU test, move the bike body for the reading! ')
        self.pot = Potentiometer()
        start_time = time.time()
        if number_samples_POT is not 0:
             # Setup CSV file
             results_pot = open('Tests_Lukas/%s-SensorTest_Lukas_POT.csv' % timestr, 'wb')
             writer_pot = csv.writer(results_pot)
             writer_pot.writerow(('Time (s)', 'Pot'))
        for x in range(1,int(number_samples_POT)+1):
             self.potdata = self.pot.read_pot_value()
             print 'Time = %g\tPot = %g\t' % (time.time() - start_time, self.potdata)

             # Write to CSV file
             writer_pot.writerow((time.time() - start_time, self.potdata))
test = Test()
