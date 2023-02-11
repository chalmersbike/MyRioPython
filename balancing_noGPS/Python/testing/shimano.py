# Test for shimano rear motor.
# Spins until E-STOP is pressed or CTRL-C

import sys
import time

sys.path.append('../')

from bike import Bike

bike = Bike(debug=True)

ESTOP = False

try:
    ESTOP = bike.emergency_stop_check()
    print ESTOP

    raw_input('Press ENTER to start')
    while not ESTOP:
        ESTOP = bike.emergency_stop_check()
        bike.forward()
        time.sleep(0.01)
except (ValueError, KeyboardInterrupt):
    bike.stop()
    print 'BREAK'
