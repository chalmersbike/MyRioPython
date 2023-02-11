# -*- coding: utf-8 -*-

"""
Drive forward for a specified time then stop
"""

import sys
import time

sys.path.append('../../')
from bike import Bike

CONTROLLER_FREQUENCY = 50  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY

bike = Bike(debug=1)


def main():
    try:
        time_count = 0.0

        print 'Rear Wheel Velocity Test'
        RUN_TIME = float(raw_input('Please enter a run time: '))

        while time_count < RUN_TIME:
            start_time = time.time()
            bike.set_velocity(4)
            calculation_time = time.time() - start_time

            if calculation_time < TIME_CONSTANT:
                time.sleep(TIME_CONSTANT - calculation_time)
            time_count += time.time() - start_time

            print time_count

    except (KeyboardInterrupt, ValueError):
        print 'KEYBOARD BREAK'
        bike.stop()

    finally:
        print "Test has finished"
        bike.stop()


if __name__ == "__main__":
    while True:
        main()
