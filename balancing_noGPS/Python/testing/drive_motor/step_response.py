# -*- coding: utf-8 -*-

"""
Measures the step response of the rear motor.
The test has been designed in a way such that the motor is spinning before a step input is given - this represents the actual system implementation
"""

import csv
import sys
import time
import numpy

sys.path.append('../../')
from bike import Bike

CONTROLLER_FREQUENCY = 50  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('shimano_step-%s.csv' % timestr, 'wb')

bike = Bike(debug=1)


def main():
    writer = csv.writer(RESULTS)
    writer.writerow(('Time', 'Desired Velocity', 'Measured Velocity'))  # headers

    try:

        DESIRED_VELOCITY = float(raw_input('Please enter a starting velocity'))
        bike.rear_motor.set_velocity(
            1)  # minimum input to produce movement. This allows to start the motor running for the test.
        raw_input('Press Enter when wheel is spinning')
        time_count = 0.0

        while True:
            start_time = time.time()

            if time_count > 10:
                bike.rear_motor.set_velocity(DESIRED_VELOCITY)

            velocity = bike.get_velocity()

            calculation_time = time.time() - start_time

            if calculation_time < TIME_CONSTANT:
                time.sleep(TIME_CONSTANT - calculation_time)
            time_count += time.time() - start_time

            # log data
            writer.writerow((time_count, DESIRED_VELOCITY, velocity))
            print 'Time = %f\tReference Velocity = %f\tMeasured Velocity = %f' % (
                time_count, DESIRED_VELOCITY, velocity)

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.stop()
        RESULTS.close()

    finally:
        bike.stop()
        RESULTS.close()


if __name__ == "__main__":
    print 'Rear Wheel Velocity Test'
    raw_input('Press Enter to start')
    main()
