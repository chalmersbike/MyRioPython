# -*- coding: utf-8 -*-

"""
Drive forward for a specified time then stop
"""

import sys
import time
import csv

sys.path.append('../../')
from bike import Bike

CONTROLLER_FREQUENCY = 50  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
REF_VELOCITY = 3.0

bike = Bike(debug=1)


def main():
    try:

        print 'Rear Wheel Velocity Test'
        RUN_TIME = 50.0
        raw_input('Press Enter to start')

        timestr = time.strftime("%Y%m%d-%H%M%S")
        RESULTS = open('shimano_%s.csv' % (timestr), 'wb')

        writer = csv.writer(RESULTS)
        writer.writerow(('Vel=%f' % REF_VELOCITY))

        writer.writerow(('Time', 'Reference Velocity', 'Measured Velocity'))  # headers

        bike.rear_motor.rear_set_pwm(1300)  # 1260 Corresponds to 3600 RPM which is minimum speed for ESCON 50/4

        raw_input('Press Enter when wheel is spinning')
        time.sleep(2)
        time_count = 0.0

        while time_count < RUN_TIME:
            start_time = time.time()
            bike.set_velocity(REF_VELOCITY)
            velocity = bike.get_velocity()
            calculation_time = time.time() - start_time

            if calculation_time < TIME_CONSTANT:
                time.sleep(TIME_CONSTANT - calculation_time)
            time_count += time.time() - start_time

            # log data
            writer.writerow((time_count, REF_VELOCITY, velocity))
            print 'Time = %f\tReference Velocity = %2f\tMeasured Velocity = %f' % (
                time_count, REF_VELOCITY, velocity)

    except (KeyboardInterrupt, ValueError):
        print 'KEYBOARD BREAK'
        bike.stop()
        RESULTS.close()

    finally:
        print "Test has finished"
        bike.stop()
        RESULTS.close()


if __name__ == "__main__":
    while True:
        main()
