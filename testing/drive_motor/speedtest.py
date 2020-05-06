# -*- coding: utf-8 -*-

"""
Finds the relationship between PWM input and wheel speed (m/s).
This is achieved by sending increasing PWM values and recording the velocity.
Hysteresis will also be measured by decreasing the PWM signal and observing the speed.
Used for testing Turnigy motor controller. Test file is outdated
"""

import csv
import sys
import time
import numpy

sys.path.append('../../')
from bike import Bike

PWM_STOP = 1050
PWM_MAX = 2100
CONTROLLER_FREQUENCY = 10  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
ON_TIME = 5.0
SETTLE_TIME = 2.0

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('shimano_speed-%s.csv' % timestr, 'wb')

bike = Bike(debug=1)


def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush()  # As suggested by Rom Ruben (see:
    # http://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console/27871113#comment50529068_27871113)


def main():
    try:
        print 'Rear Wheel Velocity Test'
        raw_input('Press Enter to start')

        writer = csv.writer(RESULTS)
        writer.writerow(('PWM', 'Velocity'))  # headers

        # accelerate
        for j in range(PWM_STOP, PWM_MAX + 1, 10):
            tock = 0.0
            velocity_array = []

            tick = time.time()
            bike.rear_motor.rear_set_pwm(j)
            while tock < SETTLE_TIME:  # settling time
                bike.rear_motor.rear_set_pwm(j)
                time.sleep(TIME_CONSTANT)
                tock = time.time() - tick

            tick = time.time()  # reset time count
            tock = 0.0

            while tock < ON_TIME:
                bike.rear_motor.rear_set_pwm(j)
                velocity_array.append(bike.get_velocity())
                time.sleep(TIME_CONSTANT)
                tock = time.time() - tick

            velocity_average = numpy.average(velocity_array)
            writer.writerow((j, velocity_average))
            print 'PWM = %d\tVelocity = %f' % (j, velocity_average)
            progress((j - PWM_STOP) / 10, len(range(PWM_STOP, PWM_MAX + 1, 10)), "Acceleration Phase")


    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.stop()

    finally:
        bike.stop()
        RESULTS.close()


if __name__ == "__main__":
    main()
