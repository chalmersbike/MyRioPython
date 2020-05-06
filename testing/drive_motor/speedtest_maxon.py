# -*- coding: utf-8 -*-

"""
Measures the RPM of the rear motor and the wheel speed. Used for testing Maxon motor controller
"""

import csv
import sys
import time
import numpy

sys.path.append('../../')
from bike import Bike

CONTROLLER_FREQUENCY = 50  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY

ON_TIME = 5.0
SETTLE_TIME = 2.0

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('shimano_speed-%s.csv' % timestr, 'wb')

bike = Bike(debug=True)


def main():
    try:

        bike.rear_motor.rear_set_pwm(1260)  # Corresponds to minimum speed, 3600 RPM
        raw_input('Press Enter when wheel is spinning')

        writer = csv.writer(RESULTS)
        writer.writerow(('Time', 'Input Signal', 'Measured RPM', 'Measured Velocity'))  # headers
        time_count = 0.0

        while True:

            # accelerate
            for j in range(1260, 2100 + 1, 10):
                tock = 0.0
                velocity = 0.0
                rpm = 0.0

                tick = time.time()
                bike.rear_motor.rear_set_pwm(j)
                while tock < SETTLE_TIME:  # settling time
                    bike.rear_motor.rear_set_pwm(j)
                    time.sleep(TIME_CONSTANT)
                    time_count += TIME_CONSTANT
                    tock = time.time() - tick

                tick = time.time()  # reset time count
                tock = 0.0

                while tock < ON_TIME:
                    bike.rear_motor.rear_set_pwm(j)
                    rpm = bike.rear_motor.read_motor_rpm()
                    velocity = bike.get_velocity()

                    time.sleep(TIME_CONSTANT)
                    time_count += TIME_CONSTANT
                    tock = time.time() - tick

                    writer.writerow((time_count, j, rpm, velocity))
                    print 'Time = %d\tInput = %f\tMotor RPM = %f\tWheel Velocity = %f' % (time_count, j, rpm, velocity)


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
