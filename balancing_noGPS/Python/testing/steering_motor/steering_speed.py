# -*- coding: utf-8 -*-

"""
Compares the steering input vs measurements in terms of steering speed [rad/s]
TEST IS PERFORMED WITH HANDLEBAR REMOVED TO PREVENT DAMAGE
"""

import csv
import sys
import time

sys.path.append('../../')
from bike import Bike

ON_TIME = 5.0
SETTLE_TIME = 2.0

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('steering_input_output_check-%s.csv' % timestr, 'wb')

bike = Bike(debug=1)


def calculate_rps(position_initial, position_final, time_count):
    return (position_final - position_initial) / time_count


def main():
    try:
        writer = csv.writer(RESULTS)
        writer.writerow(('Input [RPS]', 'Output [RPS]'))  # headers

        # POSITIVE ROTATION
        # accelerate
        for j in range(-30, 30, 1):  #
            j/=10.0
            bike.steering_motor.set_angular_velocity(j)
            time.sleep(SETTLE_TIME)  # settling time
            position_initial = bike.get_handlebar_angle()
            time.sleep(ON_TIME)  # motor ON time
            position_final = bike.get_handlebar_angle()
            rps = calculate_rps(position_initial, position_final, time_count=ON_TIME)
            writer.writerow((j, rps))
            print 'Reference Input = %f\tOutput = %f' % (j, rps)

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.set_handlebar_angular_velocity(0)

    finally:
        bike.set_handlebar_angular_velocity(0)
        RESULTS.close()


if __name__ == "__main__":
    print 'Make sure handlebar is disconnected!'
    raw_input('Press Enter to start')
    main()
