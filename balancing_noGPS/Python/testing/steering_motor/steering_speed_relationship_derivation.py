# -*- coding: utf-8 -*-

"""
This code is for finding the relation between ms signal sent to motor and measured velocity [rad/s]
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
RESULTS = open('steering_motor_relationship_derivation_ms-%s.csv' % timestr, 'wb')

bike = Bike(debug=1)


def calculate_rps(position_initial, position_final, time_count):
    return (position_final - position_initial) / time_count


def main():
    try:
        writer = csv.writer(RESULTS)
        writer.writerow(('Input [ms]', 'Output Speed [RPS]'))  # headers

        # POSITIVE ROTATION
        # accelerate
        for j in range(970, 1130, 5):  #
            bike.steering_motor.steer_set_pwm(j)
            time.sleep(SETTLE_TIME)  # settling time
            position_initial = bike.get_handlebar_angle()
            time.sleep(ON_TIME)  # motor ON time
            position_final = bike.get_handlebar_angle()
            rps = calculate_rps(position_initial, position_final, time_count=ON_TIME)
            writer.writerow((j, rps))
            print 'Input [ms] = %f\tOutput Speed [rad/s] = %f' % (j, rps)

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.steering_motor.steer_set_pwm(1055)

    finally:
        bike.steering_motor.steer_set_pwm(1055)
        RESULTS.close()


if __name__ == "__main__":
    print 'Make sure handlebar is disconnected!'
    raw_input('Press Enter to start')
    main()
