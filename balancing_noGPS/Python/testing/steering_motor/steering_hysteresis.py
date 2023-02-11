# -*- coding: utf-8 -*-

"""
Finds the motor hysteresis. This is achieved by sending PWM values and slowly accelerating then decelerating
TEST IS PERFORMED WITH HANDLEBAR REMOVED TO PREVENT DAMAGE
"""

import csv
import sys
import time

sys.path.append('../../')
from bike import Bike

ON_TIME = 3.0
SETTLE_TIME = 2.0

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('steering_hysteresis-%s.csv' % timestr, 'wb')

bike = Bike(debug=1)


def calculate_rps(position_initial, position_final, time_count):
    return (position_final - position_initial) / time_count


def main():
    try:
        writer = csv.writer(RESULTS)
        writer.writerow(('PWM', 'RPS'))  # headers

        # POSITIVE ROTATION
        # accelerate
        for j in range(1580, 2100 + 1, 10):  #
            bike.steering_motor.steer_set_pwm(j)
            time.sleep(SETTLE_TIME)  # settling time
            position_initial = bike.get_handlebar_angle()
            time.sleep(ON_TIME)  # motor ON time
            position_final = bike.get_handlebar_angle()
            rps = calculate_rps(position_initial, position_final, time_count=ON_TIME)
            writer.writerow((j, rps))
            print 'PWM = %f\tRPS = %f' % (j, rps)

        # decelerate
        for j in range(2100, 1580 - 1, -10):
            bike.steering_motor.steer_set_pwm(j)
            time.sleep(SETTLE_TIME)  # settling time
            position_initial = bike.get_handlebar_angle()
            time.sleep(ON_TIME)  # motor ON time
            position_final = bike.get_handlebar_angle()
            rps = calculate_rps(position_initial, position_final, time_count=ON_TIME)
            writer.writerow((j, rps))
            print 'PWM = %f\tRPS = %f' % (j, rps)

        # NEGATIVE ROTATION
        # accelerate
        for j in range(1580, 1050 - 1, -10):  #
            bike.steering_motor.steer_set_pwm(j)
            time.sleep(SETTLE_TIME)  # settling time
            position_initial = bike.get_handlebar_angle()
            time.sleep(ON_TIME)  # motor ON time
            position_final = bike.get_handlebar_angle()
            rps = calculate_rps(position_initial, position_final, time_count=ON_TIME)
            writer.writerow((j, rps))
            print 'PWM = %f\tRPS = %f' % (j, rps)

        # decelerate
        for j in range(1050, 1580 + 1, 10):
            bike.steering_motor.steer_set_pwm(j)
            time.sleep(SETTLE_TIME)  # settling time
            position_initial = bike.get_handlebar_angle()
            time.sleep(ON_TIME)  # motor ON time
            position_final = bike.get_handlebar_angle()
            rps = calculate_rps(position_initial, position_final, time_count=ON_TIME)
            writer.writerow((j, rps))
            print 'PWM = %f\tRPS = %f' % (j, rps)

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
