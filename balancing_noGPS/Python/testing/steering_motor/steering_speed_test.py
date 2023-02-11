# -*- coding: utf-8 -*-

"""
Finds the motor velocity for a given PWM value. Sweeps between limits
"""

import sys
import time
from math import pi as PI

sys.path.append('../../')
from bike import Bike

MAX_HANDLEBAR_ANGLE = PI/4

bike = Bike(debug=1)


def keep_handlebar_angle_within_safety_margins():
    handlebar_angle = bike.get_handlebar_angle()
    print 'Handlebar Angle = %f' % handlebar_angle
    global upperSaturation
    if abs(handlebar_angle) > MAX_HANDLEBAR_ANGLE:
        print 'Exceeded MAX_HANDLEBAR_ANGLE'
        # self.set_handlebar_angular_acceleration(0)
        bike.set_handlebar_angular_velocity(0)
        upperSaturation = True


def calculate_rps(position_initial, position_final, time_count):
        return (position_final - position_initial) / time_count


def main():
    global upperSaturation
    time_count = 0.0
    rps = 0.0

    try:
        upperSaturation = False
        pwm_input = input('Please enter a PWM value: ')
        position_initial = bike.get_handlebar_angle()
        bike.steering_motor.set_pwm(pwm_input)

        while upperSaturation == False:
            keep_handlebar_angle_within_safety_margins()
            time.sleep(0.01)
            time_count += 0.01

        bike.set_handlebar_angular_velocity(0)
        position_final = bike.get_handlebar_angle()
        rps = calculate_rps(position_initial, position_final, time_count)

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.set_handlebar_angular_velocity(0)

    finally:
        bike.set_handlebar_angular_velocity(0)
        print 'RPS = %f' % rps


if __name__ == "__main__":
    while True:
        main()
