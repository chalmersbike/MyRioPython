# This test is used to measure the delay in the motor for a step input.

import csv
import numpy
import sys
import time
from math import pi as PI

sys.path.append('../../')
from bike import Bike

CONTROLLER_FREQUENCY = 50  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
MAX_HANDLEBAR_ANGLE = PI / 6  # rad
MIN_HANDLEBAR_ANGLE = -MAX_HANDLEBAR_ANGLE  # rad

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('SteeringDelay-%s.csv' % timestr, 'wb')

bike = Bike(debug=1)

if __name__ == '__main__':

    ESTOP = False
    time_count = 0.0
    global_time = 0.0
    delta = 0.0
    delta_prev = 0.0
    delta_dot = 0.0

    try:
        writer = csv.writer(RESULTS)
        writer.writerow(('Time', 'Desired Angular Velocity', 'Steering Angle', 'Steering Anglular Velocity'))

        print 'Testing positive scale 0-6.0'
        raw_input('Press ENTER to confirm that handlebars are DISCONNECTED')
        # input_speed = float(raw_input('Type a motor speed [rad/s]: '))

        for i in numpy.arange(0, 6.5, 0.5):
            print 'Testing input speed = %f' % i
            raw_input('Press ENTER to start')
            time_count = 0.0
            bike.set_handlebar_angular_velocity(i)

            while time_count < 4.0:
                ESTOP = bike.emergency_stop_check()
                if ESTOP:
                    print "E-STOP"
                    bike.set_handlebar_angular_velocity(0)
                    break

                delta_prev = delta
                delta = bike.get_handlebar_angle()
                delta_dot = (delta - delta_prev) / TIME_CONSTANT

                writer.writerow((global_time, i, delta, delta_dot))
                print('Desired Angular Velocity= %f\tSteering Angular Velocity = %f' % (i, delta_dot))
                time.sleep(TIME_CONSTANT)
                time_count += TIME_CONSTANT
                global_time += TIME_CONSTANT

            bike.set_handlebar_angular_velocity(0)

        bike.set_handlebar_angular_velocity(0)
        raw_input('Press ENTER to begin negative scale')
        # input_speed = float(raw_input('Type a motor speed [rad/s]: '))

        for i in numpy.arange(0, -6.5, -0.5):
            print 'Testing input speed = %f' % i
            raw_input('Press ENTER to start')
            time_count = 0.0
            bike.set_handlebar_angular_velocity(i)

            while time_count < 4.0:
                ESTOP = bike.emergency_stop_check()
                if ESTOP:
                    print "E-STOP"
                    bike.set_handlebar_angular_velocity(0)
                    break

                delta_prev = delta
                delta = bike.get_handlebar_angle()
                delta_dot = (delta - delta_prev) / TIME_CONSTANT

                writer.writerow((global_time, i, delta, delta_dot))
                print('Desired Angular Velocity= %f\tSteering Angular Velocity = %f' % (i, delta_dot))
                time.sleep(TIME_CONSTANT)
                time_count += TIME_CONSTANT
                global_time += TIME_CONSTANT

            bike.set_handlebar_angular_velocity(0)
        bike.set_handlebar_angular_velocity(0)

    except (ValueError, KeyboardInterrupt):
        bike.set_handlebar_angular_velocity(0)
        print 'BREAK'
