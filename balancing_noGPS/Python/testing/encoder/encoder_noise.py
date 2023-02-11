# This test holds the steering motor at a constant velocity and reads encoder values
# Used to find errors in encoder reads.

import csv
import sys
import time
from math import pi as PI

import numpy

sys.path.append('../../')
from bike import Bike

CONTROLLER_FREQUENCY = 50  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
MAX_HANDLEBAR_ANGLE = PI / 6  # rad
MIN_HANDLEBAR_ANGLE = -MAX_HANDLEBAR_ANGLE  # rad

timestr = time.strftime("%Y%m%d-%H%M%S")
RESULTS = open('BikeData-%s.csv' % timestr, 'wb')

bike = Bike(debug=1)

if __name__ == '__main__':

    ESTOP = False
    global_time = 0.0
    time_count = 0.0
    delta = 0.0

    try:
        writer = csv.writer(RESULTS)
        writer.writerow(('Time', 'Desired Angular Velocity', 'Steering Angle', 'Steering Anglular Velocity'))

        raw_input('Press ENTER to confirm that handlebars are DISCONNECTED')
        # input_speed = float(raw_input('Type a motor speed [rad/s]: '))

        for input_speed in numpy.arange(0, -7, -1):
            print 'Testing input speed %f' % input_speed

            raw_input('Press ENTER to start')
            bike.set_handlebar_angular_velocity(input_speed)

            while time_count < 20.0:
                ESTOP = bike.emergency_stop_check()
                if ESTOP:
                    print "E-STOP"
                    bike.set_handlebar_angular_velocity(0)
                    break

                delta = bike.get_handlebar_angle()

                writer.writerow((time_count, input_speed, delta))
                print('Time = %f\tDesired Angular Velocity= %f\tSteering Angle = %f' % (time_count, input_speed, delta))
                time.sleep(TIME_CONSTANT)
                time_count += TIME_CONSTANT
                global_time += TIME_CONSTANT
            time_count = 0.0
            bike.set_handlebar_angular_velocity(0)

    except (ValueError, KeyboardInterrupt, SystemExit):
        bike.set_handlebar_angular_velocity(0)
        print 'BREAK'
