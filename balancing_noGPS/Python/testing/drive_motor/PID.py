# -*- coding: utf-8 -*-

"""
Performs a closed loop PID control test for the bike rear motor
"""

import csv
import sys
import time
import numpy

sys.path.append('../../')
from bike import Bike
from utils import PID

CONTROLLER_FREQUENCY = 20  # Hz
TIME_CONSTANT = 1.0 / CONTROLLER_FREQUENCY
REF_VELOCITY = 2.0  # m/s
PVAL = 3.0
IVAL = 1.0
DVAL = 0.0
WINDUP = REF_VELOCITY


def main():
    print 'Rear Wheel Velocity PID Test'
    raw_input('Press Enter to start')

    timestr = time.strftime("%Y%m%d-%H%M%S")
    RESULTS = open('shimano_P=%.2fI=%.2fD=%.2f-%s.csv' % (PVAL, IVAL, DVAL, timestr), 'wb')

    writer = csv.writer(RESULTS)
    writer.writerow(('P = %f' % PVAL, 'I = %f' % IVAL, 'D = %f' % DVAL))

    writer.writerow(('Time', 'Reference Velocity', 'Measured Velocity', 'Control Input', 'P Term', 'I Term'))  # headers

    bike.rear_motor.rear_set_pwm(1300)  #1260 Corresponds to 3600 RPM which is minimum speed for ESCON 50/4
    raw_input('Press Enter when wheel is spinning')
#    bike.rear_motor.rear_set_pwm(1050)
#    raw_input('Press Enter when wheel is stopped')
    time.sleep(2)
    time_count = 0.0
    pid.ITERM = 0.0

    try:
        while time_count < 12.0:
            start_time = time.time()

            velocity = bike.get_velocity()
            u = pid.update(velocity)
            bike.set_velocity(u)

            calculation_time = time.time() - start_time

            if calculation_time < TIME_CONSTANT:
                time.sleep(TIME_CONSTANT - calculation_time)
            time_count += time.time() - start_time

            # log data
            writer.writerow((time_count, REF_VELOCITY, velocity, u, pid.PTerm, pid.ITerm))
            print 'Time = %f\tReference Velocity = %2f\tMeasured Velocity = %f\tControl Input= %f' % (
                time_count, REF_VELOCITY, velocity, u)

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.stop()
        RESULTS.close()

    finally:
        bike.stop()
        RESULTS.close()


if __name__ == "__main__":
    bike = Bike(debug=1)
    pid = PID(P=PVAL, I=IVAL, D=DVAL)
    pid.setSampleTime(TIME_CONSTANT)
    pid.setReference(REF_VELOCITY)
    pid.setWindup(WINDUP)
    main()
