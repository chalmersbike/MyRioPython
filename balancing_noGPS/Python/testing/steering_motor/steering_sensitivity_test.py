# -*- coding: utf-8 -*-

"""
TESTS THE MOTOR ANGULAR VELOCITIES FOR GIVEN PWM VALUES
MAKE SURE MOTOR IS DISCONNECTED FROM HANDLEBARS""
"""

import csv
import sys
import time

import numpy as np

sys.path.append('../../')

from bike import Bike

PWD_MAP_CSV_FILE_NAME = 'pwmmap2.csv'
RESULTS = open('speed_pwm_test.csv', 'wb')
SLEEP_TIME = 10

bike = Bike()
bike.controller.stop() # disable safety limits

def progress(count, total, status=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush()  # As suggested by Rom Ruben (see: http://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console/27871113#comment50529068_27871113)


# Read PWM values
with open(PWD_MAP_CSV_FILE_NAME, 'r') as csvfile:
    pwm_map = dict([(int(pwm), character.strip(),)
                         for pwm, character in list(csv.reader(csvfile))[1:]])
    available_pwm_signals = np.array(sorted(pwm_map.keys()))

# Iterate through and write the results into a different CSV
i = 0
try:
    writer = csv.writer(RESULTS)
    writer.writerow(('PWM', 'ANGVEL')) # headers

    # Forward Values
    for j in available_pwm_signals:

        bike.steering_motor.set_pwm(j)
        time.sleep(1) # start motor for 1s to settle

        original_angle = bike.encoder.get_angle()
        time.sleep(SLEEP_TIME)
        bike.steering_motor.stop()
        final_angle = bike.encoder.get_angle()

        angular_velocity = (final_angle - original_angle) / SLEEP_TIME

        writer.writerow((j,angular_velocity))
        progress(i,len(available_pwm_signals),"working")
        i += 1
        time.sleep(1)

finally:
    RESULTS.close()
