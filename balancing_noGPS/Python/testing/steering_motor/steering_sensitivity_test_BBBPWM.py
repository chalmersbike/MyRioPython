# -*- coding: utf-8 -*-

"""
TESTS THE MOTOR ANGULAR VELOCITIES FOR GIVEN PWM VALUES
MAKE SURE MOTOR IS DISCONNECTED FROM HANDLEBARS""
"""

import csv
import sys
import time

import Adafruit_BBIO.PWM as PWM

from src.bike import Bike

bike = Bike()
bike.controller.stop() # disable safety limits
PWM.start("P8_13", duty_cycle=0, frequency=50) # 50Hz = 20ms period

PWD_MAP_CSV_FILE_NAME = 'pwmmap2.csv'
RESULTS = open('speed_pwm_test.csv', 'wb')
MOTOR_ON_TIME = 10

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
    # available_pwm_signals = np.array(sorted(pwm_map.keys()))
    available_pwm_signals = range(1186,2450+1,5)

# Iterate through and write the results into a different CSV
i = 0
try:
    writer = csv.writer(RESULTS)
    writer.writerow(('PWM', 'ANGVEL')) # headers

    for j in available_pwm_signals:

        duty_cycle = ((j/1000.0) / 20) * 100  # convert to ms, then find % of total PWM period

        PWM.set_duty_cycle("P8_13", duty_cycle)
        time.sleep(2) # start motor for 1s to settle

        original_angle = bike.encoder.get_angle()
        time.sleep(MOTOR_ON_TIME)
        PWM.set_duty_cycle("P8_13", 0)  # stop motor
        final_angle = bike.encoder.get_angle()

        angular_velocity = (final_angle - original_angle) / MOTOR_ON_TIME

        writer.writerow((j,angular_velocity))
        progress(i,len(available_pwm_signals),"working")
        i += 1

        # proportional rest time
        if duty_cycle < 30.0 or duty_cycle > 70.0:
            proportional_rest = 10
        else: proportional_rest = 5

        time.sleep(proportional_rest)
        print 'Testing PWM signal = %s, DC = %s, AV = %s %%' % (j, duty_cycle, angular_velocity)

except KeyboardInterrupt, SystemExit:
    PWM.set_duty_cycle("P8_13", 0)  # stop motor

finally:
    RESULTS.close()
    PWM.set_duty_cycle("P8_13", 0)  # stop motor
    PWM.cleanup()
