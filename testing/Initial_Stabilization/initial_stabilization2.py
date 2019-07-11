# -*- coding: utf-8 -*-

"""
Drive forward for a specified time, then apply a step steering input, then stop
"""

import sys
import time
import csv

sys.path.append('../../')
from bike import Bike
from math import pi as PI

CONTROLLER_FREQUENCY = 50  # Hz
TIME_CONSTANT = 1. / CONTROLLER_FREQUENCY
REF_VELOCITY=2.0
STEER_TIME=9.0
STRAIGHT_TIME_AT_THE_END=5.0
STEERING_SPEED=-0.4
MAX_HANDLEBAR_ANGLE=PI/4
MIN_HANDLEBAR_ANGLE=-PI/4
REFERENCE_STEERING_ANGLE=-19*(PI/180.0)

bike = Bike(debug=1)


def main():
    try:


        print 'Initial Stabilization Test'
        RUN_TIME = 40.0
        raw_input('Press Enter to start')

        timestr = time.strftime("%Y%m%d-%H%M%S")
        RESULTS = open('Initial_stabilization_%s.csv' % (timestr), 'wb')

        writer = csv.writer(RESULTS)
        writer.writerow(('Vel=%f' % REF_VELOCITY))
        writer.writerow(('Steering Speed [RAD]=%f' % STEERING_SPEED))
        writer.writerow(('Steering Time [sec]=%f' % STEER_TIME))

        writer.writerow(('Time', 'Reference Velocity', 'Measured Velocity', 'Handlebar Angle'))  # headers

        bike.rear_motor.rear_set_pwm(1300)  # 1260 Corresponds to 3600 RPM which is minimum speed for ESCON 50/4

        raw_input('Press Enter when wheel is spinning')
        time.sleep(2)
        time_count = 0.0


        while time_count < RUN_TIME:
            start_time = time.time()
            bike.set_velocity(REF_VELOCITY)
            velocity = bike.get_velocity()
            calculation_time = time.time() - start_time
            handlebar_angle = bike.get_handlebar_angle()

            if (time_count >= RUN_TIME - 2 * STEER_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        time_count < RUN_TIME - STEER_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        handlebar_angle > REFERENCE_STEERING_ANGLE):
                bike.set_handlebar_angular_velocity(STEERING_SPEED)
                keep_handlebar_angle_within_safety_margins()
            elif (time_count >= RUN_TIME - 2 * STEER_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        time_count < RUN_TIME - STEER_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        handlebar_angle <= REFERENCE_STEERING_ANGLE):
                bike.set_handlebar_angular_velocity(0.0)
                keep_handlebar_angle_within_safety_margins()

            if (time_count >= RUN_TIME - STEER_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        time_count < RUN_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        handlebar_angle < 0):
                bike.set_handlebar_angular_velocity(-1.5*STEERING_SPEED)
                keep_handlebar_angle_within_safety_margins()
            elif (time_count >= RUN_TIME - STEER_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        time_count < RUN_TIME - STRAIGHT_TIME_AT_THE_END) and (
                        handlebar_angle >= 0):
                bike.set_handlebar_angular_velocity(0.0)
                keep_handlebar_angle_within_safety_margins()

            if (time_count >= RUN_TIME - STRAIGHT_TIME_AT_THE_END) :
                bike.set_handlebar_angular_velocity(0.0)
                keep_handlebar_angle_within_safety_margins()

            if calculation_time < TIME_CONSTANT:
                time.sleep(TIME_CONSTANT - calculation_time)
            time_count += time.time() - start_time

            # log data
            writer.writerow((time_count, REF_VELOCITY, velocity, handlebar_angle))
            print 'Time = %f\tReference Velocity = %2f\tMeasured Velocity = %f' % (
                time_count, REF_VELOCITY, velocity)

    except (KeyboardInterrupt, ValueError):
        print 'KEYBOARD BREAK'
        bike.stop()
        RESULTS.close()

    finally:
        print "Test has finished"
        bike.stop_all()
        RESULTS.close()


def keep_handlebar_angle_within_safety_margins():
    handlebar_angle = bike.get_handlebar_angle()
    print 'Handlebar Angle = %f' % handlebar_angle
    global upperSaturation
    global lowerSaturation
    if handlebar_angle > MAX_HANDLEBAR_ANGLE:
        print 'Exceeded MAX_HANDLEBAR_ANGLE'
        bike.set_handlebar_angular_velocity(0)
        upperSaturation = True
    elif handlebar_angle < MIN_HANDLEBAR_ANGLE:
        print 'Exceeded MIN_HANDLEBAR_ANGLE'
        bike.set_handlebar_angular_velocity(0)
        lowerSaturation = True
    else:
        upperSaturation = False
        lowerSaturation = False


if __name__ == "__main__":
    while True:
        main()
