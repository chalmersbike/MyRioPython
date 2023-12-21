# ISE stands for IMU Steering and Encoder Test.
from param import *
from sensors import Encoder, HallSensor, SafetyStop
from sensors import IMU
from actuators import DriveMotor, SteeringMotor
# from actuators import DriveMotor
from time import sleep
from PID import PID
import time
from nifpga import Session
from numpy import rad2deg, deg2rad, sign, abs
import math
import re
import traceback
import signal
import threading
import sys
import pyvisa
import csv
# import pyvesc
# import pysnooper
from sensors import GPS
# from vesc_gps_resource_v2 import VESC_GPS
from vesc_gps_resource_v3 import VESC_GPS
from numpy import rad2deg

GAIN_SCHEDULING_K = True
# GAIN_SCHEDULING_K = False

a_x_assumed = 0.5  # m/s^2
drive_current = 11.0
# drive_current = 13.0
# break_drive_current = -10.0
break_drive_current = 0.0
startup_current = 40  # A
# startup_current = 10  # A
# idle_drive_current = 0.0  # A
idle_drive_current = break_drive_current  # A

# Roll-Steering Angle
Kp_roll_str = 7.0
Ki_roll_str = 0.0

# Rollrate-steering rate Inner loop
Kp_balancing = 5.0
# Kp_strAngle = 15.0
# Ki_strAngle = 1

# Roll angle - Roll rate ref Outer loop
Kp_roll_outer_strrate = 1.3

# Steering Angle controller
Kp_strAngle = 4.0
Ki_strAngle = 0.1

# An instance of K_1 and K_2 for acceleration control
# K_1_rollstr = 20.2644 / (2.1563 * 0.3 ** 2 + 0.9069 * a_x_assumed)
K_1_rollstr = 7.0
# K_2_rollratestr = 3.0
K_2_rollratestr = 0.0
# GAIN_SCHEDULING_K = True
# GAIN_SCHEDULING_K = False


start_up_interval = 5.0  # Seconds
ACCELERATING_FLAG = True
DECELERATING_FLAG = False
EXP_STOP_FLAG = False
REPEAT_ACC_BREAK_FLAG = True

# switch to acceleration mode when lower than it
acceleration_start_speed = 0.8
# Use High startup current when lower than it
stationary_threshold_speed = 0.3
# Once reached, switch to idle current (But not deceleration)
# acceleration_stop_speed = 2
acceleration_stop_speed = 2.6

# Start breaking if lower than it
# breaking_start_speed = acceleration_stop_speed
breaking_start_speed = 10.0




# Threshold switching to balancing Controller
ctrl_switch_speed = acceleration_stop_speed
# ctrl_switch_speed = 3.0  # Threshold switching to balancing Controller



# Switch when low speed and STR or Roll is too high
MAX_ROLL_DECELERATION = deg2rad(5.0)
LOW_SPEED_FOR_STR_ROLL = 1.4
ACC_SWITCHING_STR_ANGLE = deg2rad(25)


MAX_HANDLEBAR_ANGLE = deg2rad(45)
MAX_HANDLEBAR_SPEED = 0.7
MAX_ROLL_ANGLE = deg2rad(20)


rollrate_balancing_start_time = time.time()
rollrate_balancing_duration = 8  # secs



IMU_SWITCH = 1
STR_SWITCH = 1
ENC_SWITCH = 1
DRIVE_SWITCH = 1
HALLSENSOR = 1
EMERGENCY = 1
GPS_USE = 1
RECORD_DATA = 1
ROLL_CTRL_SWITCH = True


with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_5.lvbitx", "RIO0") as session:  # balancing Control V5


    # print("Session started, Wait for 3 secs")
#Declaration of the session registers
    rm = pyvisa.ResourceManager()
    strMotorObj = SteeringMotor(session)
    PositiveStrSpeed = 0.1
    NegativeStrSpeed = -0.8


    if IMU_SWITCH:
        phi = 0.0
        imu_object = IMU(session)
    if ENC_SWITCH:
        steeringAngle = 0.0
        encoder_object = Encoder(session)
    if DRIVE_SWITCH or GPS_USE:
        vesc_speed = 0.0
        gps_vesc_object = VESC_GPS(session)
        # gps_vesc_object.heart_pipe_parent.send('start_heart_beat')
        # gps_vesc_object.heart_pipe_parent.send(2.0)
        # print("wait for 3 secs for VESC initialization!")
        # time.sleep(3)
        # gps_vesc_object.heart_pipe_parent.send(drive_current)
        gps_vesc_object.heart_pipe_parent.send(0.0)

        # acceleration_binary_code = gps_vesc_object.set_current_binary(startup_current)
        acceleration_binary_code = gps_vesc_object.set_velocity_binary(acceleration_start_speed+0.1)
        print(acceleration_binary_code)
    if STR_SWITCH:
        steeringSpeedCmd = 0.0
        strMotorObj.stop()
        strMotorObj.set_angular_velocity(steeringSpeedCmd)

        # fpga_SteeringEnable.write(True)
        # print("Steering is NOW activated")
        # PrevStrSpeed = PositiveStrSpeed

    if HALLSENSOR:
        hall_sensor = HallSensor(session)
    if EMERGENCY or DRIVE_SWITCH or RECORD_DATA:
        estop_object = SafetyStop(session)
        if estop_object.button_check():
            input('\n Emergency Pressed. Will not continue if remain set after 5 s count')
    if ROLL_CTRL_SWITCH:
        # Roll ref -> Steering Angle Ref
        roll_ctrl = PID()
        roll_ctrl.setSampleTime(pid_balance_sample_time)
        roll_ctrl.setReference(pid_balance_reference)
        roll_ctrl.setKp(Kp_roll_str)
        roll_ctrl.setKi(Ki_roll_str)

        # Steering Angle Ref -> Steering Rate Ref 1
        strAngle_ctrl = PID()
        strAngle_ctrl.setSampleTime(pid_balance_sample_time)
        strAngle_ctrl.setReference(pid_balance_reference)
        strAngle_ctrl.setKp(Kp_strAngle)
        strAngle_ctrl.setKi(Ki_strAngle)

        # Roll Rate -> Steering Rate Ref 2
        rollrate_inner_ctrl = PID()
        rollrate_inner_ctrl.setSampleTime(pid_balance_sample_time)
        rollrate_inner_ctrl.setReference(pid_balance_reference)
        rollrate_inner_ctrl.setKp(Kp_balancing)

        roll_outer_ctrl = PID()
        roll_outer_ctrl.setSampleTime(pid_balance_sample_time)
        roll_outer_ctrl.setReference(pid_balance_outerloop_reference)
        roll_outer_ctrl.setKp(Kp_roll_outer_strrate)



    if RECORD_DATA:
        log_header_str = ['Time']
        if IMU_SWITCH:
            log_header_str += ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'Roll']
        if ENC_SWITCH:
            log_header_str += ['SteeringAngle']
        if DRIVE_SWITCH:
            log_header_str += ['vesc_speed', 'vesc_v_in', 'vesc_avg_motor_current', 'vesc_avg_input_current']
        if GPS_USE:
            log_header_str += ['x_estimated', 'y_estimated',
                               'latitude', 'longitude',
                               'status', 'NMEA timestamp',
                               'v_gps', 'course_gps',
                               ]
        if STR_SWITCH:
            log_header_str += ['ControlInput', 'SteerMotorCurrent']
        if HALLSENSOR:
            log_header_str += ['MeasuredVelocity']
        if ROLL_CTRL_SWITCH:
            log_header_str += ['a_x_guessed', 'K_1_phi', 'K_2_phidot', 'SteeringAngleRef']


    descr = input('\nPress ENTER to calibrate roll estimation for 3 secs. ')
    # Steering loop
    for i in range(0, 300):
        phi, phi_gyro, gx, gy, gz, ax, ay, az, sensor_read_timing = \
            imu_object.get_imu_data(vesc_speed, steeringAngle, phi)
        time.sleep(0.005)
    # Steering loop
    descr = input('\nPress ENTER to start the experiment. ')

    for i in range(0, int(math.ceil(start_up_interval))):
        time.sleep(1)
        print("Experiment starting in %is" % (int(math.ceil(start_up_interval)) - i))

    if RECORD_DATA:
        timestr = time.strftime("%Y%m%d-%H%M%S")
        csv_path = './ExpData_%s/I_test_loaded_%sA_%s.csv' % (bike, "{:.2f}".format(drive_current).replace(".", "_"), timestr)
        results_csv = open(csv_path, 'w')
        writer = csv.writer(results_csv)
        writer.writerow(log_header_str)
    strMotorObj.enable()
    try:
        start_time_exp = time.time()
        # while True:
        while not EXP_STOP_FLAG:
            loop_start_time = time.time()
            if IMU_SWITCH:
                # t_startLoop = time.time()
                 phi, phi_gyro, gx, gy, gz, ax, ay, az, sensor_read_timing = \
                     imu_object.get_imu_data(vesc_speed, steeringAngle, phi)
                 if abs(phi) > MAX_ROLL_ANGLE:
                    print('TOO LARGE ROLL!!!!')
                   # EXP_STOP_FLAG = True
                 # print(phi)
                # t_start_IMU = time.time()
                #
                # # print('IMU : t = %f ; gx = %f ; gy = %f ; gz = %f ; ax = %f ; ay = %f ; az = %f' %(time.time()-t_start,gx,gy,gz,ax,ay,az))
                # t_end_IMU = time.time()
            if ENC_SWITCH:
                # print('ENC Reading: \n')
                # fpga_EncoderCounts.read()
                # print(rad2deg(-fpga_EncoderCounts.read() * steeringEncoder_TicksToRadianRatio))
                steeringAngle = encoder_object.get_angle()
                if abs(steeringAngle) > MAX_HANDLEBAR_ANGLE + 0.1:
                    print('TOO HIGH STEERING ANGLE!!!')
                    EXP_STOP_FLAG = True
                    print(rad2deg(steeringAngle))

            if ROLL_CTRL_SWITCH:
                if ACCELERATING_FLAG and vesc_speed < ctrl_switch_speed:
                    # print(phi)
                    ref_strAngle = roll_ctrl.update(phi)
                    # print(ref_strAngle)
                    if GAIN_SCHEDULING_K:
                        K_1_rollstr = 9.3980/(vesc_speed ** 2 + 0.4206 * a_x_assumed)
                    K_2_rollratestr = 0.0
                    ref_strAngle = -(K_1_rollstr * phi + K_2_rollratestr * gx)

                    if abs(ref_strAngle) > MAX_HANDLEBAR_ANGLE:
                        ref_strAngle = sign(ref_strAngle) * MAX_HANDLEBAR_ANGLE
                        print('COMMAND Str Angle Out of Range!!!!')
                    strAngle_ctrl.setReference(ref_strAngle)
                    steeringSpeedCmd = strAngle_ctrl.update(steeringAngle)
                elif vesc_speed > ctrl_switch_speed:
                    roll_outer_ctrl.setReference(0.0)
                    rollrate_inner_ctrl.setReference(roll_outer_ctrl.update(phi))
                    steeringSpeedCmd = rollrate_inner_ctrl.update(gx)
                    K_1_rollstr = Kp_roll_outer_strrate
                    K_2_rollratestr = Kp_balancing
                elif DECELERATING_FLAG:
                    roll_outer_ctrl.setReference(-deg2rad(1.0))
                    rollrate_inner_ctrl.setReference(roll_outer_ctrl.update(phi))
                    steeringSpeedCmd = rollrate_inner_ctrl.update(gx)
                    if abs(steeringAngle) > MAX_HANDLEBAR_ANGLE:
                        if sign(steeringSpeedCmd) == sign(steeringAngle):
                            steeringSpeedCmd = 0.0
                else:
                    print('STRANGE that slow speed <2.0 but not in deceleration mode')



            if STR_SWITCH:

                strMotorObj.set_angular_velocity(steeringSpeedCmd)
                steeringCurrent = strMotorObj.read_steer_current()

            if GPS_USE:
                # gpsStartTime = time.time()
                dx, dy, lat, lon, gpsstatus, utc, speed_m_s, course = \
                    gps_vesc_object.get_position()

            if DRIVE_SWITCH:

                rpm, avg_motor_current, v_in, avg_input_current = gps_vesc_object.retrieve_vesc_data()
                vesc_speed = rpm / 608.0
                # print(rpm, v_in, avg_motor_current)
                if ACCELERATING_FLAG:
                    if vesc_speed < stationary_threshold_speed:
                        gps_vesc_object.heart_pipe_parent.send(acceleration_binary_code)
                    else:
                        gps_vesc_object.heart_pipe_parent.send(drive_current)
                elif not DECELERATING_FLAG:
                    gps_vesc_object.heart_pipe_parent.send(idle_drive_current)
                else:
                    gps_vesc_object.heart_pipe_parent.send(break_drive_current)
            if HALLSENSOR:
                hall_sensor_speed = hall_sensor.get_velocity(1)  # Dummy input

            if estop_object.button_check():  # True for pressed
                EXP_STOP_FLAG = True

            # Read the data from VESC!

            if RECORD_DATA:
                log_str = [
                    "{0:.5f}".format(time.time()-start_time_exp),
                ]
                if IMU_SWITCH:
                    log_str += [
                        "{0:.5f}".format(ax),
                        "{0:.5f}".format(ay),
                        "{0:.5f}".format(az),
                        "{0:.5f}".format(gx),
                        "{0:.5f}".format(gy),
                        "{0:.5f}".format(gz),
                        "{0:.5f}".format(phi),
                    ]
                if ENC_SWITCH:
                    log_str += [
                        "{0:.5f}".format(steeringAngle),
                    ]
                if DRIVE_SWITCH:
                    log_str += [
                        "{0:.5f}".format(vesc_speed),
                        "{0:.5f}".format(v_in),
                        "{0:.5f}".format(avg_motor_current),
                        "{0:.5f}".format(avg_input_current),
                    ]
                if GPS_USE:
                    log_str += [
                        "{0:.5f}".format(dx),
                        "{0:.5f}".format(dy),
                        "{0:.8f}".format(lat),
                        "{0:.8f}".format(lon),
                        gpsstatus,
                        utc,
                        "{0:.5f}".format(speed_m_s),
                        "{0:.5f}".format(course),
                    ]
                if STR_SWITCH:
                    log_str += [
                        "{0:.5f}".format(steeringSpeedCmd),
                        "{0:.5f}".format(steeringCurrent),
                    ]
                if HALLSENSOR:
                    log_str += [
                        "{0:.5f}".format(hall_sensor_speed),
                    ]
                if ROLL_CTRL_SWITCH:
                    log_str += [
                        "{0:.5f}".format(a_x_assumed),
                        "{0:.5f}".format(K_1_rollstr),
                        "{0:.5f}".format(K_2_rollratestr),
                        "{0:.5f}".format(ref_strAngle),
                    ]
                writer.writerow(log_str)

            # if fpga_ESTOP.read():
            #     raise Exception
            t_sleep = (0.01-time.time()+loop_start_time)
            if t_sleep > 0:
                time.sleep(t_sleep)
            if ACCELERATING_FLAG and vesc_speed > acceleration_stop_speed:
                ACCELERATING_FLAG = False
                print('Exit Acceleration!')
                strAngle_ctrl.clear()
                # rollrate_balancing_start_time = time.time()
            if not ACCELERATING_FLAG and not DECELERATING_FLAG and (vesc_speed < breaking_start_speed):
                print('Decelerating TRUE!!!')
                DECELERATING_FLAG = True
                ACCELERATING_FLAG = False
            if REPEAT_ACC_BREAK_FLAG and not ACCELERATING_FLAG and \
                    (vesc_speed < acceleration_start_speed+0.2 or
                     ((abs(phi) > MAX_ROLL_DECELERATION or
                       abs(steeringAngle) > ACC_SWITCHING_STR_ANGLE) and
                      vesc_speed < LOW_SPEED_FOR_STR_ROLL)):
                print('Reset to Acceleration!')
                DECELERATING_FLAG = False
                ACCELERATING_FLAG = True

        if STR_SWITCH:
            strMotorObj.stop()
            strMotorObj.set_angular_velocity(0.0)
        if DRIVE_SWITCH:
            gps_vesc_object.stop()
            gps_vesc_object.stop_GPS_loop.set()
            time.sleep(3)

    except Exception as e:
        if STR_SWITCH:
            strMotorObj.stop()
            strMotorObj.set_angular_velocity(0.0)
        if DRIVE_SWITCH:
            gps_vesc_object.stop()
            gps_vesc_object.stop_GPS_loop.set()
            time.sleep(3)

            print('Steering Control terminated')
        print(e)

        print('str(Exception):\t', str(Exception))
        print('str(e):\t\t', str(e))
        print('repr(e):\t', repr(e))

        # Get information about the exception that is currently being handled
        exc_type, exc_value, exc_traceback = sys.exc_info()
        print('e.message:\t', exc_value)
        print("Note, object e and exc of Class %s is %s the same." %
              (type(exc_value), ('not', '')[exc_value is e]))
        print('traceback.print_exc(): ', traceback.print_exc())
        print('traceback.format_exc():\n%s' % traceback.format_exc())

        import EasyTerminator
        session.abort()


