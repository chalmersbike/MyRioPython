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

drive_current = 9.0
# drive_current = 6.0
break_drive_current = -10.0
# break_drive_current = -2.0
# startup_current = 40  # A
startup_current = 10  # A
idle_drive_current = 0.0  # A
idle_drive_current = break_drive_current  # A

Kp_roll_str = 7.0
Ki_roll_str = 0.0
Kp_balancing = 5.0
# Kp_strAngle = 15.0
# Ki_strAngle = 1
Kp_strAngle = 6.0
Ki_strAngle = 0.2
start_up_interval = 5.0  # Seconds
ACCELERATING_FLAG = True
DECELERATING_FLAG = False
EXP_STOP_FLAG = False
REPEAT_ACC_BREAK_FLAG = True

# switch to acceleration mode when lower than it
acceleration_start_speed = 0.6
# Use High startup current when lower than it
stationary_threshold_speed = 0.3
# Once reached, switch to idle current (But not deceleration)
acceleration_stop_speed = 2
# acceleration_stop_speed = 2.6
# Start breaking if lower than it
breaking_start_speed = acceleration_stop_speed


# Threshold switching to balancing Controller
ctrl_switch_speed = acceleration_stop_speed
# ctrl_switch_speed = 3.0  # Threshold switching to balancing Controller




MAX_HANDLEBAR_ANGLE = deg2rad(50)
MAX_HANDLEBAR_SPEED = 1
MAX_ROLL_ANGLE = deg2rad(20)
rollrate_balancing_start_time = time.time()
rollrate_balancing_duration = 8  # secs



IMU_SWITCH = 1
STR_SWITCH = 1
ENC_SWITCH = 1
DRIVE_SWITCH = 1
HALLSENSOR = False
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
        
        acceleration_binary_code = gps_vesc_object.set_current_binary(startup_current)
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


        # rm = pyvisa.ResourceManager()
        # gps_vesc_object = GPS(session, rm)
        # gps_vesc_object.heart_pipe_child.send('start_heart_beat')
#Begin the execution of the FPGA VI
    # session.abort()
    # session.run()
    # print("Session is running")
    # print("Calibrating the Roll estimation")
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
            # time.sleep(0.1)
            if ROLL_CTRL_SWITCH:
                if ACCELERATING_FLAG and vesc_speed < ctrl_switch_speed:
                    # print(phi)
                    ref_strAngle = roll_ctrl.update(phi)
                    # print(ref_strAngle)
                    if abs(ref_strAngle) > MAX_HANDLEBAR_ANGLE:
                        ref_strAngle = sign(ref_strAngle) * MAX_HANDLEBAR_ANGLE
                        print('Out of Str Angle Range!!!!')
                    strAngle_ctrl.setReference(ref_strAngle)
                    steeringSpeedCmd = strAngle_ctrl.update(steeringAngle)
                elif vesc_speed > ctrl_switch_speed:
                    steeringSpeedCmd = rollrate_inner_ctrl.update(gx)
                elif DECELERATING_FLAG:
                    ref_strAngle = roll_ctrl.update(-phi)
                    # print(ref_strAngle)
                    if abs(ref_strAngle) > MAX_HANDLEBAR_ANGLE:
                        ref_strAngle = sign(ref_strAngle) * MAX_HANDLEBAR_ANGLE
                        print('Out of Str Angle Range!!!!')
                    strAngle_ctrl.setReference(ref_strAngle)
                    steeringSpeedCmd = strAngle_ctrl.update(steeringAngle)
                else:
                    print('STRANGE that slow speed <2.0 but not in deceleration mode')



            if STR_SWITCH:
                # if abs(steeringSpeedCmd) < MAX_HANDLEBAR_SPEED:
                #     strMotorObj.set_angular_velocity(steeringSpeedCmd)
                # else:
                #     strMotorObj.set_angular_velocity(0.0)
                # print(steeringSpeedCmd)
                strMotorObj.set_angular_velocity(steeringSpeedCmd)
                steeringCurrent = strMotorObj.read_steer_current()
            # if HALLSENSOR:
            #     print("Speed Measured:")
            #     print(hall_sensor.get_velocity(Drive_Speed))
            #     # print('FPGA Measurement')
            #     # print(HallSpeedFPGA.read())
            if GPS_USE:
                # gpsStartTime = time.time()
                dx, dy, lat, lon, gpsstatus, utc, speed_m_s, course = \
                    gps_vesc_object.get_position()

                # # gps_vesc_object.get_latlon
                # # print(gps_vesc_object.get_position())
                # GPS_time_cost = time.time() - gpsStartTime
                # if GPS_time_cost > 2e-3:
                #     print('GPS Read costs' + str(GPS_time_cost))
                # # print(gps_vesc_object.Speed_over_Ground)
                # # print(gps_vesc_object.Course_over_Ground)
            if DRIVE_SWITCH:
                # Drive.set_velocity(Drive_Speed)
            #     # Drive.serial_write_character('ident \n')
            #     print('Sending Drive Ident Commands!')
            #     if gps_vesc_object.heart_pipe_child.poll():
            #         last_vesc_data = gps_vesc_object.heart_pipe_child.recv()
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
            if REPEAT_ACC_BREAK_FLAG and not ACCELERATING_FLAG and vesc_speed < acceleration_start_speed:
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



