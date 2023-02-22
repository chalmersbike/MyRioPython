# ISE stands for IMU Steering and Encoder Test.
from param import *
from sensors import Encoder, HallSensor, SafetyStop
from sensors import IMU
from actuators import DriveMotor, SteeringMotor
# from actuators import DriveMotor
from time import sleep
import time
from nifpga import Session
from numpy import rad2deg
import math
import re
import traceback
import signal
import threading
import sys
import pyvisa
# import pysnooper
# with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_O+w-wwa0L-U.lvbitx", "RIO0") as session:
#with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_9KQYwNHlq+s.lvbitx", "RIO0") as session:

IMU_SWITCH = 0
STR_SWITCH = 0
ENC_SWITCH = 0
DRIVE_SWITCH = 0
HALLSENSOR = 0
EMERGENCY = 0
GPS_USE = 1
from sensors import GPS
from vesc_gps_resource_v2 import VESC_GPS
from numpy import rad2deg

# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_yn8cRoiCUew.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_wV6o5H-69rg.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_4P-eIdLgj4Y.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_ip6b6zVtH2s.lvbitx", "RIO0") as session:  # balancing Control New
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingno20210901.lvbitx","RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV5_iSMcVs1R7M8.lvbitx", "RIO0") as session:  # balancing Control V5
with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_5.lvbitx", "RIO0") as session:  # balancing Control V5

    print("Session started, Wait for 3 secs")
    # time.sleep(3)
#Declaration of the session registers

    if STR_SWITCH:
        # fpga_SteeringWriteDutyCycle = session.registers['Duty Cycle (%)']
        # fpga_SteeringWriteFrequency = session.registers['Frequency (Hz)']
        # fpga_SteeringEnable = session.registers['Write']
        # fpga_SteeringWriteDutyCycle = session.registers['Steering PWM Duty Cycle']
        # fpga_SteeringWriteFrequency = session.registers['Steering PWM Frequency']
        # fpga_SteeringEnable = session.registers['Steering Enable']
        strMotorObj = SteeringMotor(session)
        PositiveStrSpeed = 0.1
        NegativeStrSpeed = -0.8

    if IMU_SWITCH:
        ###################################################################
        ############################### IMU ###############################
        ###################################################################
        fpga_GyroData = session.registers['SPI Read Gyro Data']
        fpga_AccData = session.registers['SPI Read Acc Data']
        # WhoAmI = session.registers['WHOAMI']
        t_start = time.time()
        gyro_sensitivity = 0.00875
        accel_sensitivity = 0.000061
        deg2rad = math.pi / 180.0
    if ENC_SWITCH:
        encoder_object = Encoder(session)
        # fpga_EncoderCounts = session.registers['Encoder Counts']
        # fpga_EncoderPositionReset = session.registers['Encoder Position Reset']
        # fpga_EncoderPositionReset.write(True)
    if DRIVE_SWITCH:
        Drive_Speed = 3
        Drive = DriveMotor(session)
        # Drive = DriveMotor()
        print('DRIVE MOTOR will start in 3 secs')
        time.sleep(3)
        Drive.set_velocity(Drive_Speed)
        time.sleep(0.1)
        Drive.heart_pipe_parent.send('start_heart_beat')
        time.sleep(0.1)
        Drive.vescdata_pipe_parent.send('start_vesc_query')

    if HALLSENSOR:
        hall_sensor = HallSensor(session)
        HallSpeedFPGA = session.registers['Speed m/s']
        session.registers['Hall Edge Detection '] = "rising"
        session.registers['Bounce time (uSec)'] = 0
    if EMERGENCY:
        fpga_ESTOP = session.registers['ESTOP']
    if GPS_USE:
        # rm = pyvisa.ResourceManager()
        # gps_object = GPS(session, rm)
        gps_object = VESC_GPS(session)
        # gps_object.heart_pipe_child.send('start_heart_beat')
#Begin the execution of the FPGA VI
    session.abort()
    session.run()
    print("Session is running")
    # Steering loop
    if STR_SWITCH:
        strMotorObj.stop()
        # fpga_SteeringEnable.write(False)
        print("Steering is desactivated")
        time.sleep(3)

        strMotorObj.set_angular_velocity(0)
        strMotorObj.enable()
        # fpga_SteeringEnable.write(True)
        print("Steering is NOW activated")
        PrevStrSpeed = PositiveStrSpeed
        # fpga_SteeringWriteDutyCycle.write(50)
        # fpga_SteeringWriteFrequency.write(5000)
        # dc = 50
        # print('Ramping between 30-70 duty cycle!!!!')


    try:
        while True:

            if IMU_SWITCH:
                t_startLoop = time.time()

                t_start_IMU = time.time()
                ###################################################################
                ############################### IMU ###############################
                ###################################################################
                fpga_GyroData_value = fpga_GyroData.read()
                fpga_AccData_value = fpga_AccData.read()
                # print(WhoAmI.read())
                print('\n')
                # Read gyro to buffer
                buf = fpga_GyroData_value
                print(buf)
                buf = buf[:7]

                # Calculate gyro values from buffer
                gx = (buf[2] << 8) | buf[1]
                gy = (buf[4] << 8) | buf[3]
                gz = (buf[6] << 8) | buf[5]
                # Apply two's complement
                if (gx & (1 << (16 - 1))) != 0:
                    gx -= (1 << 16)
                if (gy & (1 << (16 - 1))) != 0:
                    gy -= (1 << 16)
                if (gz & (1 << (16 - 1))) != 0:
                    gz -= (1 << 16)
                # # the PmodNav has y pointing to the right of x (left axised) but the bike has y pointing to the left of x (right axised)
                # gy  = -gy
                gx *= gyro_sensitivity * deg2rad
                gy *= gyro_sensitivity * deg2rad
                gz *= gyro_sensitivity * deg2rad
                # gx -= gx_offset
                # gy -= gy_offset
                # gz -= gz_offset

                # Read accel to buffer
                buf = fpga_AccData_value
                buf = buf[:7]
                print(buf)
                # Calculate accel values from buffer
                ax = (buf[2] << 8) | buf[1]
                ay = (buf[4] << 8) | buf[3]
                az = (buf[6] << 8) | buf[5]
                # Apply two's complement
                if (ax & (1 << (16 - 1))) != 0:
                    ax -= (1 << 16)
                if (ay & (1 << (16 - 1))) != 0:
                    ay -= (1 << 16)
                if (az & (1 << (16 - 1))) != 0:
                    az -= (1 << 16)
                # # the PmodNav has y pointing to the right of x (left axised) but the bike has y pointing to the left of x (right axised)
                # ay = -ay
                ax *= accel_sensitivity
                ay *= accel_sensitivity
                az *= accel_sensitivity

                print('IMU : t = %f ; gx = %f ; gy = %f ; gz = %f ; ax = %f ; ay = %f ; az = %f' %(time.time()-t_start,gx,gy,gz,ax,ay,az))
                t_end_IMU = time.time()
            if ENC_SWITCH:
                print('ENC Reading: \n')
                # fpga_EncoderCounts.read()
                # print(rad2deg(-fpga_EncoderCounts.read() * steeringEncoder_TicksToRadianRatio))
                steeringAngle = encoder_object.get_angle()
                print(rad2deg(steeringAngle))
            # time.sleep(0.1)

            if STR_SWITCH:
                steeringAngleDeg = rad2deg(steeringAngle)

                if steeringAngleDeg > 30:
                    PrevStrSpeed = NegativeStrSpeed
                elif steeringAngleDeg < -30:
                    PrevStrSpeed = PositiveStrSpeed

                strMotorObj.set_angular_velocity(PrevStrSpeed)

                # fpga_SteeringWriteDutyCycle.write(dc)
                # if dc > 50.5:
                #     dc = 49.5
                # print('STR PWM Duty Cycle:')
                # print(dc)

            if HALLSENSOR:
                print("Speed Measured:")
                print(hall_sensor.get_velocity(Drive_Speed))
                # print('FPGA Measurement')
                # print(HallSpeedFPGA.read())
            if EMERGENCY:
                print('ESTOP Reading %i' %fpga_ESTOP.read())
            if GPS_USE:
                gpsStartTime = time.time()
                gps_object.get_position()
                # gps_object.get_latlon
                # print(gps_object.get_position())
                GPS_time_cost = time.time() - gpsStartTime
                if GPS_time_cost > 2e-3:
                    print('GPS Read costs' + str(GPS_time_cost))
                # print(gps_object.Speed_over_Ground)
                # print(gps_object.Course_over_Ground)
            if DRIVE_SWITCH:
                # Drive.set_velocity(Drive_Speed)
            #     # Drive.serial_write_character('ident \n')
            #     print('Sending Drive Ident Commands!')
                print(Drive.last_vesc_data)

            # Read the data from VESC!


            
            time.sleep(0.1)


    except Exception as e:
        if STR_SWITCH:
            fpga_SteeringEnable.write(False)
            fpga_SteeringWriteDutyCycle.write(50)
            if DRIVE_SWITCH:
                Drive.stop()
            time.sleep(3)
            print('Steering Control terminated')
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



