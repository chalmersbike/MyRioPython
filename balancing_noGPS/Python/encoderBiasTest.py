# ISE stands for IMU Steering and Encoder Test.
from nifpga import Session
# from actuators import SteeringMotor
import time
import math
import re
from param import *
import numpy as np
from actuators import DriveMotor
import pyvisa
# with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_O+w-wwa0L-U.lvbitx", "RIO0") as session:
#with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_9KQYwNHlq+s.lvbitx", "RIO0") as session:

IMU_SWITCH = 0
STR_SWITCH = 0
ENC_SWITCH = 1
DRIVE_SWITCH = 1

# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_yn8cRoiCUew.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_wV6o5H-69rg.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_4P-eIdLgj4Y.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_4.lvbitx","RIO0") as session:  # balancing Control V5
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_test.lvbitx","RIO0") as session:  # balancing Control V5
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_20220510.lvbitx","RIO0") as session:  # balancing Control V5
with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_5.lvbitx","RIO0") as session:  # balancing Control V5

# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingno20210901.lvbitx","RIO0") as session:
    print("Session started, Wait for 3 secs")
    # time.sleep(3)
#Declaration of the session registers

    if STR_SWITCH:
        # fpga_SteeringWriteDutyCycle = session.registers['Duty Cycle (%)']
        # fpga_SteeringWriteFrequency = session.registers['Frequency (Hz)']
        # fpga_SteeringEnable = session.registers['Write']
        fpga_SteeringWriteDutyCycle = session.registers['Steering PWM Duty Cycle']
        fpga_SteeringWriteFrequency = session.registers['Steering PWM Frequency']
        fpga_SteeringEnable = session.registers['Steering Enable']
        fpga_current = session.registers['Steering Current']


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
        fpga_EncoderCounts = session.registers['Encoder Counts']
        fpga_EncoderPositionReset = session.registers['Encoder Position Reset']
    if DRIVE_SWITCH:
        rm = pyvisa.ResourceManager()
        Drive_Speed = 3
        Drive = DriveMotor(session, rm)
        # Drive = DriveMotor()
        print('DRIVE MOTOR will start in 3 secs')
        time.sleep(3)
        Drive.set_velocity(Drive_Speed)
        time.sleep(0.1)
        Drive.heart_pipe_parent.send('start_heart_beat')

#Begin the execution of the FPGA VI
    session.abort()
    session.run()
    print("Session is running")

    # Steering loop
    if STR_SWITCH:
        fpga_SteeringEnable.write(False)
        print("Steering is desactivated")
        time.sleep(3)

        desired_str_speed = 0
        rpm_conversion = -desired_str_speed * rads2rpm * steeringMotor_GearRatio
        duty_cycle = np.interp(rpm_conversion, [steeringMotor_SpeedMinOutput, steeringMotor_SpeedMaxOutput],
                               [steeringMotor_PWMforMinOutput, steeringMotor_PWMforMaxOutput])


        fpga_SteeringWriteDutyCycle.write(duty_cycle)
        fpga_SteeringWriteFrequency.write(5000)
        fpga_SteeringEnable.write(True)
        print("Steering is NOW activated")

        print('Ramping between 30-70 duty cycle!!!!')

    str_direction = 1
    t_startLoop = time.time()
    enc_angle = 0.0
    desired_str_speed = 0.2 * str_direction
    fpga_EncoderPositionReset.write(True)
    enc_angle = -fpga_EncoderCounts.read() * steeringEncoder_TicksToRadianRatio
    print('ENC Reading: \n')
    print(enc_angle)
    periodCounter = 0
    descr = input('\n Mark the zero position!')

    try:
        while True:
            time_lastloop = t_startLoop
            t_startLoop = time.time()
            enc_angle_prev = enc_angle

            #
            if STR_SWITCH:

                if enc_angle > np.deg2rad(45):
                    str_direction = -1
                    desired_str_speed = 1.5 * str_direction
                elif enc_angle < np.deg2rad(0):
                    if str_direction < 0:
                        periodCounter = periodCounter + 1
                    str_direction = 1
                    desired_str_speed = 0.2 * str_direction

                    if periodCounter > 20:
                        fpga_SteeringWriteDutyCycle.write(50.0)
                        enc_angle = -fpga_EncoderCounts.read() * steeringEncoder_TicksToRadianRatio
                        print('ENC Reading: \n')
                        print(enc_angle)
                        input('\n Check the zero position after 20 cycles!')
                        periodCounter = 0


                rpm_conversion = -desired_str_speed * rads2rpm * steeringMotor_GearRatio
                duty_cycle = np.interp(rpm_conversion, [steeringMotor_SpeedMinOutput, steeringMotor_SpeedMaxOutput],
                                       [steeringMotor_PWMforMinOutput, steeringMotor_PWMforMaxOutput])
                print(duty_cycle)
                fpga_SteeringWriteDutyCycle.write(duty_cycle)

                print('The steering motor current is: %.2f' % ((fpga_current.read()*1.221e-3-2)*2) )
            if IMU_SWITCH:


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
                enc_angle = -fpga_EncoderCounts.read() * steeringEncoder_TicksToRadianRatio
                print('ENC Reading: \n')
                print(enc_angle)

                if STR_SWITCH:
                    if abs(desired_str_speed - (enc_angle - enc_angle_prev)/(t_startLoop - time_lastloop)) > 0.03:
                        print('The steering motor may be faulty!')
                        print(desired_str_speed)
                        print((enc_angle - enc_angle_prev)/(t_startLoop - time_lastloop))


            time.sleep(0.002)


    except:
        print("Aborting")
        if STR_SWITCH:
            fpga_SteeringEnable.write(False)
            fpga_SteeringWriteDutyCycle.write(50)
            time.sleep(3)
            print('Steering Control terminated')
        session.abort()



