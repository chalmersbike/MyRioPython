from nifpga import Session
import time
import math
import re
import pyvisa
#IMU
gyro_sensitivity = 0.00875
accel_sensitivity = 0.000061
deg2rad = math.pi/180.0

#drive motor
input_velocity = 2.0
rpm = input_velocity * 0.2
driveMotor_CommunicationFrequency = 115200
str_run = 'run -s ' + str(rpm).zfill(4) + ' -f 9999\n'
str_stop = 'run -stop\n'

# Wheel
wheel_diameter = 0.694                      # [m] Wheel diameter
tyre_ratio = 0.7 / wheel_diameter           # Tyre ratio

# Encoder
steeringMotor_GearRatio = 111.0         # Steering Motor gear ratio
steeringEncoder_TicksPerRevolution = 2 * 1000 * steeringMotor_GearRatio             # Steering Encoder number of ticks per revolution
steeringEncoder_TicksToRadianRatio = 2 * math.pi / steeringEncoder_TicksPerRevolution    # Steering Encoder number of ticks to radians conversion

# Hall Sensor
hallSensor_edgeDetection = 'falling'     # Hall sensor edge dection type : 'rising', 'falling'
# hallSensor_edgeDetection = 'rising'     # Hall sensor edge dection type : 'rising', 'falling'
hallSensor_numberOfSensors = 9          # Number of magnets placed on the wheel
hallSensor_maxElapseBetweenPulses = 1   # [s] Bike is considered stationary (0m/s speed) if time between two pulses is longer than this
hallSensor_bounceTime = 60              # [ms] Bouncetime between two pulses to avoid reading the same pulse multiple times
hallSensor_sensorPosition = 0.347       # [m] Distance between center of wheel and center of magnets (Tyre marking 40-622 = ID of 622mm + 4mm to center of magnet)
hall_Sensor_distanceBetweenMagnets = hallSensor_sensorPosition * 2 * math.pi / hallSensor_numberOfSensors     # [m] Distance between magnets


with Session("./FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_yn8cRoiCUew.lvbitx", "RIO0") as session:
# with Session("./FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_yZE+ttlxcs0.lvbitx", "RIO0") as session:
# with Session("./FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_AyrFyQ0yD9A.lvbitx", "RIO0") as session:

    ###################################################################
    ############################### Steering Motor ###############################
    ###################################################################
    fpga_SteeringWriteDutyCycle = session.registers['Steering PWM Duty Cycle']
    fpga_SteeringWriteFrequency = session.registers['Steering PWM Frequency']
    fpga_SteeringEnable = session.registers['Steering Enable']

    ###################################################################
    ############################### IMU ###############################
    ###################################################################
    fpga_GyroData = session.registers['SPI Read Gyro Data']
    fpga_AccData = session.registers['SPI Read Acc Data']

    ###################################################################
    ############################### Hall Sensor ###############################
    ###################################################################
    fpga_HallSlope = session.registers['Hall Edge Detection ']
    fpga_HallData = session.registers['Hall Period (uSec/pulse)']

    # drive motor ressource management
    rm = pyvisa.ResourceManager()
    port_b = rm.open_resource('ASRL2::INSTR')
    port_b.baud_rate = 115200

    print('Running the drive motor')
    ###################################################################
    ############################### encoder #######################
    ###################################################################
    fpga_EncoderData = session.registers['Encoder Counts']
    fpga_EncoderReset = session.registers['Encoder Position Reset']
    fpga_EncoderReset.write(True)

    ###################################################################
    ############################### GPS ###############################
    ###################################################################
    #fpga_GPSData = session.registers['Read Data']

    ############################### SESSION START ###############################
    session.abort()
    session.run()
    print("Session is running")
    time.sleep(3)
    fpga_SteeringEnable.write(False)
    t_start = time.time()

    while True:
        t_startLoop = time.time()
        t_start_Encoder = time.time()
        t_start_IMU = time.time()
        ###################################################################
        ############################### sTEERING ###########################
        ###################################################################
        fpga_SteeringEnable.write(True)
        fpga_SteeringWriteFrequency.write(5000)
        fpga_SteeringWriteDutyCycle.write(50)
        ###################################################################
        ############################### Encoder ###########################
        ###################################################################
        fpga_EncoderData_value = fpga_EncoderData.read()
        steering_position = -fpga_EncoderData_value * steeringEncoder_TicksToRadianRatio
        print('Encoder : t = %f ; steering_position = %f' % (time.time() - t_start, steering_position / deg2rad))
        t_end_Encoder = time.time()

        t_start_Hall = time.time()
        ###################################################################
        ############################### Hall sensor #######################
        ###################################################################
        fpga_HsllData_value = fpga_HallData.read()
        elapse = fpga_HsllData_value * 1.0e-6
        velocity = (0.0 if not elapse
                    else 0.0 if elapse > hallSensor_maxElapseBetweenPulses
        else (hall_Sensor_distanceBetweenMagnets / elapse) * tyre_ratio)

        print('Hall sensor : t = %f ; elapse = %f ;  velocity = %f' % (time.time() - t_start, elapse, velocity))
        t_end_Hall = time.time()

        print('t_Encoder = %f ; t_Hall = %f' % (t_end_Encoder - t_start_Encoder, t_end_Hall - t_start_Hall))

        print('\n')
        ###################################################################
        ############################### Drive Motor ###############################
        ###################################################################
        port_b.write(str_run)
        ###################################################################
        ############################### IMU ###############################
        ###################################################################
        fpga_GyroData_value = fpga_GyroData.read()
        fpga_AccData_value = fpga_AccData.read()
        1680000
        24

        # Read gyro to buffer
        buf = fpga_GyroData_value
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

        # t_start_GPS = time.time()
        # ###################################################################
        # ############################### GPS ###############################
        # ###################################################################
        # fpga_GPSData_value = fpga_GPSData.read()
        # nmea_str = ''.join(chr(int(i)) for i in fpga_GPSData_value)
        # nmea_str_split = re.split(',|\n|\r',nmea_str)
        #
        # try:
        #     if nmea_str_split[0] == '$GPRMC' or nmea_str_split[0] == '$GNRMC':
        #         rmc = nmea_str_split
        #         utc = rmc[1]
        #         status = rmc[2]
        #         latitude = rmc[3]
        #         NS_indicator = rmc[4]
        #         longitude = rmc[5]
        #         EW_indicator = rmc[6]
        #         Speed_over_Ground = rmc[7]
        #         Course_over_Ground = rmc[8]
        #         Date = rmc[9]
        #         Magnetic_Variation = rmc[10]
        #         Mode_rmc = rmc[11]
        #
        #         latitude = float(latitude)
        #         longitude = float(longitude)
        #
        #         print('GPS : t = %f ; lat = %f ; lon = %f ; NMEA_UTC = %s' % (time.time()-t_start,latitude,longitude,utc))
        #     else:
        #         print('Incorrect NMEA string received : ' + re.split('[\n\r]',nmea_str)[0])
        # except:
        #     print('Incorrect NMEA string received : ' + re.split('[\n\r]',nmea_str)[0])
        #
        # t_end_GPS = time.time()
        #
        # print('t_IMU = %f ; t_GPS = %f' % (t_end_IMU-t_start_IMU,t_end_GPS-t_start_GPS))
        print('\n')
        #fpga_SteeringEnable.write(False)
        port_b.write(str_stop)
        time.sleep(0.1)