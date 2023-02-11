from nifpga import Session
import time
import math
import re

deg2rad = math.pi/180.0

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


with Session("../FPGA Bitfiles/fpgaencoderhall_FPGATarget_FPGAencoderhall_mIDLWcLeA34.lvbitx", "RIO0") as session:
    ###################################################################
    ############################### Encoder ###########################
    ###################################################################
    fpga_HallSlope = session.registers['Slope']
    fpga_HallData = session.registers['Hall Period (uSec/pulse)']
    # fpga_HallSlope.write(hallSensor_edgeDetection)

    ###################################################################
    ############################### Hall sensor #######################
    ###################################################################
    fpga_EncoderData = session.registers['Encoder (Counts)']
    fpga_EncoderReset = session.registers['Encoder Position Reset']
    fpga_EncoderReset.write(True)

    t_start = time.time()

    while True:
        t_startLoop = time.time()

        t_start_Encoder = time.time()
        ###################################################################
        ############################### Encoder ###########################
        ###################################################################
        fpga_EncoderData_value = fpga_EncoderData.read()
        steering_position = -fpga_EncoderData_value * steeringEncoder_TicksToRadianRatio
        print('Encoder : t = %f ; steering_position = %f' %(time.time()-t_start,steering_position/deg2rad))
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

        print('Hall sensor : t = %f ; elapse = %f ;  velocity = %f' %(time.time()-t_start,elapse,velocity))
        t_end_Hall = time.time()

        print('t_Encoder = %f ; t_Hall = %f' % (t_end_Encoder-t_start_Encoder,t_end_Hall-t_start_Hall))

        print('\n')

        time.sleep(0.01)