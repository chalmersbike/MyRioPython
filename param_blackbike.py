from math import pi as PI
import numpy as np
from param import initial_speed, controller_frequency
from constants import *


########################################################################################################################
# Bike Parameters
# b = 1.095                                   # [m] Distance between wheel centers
# LENGTH_A = 0.325  # [m] Distance from rear wheel to COM
LENGTH_A = 0.6687  # [m] Distance from rear wheel to COM
LENGTH_B = 1.15   # [m] Distance between wheel centers
lambda_bike = 66*deg2rad # [rad] Angle of the fork axis

# Wheel
wheel_diameter = 0.694                      # [m] Wheel diameter
tyre_ratio = 0.7 / wheel_diameter           # Tyre ratio
wheel_circumference = wheel_diameter * PI   # [m] Wheel circumference

# IMU
imu_height = 0.215                          # [m] IMU height

# Safety constraints
# Lean (roll) angle limitations
MAX_LEAN_ANGLE = 0.6  # rad (35 deg)
MIN_LEAN_ANGLE = - MAX_LEAN_ANGLE
# Steering angle limitations
# MAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad, 50 deg
LowSpeedMAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad, 50 deg
HighSpeedMAX_HANDLEBAR_ANGLE = ((1.0/ 3.0) * PI) / 6.0   # rad 10 deg
MAX_HANDLEBAR_ANGLE = LowSpeedMAX_HANDLEBAR_ANGLE
MIN_HANDLEBAR_ANGLE = - MAX_HANDLEBAR_ANGLE
MAX_HANDLEBAR_ANGLE_ADJUST = 0  # Switch on/off the handle bar MAX angle range dynamic adjustment
HANDLEBAR_CORRECTION_ANGVEL = 0  # When the handlebar is out of tolerance range, drag it back to the region
# Velocity
HIGHSPEEDBOUND = 3  # m/s


########################################################################################################################
# State Estimators
# Speed estimation gains
statesEstimators_Kv = 0.2
statesEstimators_KvH = 1
statesEstimators_KvGPS = 0
statesEstimators_KvRef = 1 - statesEstimators_KvH - statesEstimators_KvGPS

# Heading estimation gains
# statesEstimators_Kpsi = 0.07
statesEstimators_Kpsi = 0.15
# statesEstimators_Kpsi = 0.01

# Position estimation gains
statesEstimators_Kx = 0.05
statesEstimators_Ky = 0.05
# statesEstimators_Kx = 0.01
# statesEstimators_Ky = 0.01
statesEstimators_Kxy = np.matrix([[statesEstimators_Kx , 0] , [0 , statesEstimators_Ky]])


########################################################################################################################
# Drive Motor
driveMotor_port  = '/dev/ttyO4'             # Drive Motor serial port
driveMotor_UARTPort = "UART4"               # Drive Motor UART port
driveMotor_CommunicationFrequency = 115200  # [Hz] Drive Motor Serial port communication frequency
driveMotor_logFrequency = 10                # [Hz] Frequency at which the motor controller outputs log data


########################################################################################################################
# Steering Motor
steeringMotor_Mode = 'speed'            # Steering Motor mode : 'speed' or 'current'

steeringMotor_GearRatio = 111.0         # Steering Motor gear ratio

steeringMotor_Frequency = 5000          # [Hz] Steering Motor PWM frequency
steeringMotor_IdleDuty = 50.0           # Steering Motor PWM duty cycle for idling (no speed/current)
steeringMotor_PWMforMaxOutput = 90.0    # Steering Motor PWM duty cycle for maximum output (max speed/current)
steeringMotor_PWMforMinOutput = 10.0    # Steering Motor PWM duty cycle for minimum output (min speed/current)

steeringMotor_SpeedMaxOutput = 4000.0   # [rpm] Steering Motor maximum speed
steeringMotor_SpeedMinOutput = -4000.0  # [rpm] Steering Motor minimum speed
# steeringMotor_SpeedMaxOutput = 200.0   # [rpm] Steering Motor maximum speed
# steeringMotor_SpeedMinOutput = -200.0  # [rpm] Steering Motor minimum speed

steeringMotor_CurrentMaxOutput = 1.0    # [A] Steering Motor maximum current
steeringMotor_CurrentMinOutput = -1.0   # [A] Steering Motor minimum current

steeringMotor_Channel = 'EHRPWM2B'  # Steering Motor PWM channel
steeringMotor_PinEnable = 'P8_15'   # Steering Motor enable pin



########################################################################################################################
# Steering Encoder
steeringEncoder_TicksPerRevolution = 2 * 1000 * steeringMotor_GearRatio             # Steering Encoder number of ticks per revolution
steeringEncoder_TicksToRadianRatio = 2 * PI / steeringEncoder_TicksPerRevolution    # Steering Encoder number of ticks to radians conversion


########################################################################################################################
# GPS
gps_port = "/dev/ttyO1"                 # GPS serial port
# gps_baudrate = 9600                     # GPS baudrate [baud] : 9600, 57600, 115200
gps_baudrate = 115200                     # GPS baudrate [baud] : 9600, 57600, 115200
gps_typeOutputData = 'RMCONLY'          # Type of the GPS data output : RMCGGA, RMCONLY, ALLDATA, OFF, GLLONLY, VTGONLY, GGAONLY, GSAONLY, GSVONLY
# gps_typeOutputData = 'GGAONLY'          # Type of the GPS data output : RMCGGA, RMCONLY, ALLDATA, OFF, GLLONLY, VTGONLY, GGAONLY, GSAONLY, GSVONLY
gps_dataUpdateRate = 10                 # GPS data update rate [Hz] : 1, 2, 5, 10
gps_DPGSDataSourceMode = 'WAAS'         # DGPS data source mode : RTCM, WAAS
ntrip_caster_address = "192.71.190.141" # Address of the NTRIP caster, can be IP or domain name
ntrip_port = 80                         # NTRIP Port
ntrip_mountpoint = "RTCM3_GNSS"         # NTRIP Mountpoint
ntrip_username = "ChalmersE2RTK"        # Username to connect to NTRIP caster
ntrip_password = "885511"               # Password to connect to NTRIP caster
ntrip_verbose = 0                       # 1 = NTRIP library is verbose (will output extra information) ; 0 = not verbose

antenna_offset_x = LENGTH_A - 0.1       # [m] Distance from antenna to center of mass on x-axis
antenna_offset_y = 0.0                  # [m] Distance from antenna to center of mass on y-axis


########################################################################################################################
# Hall Sensor
hallSensor_port = 'P9_30'               # Hall sensor GPIO port
hallSensor_pullUpDown = 'up'            # Hall sensor pull-up or pull-down choice : 'up', 'down'
hallSensor_edgeDetection = 'rising'     # Hall sensor edge dection type : 'rising', 'falling'
# hallSensor_numberOfSensors = 3          # Number of magnets placed on the wheel
hallSensor_numberOfSensors = 9          # Number of magnets placed on the wheel
hallSensor_maxElapseBetweenPulses = 1   # [s] Bike is considered stationary (0m/s speed) if time between two pulses is longer than this
# hallSensor_bounceTime = 30              # [ms] Bouncetime between two pulses to avoid reading the same pulse multiple times
hallSensor_bounceTime = 60              # [ms] Bouncetime between two pulses to avoid reading the same pulse multiple times
hallSensor_sensorPosition = 0.347       # [m] Distance between center of wheel and center of magnets (Tyre marking 40-622 = ID of 622mm + 4mm to center of magnet)
hall_Sensor_distanceBetweenMagnets = hallSensor_sensorPosition * 2 * PI / hallSensor_numberOfSensors     # [m] Distance between magnets


########################################################################################################################
# IMU
imu_calibrationSamples = 500                # IMU number of calibration samples used at startup
imu_gyroscopeRange = 245                    # [deg/s] IMU range of the gyroscope : 245, 500, 2000
imu_gyroscopeODR = 6                        # IMU Output Data Rate of the gyroscope, follows the table below
                                                # 0 = gyro off    3 = 119Hz    6 = 952Hz
                                                # 1 = 14.9Hz      4 = 238Hz
                                                # 2 = 59.5Hz      5 = 476Hz
imu_accelerometerRange = 2                  # [g] IMU range of the accelerometer : 2, 4, 8, 16
imu_accelerometerODR = 6                    # IMU Output Data Rate of the accelerometer, follows the table below
                                                # 0 = accel off   3 = 119Hz    6 = 952Hz
                                                # 1 = 10Hz        4 = 238Hz
                                                # 2 = 50Hz        5 = 476Hz

# Complementary filter
# imu_complementaryFilterRatio = 0.985        # IMU complementary filter ratio
imu_complementaryFilterRatio = 0.995        # IMU complementary filter ratio
imu_centripetal_compensation = 1            # 1 = turn on centripetal acceleration compensation ; 0 = turn off
imu_roll_compensation = 1                   # 1 = turn on roll acceleration compensation ; 0 = turn off


########################################################################################################################
# Laser Ranger
laserRanger_roller_width = 380                          # [mm] Width of the roller
laserRanger_sensor1_shutdown = 'P9_23'                  # Laser ranger sensor 1 GPIO shutdown pin
laserRanger_sensor2_shutdown = 'P9_27'                  # Laser ranger sensor 2 GPIO shutdown pin
laserRanger_sensor1_shutdown_pullUpDown = 'down'        # Laser ranger sensor 1 GPIO shutdown pull-up or pull-down choice : 'up', 'down'
laserRanger_sensor2_shutdown_pullUpDown = 'down'        # Laser ranger sensor 2 GPIO shutdown pull-up or pull-down choice : 'up', 'down'
laserRanger_sensor1_I2Caddress = 0x39                   # Laser ranger sensor 1 I2C address
laserRanger_sensor2_I2Caddress = 0x29                   # Laser ranger sensor 2 I2C address
laserRanger_sensor1_accuracyMode = 4                    # Laser ranger sensor 1 accuracy mode, follows the table below
laserRanger_sensor2_accuracyMode = 4                    # Laser ranger sensor 2 accuracy mode, follows the table below
                                                            # 0 = Good Accuracy mode
                                                            # 1 = Better Accuracy mode
                                                            # 2 = Best Accuracy mode
                                                            # 3 = Long Range mode
                                                            # 4 = High Speed mode


########################################################################################################################
# Potentiometer
potentiometer_port = 'P9_36'        # Potentiometer analog input port
potentiometer_maxVoltage = 0.29     # [V] Maximum voltage measured at the potentiometer's output

########################################################################################################################
# SteeringCurrent Read
steering_current_port = 'P9_39'     # ADIN for steering current
steering_max_current = 10.0         # Amper, the maximal current, which gives 1.8 V reading here
########################################################################################################################
# Safety Stop
safetyStop_port = 'P9_15'       # Safety stop GPIO port
safetyStop_pullUpDown = 'up'    # Safety stop pull-up or pull-down choice : 'up', 'down'


########################################################################################################################
# Velocity Controller
# PID Velocity Controller Parameters
pid_velocity_active = False  # Boolean to control if PID controller is used
pid_velocity_reference = initial_speed  # m/s
pid_velocity_P = 3.0
pid_velocity_I = 0.01
pid_velocity_D = 0.0
pid_velocity_sample_time = 1.0 / controller_frequency


########################################################################################################################
# Steering Controller
# PID Steering Controller Parameters
pid_steering_reference = 0.0  # PID code uses constant reference setpoint and feedback. Since this controller doesn't have a constant setpoint, we only use the feedback value.
pid_steering_P = 10.0
pid_steering_I = 10.0
pid_steering_D = 0.0
pid_steering_sample_time = 1.0 / controller_frequency

# STEERING ANGLE CONTROL
# # # pid_steeringangle_P = 20.0
# # # pid_steeringangle_I = 0.0
# # # pid_steeringangle_D = 0.0
# # pid_steeringangle_P = 14.0
# # pid_steeringangle_I = 30.0
# # pid_steeringangle_D = 0.0
# pid_steeringangle_P = 10.0
# pid_steeringangle_I = 5.0
# pid_steeringangle_D = 0.0
pid_steeringangle_P = 3.0
pid_steeringangle_I = 0.0
pid_steeringangle_D = 0.0
pid_steeringangle_sample_time = 1.0 / controller_frequency


########################################################################################################################
# Balancing Controller
# Controller gains can be speed dependant if they are arrays with multiple values
# If they contain a single value they are not speed dependant

# Lookup table for speed
speed_lookup_controllergains = [3,4,5]

# PID Balance Inner Loop Controller Parameters
pid_balance_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
#pid_balance_P = 1.57
# pid_balance_P = 5.0 # Gain tested with Peo for vel = 4
pid_balance_P = 5.0 # when vel = 3
pid_balance_I = 0.0
pid_balance_D = 0.0
pid_balance_sample_time = 1.0 / controller_frequency

# PID Balance Outer Loop Controller Parameters
pid_balance_outerloop_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
#pid_balance_outerloop_P = 0.57
pid_balance_outerloop_P = 1.0 # Gain tested with Peo
# pid_balance_outerloop_P = 1.3 # Gain tested with Peo
pid_balance_outerloop_I = 0.0
pid_balance_outerloop_D = 0.0
pid_balance_outerloop_sample_time = 1.0 / controller_frequency


########################################################################################################################
# Path Tracking
max_rollref = 6 # [deg] maximum roll reference to be set by path tracking controller
# Controller gains can be speed dependant if they are arrays with multiple values
# If they contain a single value they are not speed dependant
PATH_TYPE = 'NONE'  # [NONE, CIRCLE, STRAIGHT]
# PATH_TYPE = 'STRAIGHT'  # [NONE, CIRCLE, STRAIGHT]
PATH_RADIUS = 20.0  # m
time_path_stay = 10.0 # s
time_path_slope = 10.0 # s
slope = 1/100.0
path_sine_amp = 0.15 # m
path_sine_freq = 1/10.0 # Hz
path_choice = 'pot'
#path_choice = 'overtaking'
#path_choice = 'sine'

# PID Lateral Position Controller Parameters
pid_lateral_position_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
# pid_lateral_position_P = -2e-1 # 23/11/2020
pid_lateral_position_P = -1e-1 # 13/11/2020
# pid_lateral_position_P = -1e-1
# pid_lateral_position_I = -2e-2
pid_lateral_position_I = -0e-2
pid_lateral_position_D = -0e-1
pid_lateral_position_sample_time = 1.0 / controller_frequency

# PID Direction Controller Parameters
pid_direction_reference = 0.0  # PID code uses constant reference setpoint and feedback. Since this controller doesn't have a constant setpoint, we only use the feedback value.
pid_direction_P = -2e-1
pid_direction_I = -0.0
pid_direction_D = -0.0
pid_direction_sample_time = 1.0 / controller_frequency
