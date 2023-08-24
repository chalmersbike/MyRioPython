# ISE stands for IMU Steering and Encoder Test.#
from param import *

from actuators import DriveMotor, SteeringMotor

from time import sleep
import time
from nifpga import Session
from numpy import rad2deg
import math
import re
from vesc_gps_resource_v3 import VESC_GPS
# import pysnooper
# with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_O+w-wwa0L-U.lvbitx", "RIO0") as session:
#with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_9KQYwNHlq+s.lvbitx", "RIO0") as session:

 # balancing Control V5
with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_5.lvbitx", "RIO0") as session:  # balancing Control V5

    print('Terminating ALl ACTUATORS !!!')

    fpga_SteeringWriteDutyCycle = session.registers['Steering PWM Duty Cycle']
    fpga_SteeringWriteFrequency = session.registers['Steering PWM Frequency']
    fpga_SteeringEnable = session.registers['Steering Enable']

    fpga_SteeringEnable.write(False)
    fpga_SteeringWriteDutyCycle.write(50)
    fpga_SteeringWriteFrequency.write(5000)
    print('PWM Reset is DONE!')

    Drive = VESC_GPS(session)
    Drive.stop()
    print('DriveMotor Rest is DONE!')
    session.abort()
    print('Session Abort, termination finished!!!!')
    # session.run()


