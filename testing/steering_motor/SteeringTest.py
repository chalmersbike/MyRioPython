import sys
sys.path.append(sys.path[0]+'/../../')
# from steeringmotor import SteeringMotor
from actuators import SteeringMotor
import time
mot = SteeringMotor()
# mot.set_angular_velocity(0.1)
mot.enable()
mot.set_PWM_duty_cycle(55)
time.sleep(10)
mot.stop()