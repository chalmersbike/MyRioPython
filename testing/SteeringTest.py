sys.path.append('../')

from actuators.steeringmotor import SteeringMotor
import time
mot = SteeringMotor()
mot.set_angular_velocity(0.1)
time.sleep(3)
mot.stop()