from steering import SteeringMotor
import time
mot = SteeringMotor()
mot.set_angular_velocity(1)
time.sleep(1)
mot.stop()