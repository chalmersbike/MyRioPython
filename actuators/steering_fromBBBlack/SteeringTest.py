from steering import SteeringMotor
import time
mot = SteeringMotor()
mot.enable()
mot.set_angular_velocity(0.05)
time.sleep(30)
mot.stop()