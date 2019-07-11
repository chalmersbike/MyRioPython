from RearMotorDrive import RearMotorDrive
import time
mot = RearMotorDrive()
mot.set_velocity(3)
time.sleep(10)
mot.stop()