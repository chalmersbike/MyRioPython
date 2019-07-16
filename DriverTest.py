from actuators import RearMotorDrive
import time
from sensors import HallSensor
mot = RearMotorDrive()

hall_sensor = HallSensor()
start_time = time.time()
mot.set_velocity(1)
# time.sleep(10)
for x in range(1,20):
    print 'Time=%f\t Vel = %f\n' % (time.time()-start_time,hall_sensor.get_velocity())
    time.sleep(0.5)

mot.stop()