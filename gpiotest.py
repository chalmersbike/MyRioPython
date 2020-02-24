import Adafruit_BBIO.GPIO as GPIO
import time
from actuators import RearMotorDrive, SteeringMotor, RearMotor

try:
    def test(*args):
        print 'edge detected'

    INPUT_PORT = 'P9_23'
    GPIO.setup(INPUT_PORT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(INPUT_PORT, GPIO.FALLING, callback=test,bouncetime=20)  # dded a 5ms bouncetime to avoid counting the same falling edge multiple times

    rear_motor_velocity = RearMotorDrive()
    rear_motor_velocity.set_velocity(1)

    time_ini = time.time()
    while (time.time()-time_ini < 3):
        GPIO.wait_for_edge(INPUT_PORT, GPIO.FALLING)
        print 'time = %f ; gpio = %d' % ((time.time()-time_ini),GPIO.input(INPUT_PORT))
        time.sleep(0.05)
    rear_motor_velocity.stop()
except:
    print 'Error detected'
    rear_motor_velocity.stop()
