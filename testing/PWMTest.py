sys.path.append('../')

import  Adafruit_BBIO.PWM as PWM
import time

channel = 'EHRPWM1A'
frequency = 50
Pin_enable = 'P9_12'
import Adafruit_BBIO.GPIO as GPIO

rad_per_second = 0

try:


    rpm_conversion = rad_per_second / 6.28 * 60 * 111.0

    duty_cycle = 50 + rpm_conversion * 40.0 / 8000.0

    print duty_cycle
    PWM.start(channel, duty_cycle, frequency)
    GPIO.setup(Pin_enable, GPIO.OUT)
    GPIO.output(Pin_enable, GPIO.HIGH)
    time.sleep(20)
    PWM.stop(channel)
    PWM.cleanup()
    GPIO.output(Pin_enable, GPIO.LOW)
    GPIO.cleanup()


except (ValueError, KeyboardInterrupt):
    GPIO.output(Pin_enable, GPIO.LOW)
    GPIO.cleanup()
    PWM.stop(channel)
    PWM.cleanup()