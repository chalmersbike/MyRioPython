import serial, time
import  Adafruit_BBIO.PWM as PWM
import time
import Adafruit_BBIO.GPIO as GPIO
channel = 'EHRPWM2B'
frequency = 50
Pin_enable = 'P8_15'
idle_duty = 50

# Filter requirements.
order = 2
fs = 25       # sample rate, Hz
cutoff = 3  # desired cutoff frequency of the filter, Hz


class SteeringMotor(object):

    # serial = None

    def __init__(self):
        GPIO.setup(Pin_enable, GPIO.OUT)
        GPIO.output(Pin_enable, GPIO.LOW)
        print 'Enable pin set to DISABLE'
        PWM.start(channel, idle_duty, frequency)
        PWM.set_duty_cycle(channel, 50)
        print 'PWM started'
        self.pwm_list = []

    # @pysnooper.snoop()
    def set_angular_velocity(self, angular_velocity):
        rpm_conversion = angular_velocity / 6.28 * 60 * 111.0
        duty_cycle = 50 + rpm_conversion * 40.0 / 1000.0
        # print rpm_conversion, duty_cycle
        if duty_cycle > 90:
            duty_cycle = 90
        elif duty_cycle < 10:
            duty_cycle = 10
        PWM.set_duty_cycle(channel, duty_cycle)

        
    def set_current(self, current):
        duty_cycle = 50 + current * 40.0 / 1.0 # 10% = -1A ; 90% = 1A
        # print rpm_conversion, duty_cycle
        if duty_cycle > 90:
            duty_cycle = 90
        elif duty_cycle < 10:
            duty_cycle = 10
        PWM.set_duty_cycle(channel, duty_cycle)


    def stop(self):
        GPIO.output(Pin_enable, GPIO.LOW)
        GPIO.cleanup()
        PWM.stop(channel)
        PWM.cleanup()

    def enable(self):
        GPIO.output(Pin_enable, GPIO.HIGH)