import serial, time
import  Adafruit_BBIO.PWM as PWM
import time
import Adafruit_BBIO.GPIO as GPIO
from lowpass import butter_lowpass,butter_lowpass_filter
channel = 'P9_21'
frequency = 50
Pin_enable = 'P9_22'
idle_duty = 50

# Filter requirements.
order = 2
fs = 25       # sample rate, Hz
cutoff = 3  # desired cutoff frequency of the filter, Hz

# Get the filter coefficients so we can check its frequency response.
b, a = butter_lowpass(cutoff, fs, order)


class SteeringMotor(object):

    serial = None

    def __init__(self):
        PWM.start(channel, idle_duty, frequency)
        GPIO.setup(Pin_enable, GPIO.OUT)
        GPIO.output(Pin_enable, GPIO.HIGH)
        self.pwm_list = []

    def set_angular_velocity(self, angular_velocity):
        rpm_conversion = angular_velocity / 6.28 * 60 * 111.0
        duty_cycle = 50 + rpm_conversion * 40.0 / 8000.0
        PWM.set_duty_cycle(channel, duty_cycle)

    def stop(self):
        GPIO.output(Pin_enable, GPIO.LOW)
        GPIO.cleanup()
        PWM.stop(channel)
        PWM.cleanup()

    def read_UART(self):
        signal_read = self.serial.read(5)
        if abs(int(signal_read[0:3]) - int(self.pwm_str[0:3])) > 2:
            print 'The Reading doesnt match  Read: ' + signal_read + '  Sent: '+ self.pwm_str
            return signal_read
        else:
            return 1