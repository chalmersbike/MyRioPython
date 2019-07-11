import time

import Adafruit_BBIO.UART as UART
import serial

LINEAR_ACTUATOR_PORT = '/dev/ttyO1'
COMMUNICATION_FREQUENCY = 115200

PWM_MIN = 710
PWM_STOP = 1580
PWM_MAX = 2450


class LinearActuator(object):
    serial = None

    def __init__(self):
        UART.setup("UART1")
        self.serial = serial.Serial(port=LINEAR_ACTUATOR_PORT, baudrate=COMMUNICATION_FREQUENCY)
        self.serial.close()
        self.serial.open()

    def serial_write_character(self, character):
        if self.serial.isOpen():
            self.serial.write(character)
        else:
            print 'Serial is not open!'
            self.serial.close()
            self.serial.open()
            self.serial.write(character)

    def brake_set_pwm(self, pwm):
        pwm_string = str(pwm).zfill(4) + '$'
        # print 'pwm_string = %s\n' %(pwm_string)
        for character in pwm_string:
            self.serial_write_character(character)
            time.sleep(0.001)  # fixes comms issue

    def stop(self):
        self.brake_set_pwm(PWM_STOP)

    def set_duty_cycle(self, duty_cycle):
        if duty_cycle > 0 and duty_cycle <= 100:  # positive values [1, 100]
            self.brake_set_pwm(int((duty_cycle / 100.0) * (PWM_MAX - PWM_STOP) + PWM_STOP))
        elif duty_cycle < 0 and duty_cycle >= -100:  # negative values [-1, -100]
            self.brake_set_pwm(int((duty_cycle / 100.0) * (PWM_STOP - PWM_MIN) + PWM_STOP))
        else:
            self.brake_set_pwm(PWM_STOP)
