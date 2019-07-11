import Adafruit_BBIO.GPIO as GPIO
from time import sleep

OUTPUT_PORT = 'P8_10'


class RearMotor(object):

    def __init__(self):
        GPIO.setup(OUTPUT_PORT, GPIO.OUT)
        GPIO.output(OUTPUT_PORT, GPIO.HIGH)
        sleep(3)
        GPIO.output(OUTPUT_PORT, GPIO.LOW)
        sleep(2)

    def start(self):
        GPIO.output(OUTPUT_PORT, GPIO.HIGH)

    def stop(self):
        GPIO.output(OUTPUT_PORT, GPIO.LOW)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()
