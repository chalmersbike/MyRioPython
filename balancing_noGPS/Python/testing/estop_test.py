from time import sleep

import Adafruit_BBIO.GPIO as GPIO

INPUT_PORT = 'P9_15'
GPIO.setup(INPUT_PORT, GPIO.IN)


def button_detect(self):
    print "ESTOP!"
    return None


GPIO.add_event_detect(INPUT_PORT, GPIO.FALLING, callback=button_detect)

if __name__ == '__main__':
    while True:
        print "0"
        sleep(0.02)
