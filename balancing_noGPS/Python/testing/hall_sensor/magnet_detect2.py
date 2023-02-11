from time import sleep

import Adafruit_BBIO.GPIO as GPIO

INPUT_PORT = 'P9_12'
GPIO.setup(INPUT_PORT, GPIO.IN)


def magnet_detect():
    print "MAGNET DETECTED!"

GPIO.add_event_detect(INPUT_PORT, GPIO.FALLING, callback=magnet_detect)

if __name__ == '__main__':
    while True:
        print "0"
        sleep(0.2)
