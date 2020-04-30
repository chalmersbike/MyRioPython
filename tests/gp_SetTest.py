import time

import Adafruit_BBIO.GPIO as GPIO
# from time import sleep, time
INPUT_PORT = 'P9_26'
#INPUT_PORT = 'GP1_4'
# INPUT_PORT = 'RED_LED'
# INPUT_PORT = 'GREEN_LED'
# INPUT_PORT = 'GPIO3_17'
# INPUT_PORT = 'SPI1_CS0'
#GPIO.setup(INPUT_PORT, GPIO.OUT, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INPUT_PORT, GPIO.IN, pull_up_down =GPIO.PUD_UP)
print "GPIO RED has been configed Pulled UP"

for x in range(1, 100 + 1):
    if GPIO.input(INPUT_PORT):
        ESTOP = False
        print 'Estop Off'
    else:
        ESTOP = True
        print 'Estop On'
    #GPIO.output(INPUT_PORT, GPIO.HIGH)
    #print "high"
    #time.sleep(1)
    #GPIO.output(INPUT_PORT, GPIO.LOW)
    #print "low"
    time.sleep(1)

print "Test DONE"
