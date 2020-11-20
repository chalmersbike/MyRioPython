print 'Starting Bike with controller ACTIVE\n'

from bike import Bike
from controller import Controller
import Adafruit_BBIO.GPIO as GPIO
from actuators import DriveMotor, SteeringMotor
import sys
import getopt
# import pysnooper

try:
    # Get the command line arguments
    reverse = False
    straight = False
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hrs", ["reverse","straight"])
    except getopt.GetoptError:
        print 'easystart.py -r -s'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'easystart.py -r -s'
            sys.exit()
        elif opt in ("-s", "--straight"):
            straight = True
        elif opt in ("-r", "--reverse"):
            reverse = True

    bike = Bike(debug=False,recordPath=False,reverse=reverse,straight=straight)
except (ValueError, KeyboardInterrupt):
    rearmotor = DriveMotor()
    rearmotor.stop()
    steeringmotor = SteeringMotor()
    steeringmotor.stop()
    GPIO.cleanup()
    # print 'sensor reading time is %g' % bike.controller.sensor_reading_time
    # print '\n Error detected, all the control signals terminated...'
    exc_msg = '\n Error detected by easystart program, all the control signals terminated...'
    print(exc_msg)
    print ValueError
    # bike.exception_log(-2, exc_msg)