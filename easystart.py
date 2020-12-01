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
    path_file_arg = ''
    rollref_file_arg = ''
    steeringdist_file_arg = ''
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hRSp:r:s:", ["reverse","straight","path=","rollref=","steeringdist="])
    except getopt.GetoptError:
        print 'easystart.py -R -S -p "path_file.csv" -r "rollref_file.csv" -s "steeringdist_file.csv"'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'easystart.py -R -S -p "path_file.csv" -r "rollref_file.csv" -s "steeringdist_file.csv"'
            sys.exit()
        elif opt in ("-S", "--straight"):
            straight = True
        elif opt in ("-R", "--reverse"):
            reverse = True
        elif opt in ("-p","--path"):
            path_file_arg = arg
        elif opt in ("-r","--rollref"):
            rollref_file_arg = arg
        elif opt in ("-s","--steeringdist"):
            steeringdist_file_arg = arg

    print 'Starting Bike with controller ACTIVE\n'

    bike = Bike(debug=False,recordPath=False,reverse=reverse,straight=straight,path_file_arg=path_file_arg,rollref_file_arg=rollref_file_arg,steeringdist_file_arg=steeringdist_file_arg)
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