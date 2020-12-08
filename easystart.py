from bike import Bike
from controller import Controller
import Adafruit_BBIO.GPIO as GPIO
from actuators import DriveMotor, SteeringMotor
import sys
import getopt
# import pysnooper

try:
    # Variable to check if the -h argument was used
    need_help = False

    # Get the command line arguments
    reverse = False
    straight = False
    path_file_arg = ''
    rollref_file_arg = ''
    steeringdist_file_arg = ''
    simulate_file = ''

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hRSp:r:s:m:", ["reverse","straight","path=","rollref=","steeringdist=","simulate="])
    except getopt.GetoptError:
        print 'Use \'python easystart.py -h\' for help'
        print 'easystart.py -R -S -p "path_file.csv" -r "rollref_file.csv" -s "steeringdist_file.csv" -m "experimentData_file.csv"'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            need_help = True
            print 'easystart.py -R -S -p "path_file.csv" -r "rollref_file.csv" -s "steeringdist_file.csv" -m "experimentData_file.csv"'
            print '\t-R\t\t\tread path in reverse'
            print '\t-S\t\t\tread only first and last point of path to go in a straight line'
            print '\t-p\t\t\tselect reference path file name'
            print '\t-r\t\t\tselect reference roll file name'
            print '\t-d\t\t\tselect steering angle disturbance file name'
            print '\t-m\t\t\tselect a previous experiment data file and run code using this data instead of sensor measured data'
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
        elif opt in ("-m","--steeringdist"):
            simulate_file = arg

    if not need_help:
        print 'Starting Bike with controller ACTIVE\n'

        bike = Bike(debug=False,recordPath=False,reverse=reverse,straight=straight,path_file_arg=path_file_arg,rollref_file_arg=rollref_file_arg,steeringdist_file_arg=steeringdist_file_arg,simulate_file=simulate_file)
except (ValueError, KeyboardInterrupt):
    if simulate_file == '':
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