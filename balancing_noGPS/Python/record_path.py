from bike import Bike
from controller import Controller
from actuators import DriveMotor, SteeringMotor
import sys
import getopt
# import pysnooper

try:
    # Get the command line arguments
    straight = False
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hs", ["straight"])
    except getopt.GetoptError:
        print('record_path.py -s')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('record_path.py -s')
            sys.exit()
        elif opt in ("-s", "--straight"):
            straight = True

    print('Starting Bike with controller INACTIVE to record path\n')

    bike = Bike(debug=False,recordPath=True,straight=straight)
except (ValueError, KeyboardInterrupt):

    rearmotor = DriveMotor()
    rearmotor.stop()
    steeringmotor = SteeringMotor()
    steeringmotor.stop()
    GPIO.cleanup()

    # bike.controller.log_regular_recordPath()

    # print('sensor reading time is %g' % bike.controller.sensor_reading_time
    # print('\n Error detected, all the control signals terminated...'
    exc_msg = '\n Error detected by record_path program, all the control signals terminated...'
    print(exc_msg)
    print(ValueError)
    # bike.exception_log(-2, exc_msg)