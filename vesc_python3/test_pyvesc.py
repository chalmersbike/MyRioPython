import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
import sys
import getopt
from pyvesc import VESC
import time
import pysnooper

# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
driveMotor_port  = '/dev/ttyO4'             # Drive Motor serial port
driveMotor_UARTPort = "UART4"               # Drive Motor UART port


# a function to show how to use the class with a with-statement
@pysnooper.snoop()
def run_motor_using_with():
    print('FUNCTION start')
    UART.setup(driveMotor_UARTPort)
    with VESC(serial_port=driveMotor_port) as motor:
        # print("Firmware: ", motor.get_firmware_version())
        # motor.set_duty_cycle(.02)
        motor.set_rpm(3000)
        time.sleep(5)
        motor.set_rpm(-3000)
        time.sleep(5)

        # run motor and print out rpm for ~2 seconds
        # for i in range(30):
        #     time.sleep(0.1)
        #     print(motor.get_measurements().rpm)
        # motor.set_rpm(0)


# a function to show how to use the class as a static object.
def run_motor_as_object():
    motor = VESC(serial_port=driveMotor_port)
    # print("Firmware: ", motor.get_firmware_version())

    # sweep servo through full range
    for i in range(100):
        time.sleep(0.01)
        # motor.set_servo(i/100)
        motor.set_rpm(i*40)

    # IMPORTANT: YOU MUST STOP THE HEARTBEAT IF IT IS RUNNING BEFORE IT GOES OUT OF SCOPE. Otherwise, it will not
    #            clean-up properly.
    motor.stop_heartbeat()


def time_get_values():
    with VESC(serial_port=driveMotor_port) as motor:
        start = time.time()
        motor.get_measurements()
        stop = time.time()
        print("Getting values takes ", stop-start, "seconds.")


if __name__ == '__main__':
    run_motor_using_with()
    # run_motor_as_object()
    # time_get_values()
