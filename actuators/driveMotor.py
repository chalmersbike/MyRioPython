from param import *
import serial, time, numpy
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.ADC as ADC


class DriveMotor(object):
    serial = None

    def __init__(self):
        UART.setup(driveMotor_UARTPort)
        ADC.setup()
        self.serial = serial.Serial(port=driveMotor_port, baudrate=driveMotor_CommunicationFrequency)
        self.serial.close()
        self.serial.open()
        if debug:
            print 'Drive Motor : Serial port opened'

    def serial_write_character(self, character):
        if self.serial.isOpen():
            self.serial.write(character)
        else:
            print 'Drive Motor : Serial port is not open!'
            self.serial.close()
            self.serial.open()
            self.serial.write(character)

    def rear_set_rpm(self, rpm):
        rpm_string = 'run -s ' + str(rpm).zfill(4) + ' -f 5 -pi\n'
        self.serial.write(rpm_string)
        # for character in pwm_string:
        #     self.serial_write_character(character)
        #     time.sleep(0.001)  # fixes comms issue

    def set_velocity(self, input_velocity):
        # self.rear_set_pwm(self._m_per_second_to_pwm(input_velocity))
        self.rear_set_rpm(input_velocity*0.2)
        # for GEAR 6Th the coef vel -> pwm = 0.31
        # self.rear_set_rpm(input_velocity * 0.31)

    def stop(self):
        stop_string = 'run -stop\n'
        self.serial.write(stop_string)
        # for character in pwm_string:
        #     self.serial_write_character(character)
        #     time.sleep(0.001)  # fixes comms issue