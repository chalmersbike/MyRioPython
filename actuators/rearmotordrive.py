import serial, time
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.ADC as ADC


#REAR_MOTOR_PORT = '/dev/ttyO1'
REAR_MOTOR_PORT = '/dev/ttyS1'
COMMUNICATION_FREQUENCY = 115200

PWM_STOP = 1050
PWM_MIN = 1265  # Corresponds to 3600 RPM which is minimum speed for ESCON 50/4
PWM_MAX = 2100


class RearMotorDrive(object):
    serial = None

    def __init__(self):
        UART.setup("UART1")
        ADC.setup()
        self.serial = serial.Serial(port=REAR_MOTOR_PORT, baudrate=COMMUNICATION_FREQUENCY)
        self.serial.close()
        self.serial.open()

    def serial_write_character(self, character):
        if self.serial.isOpen():
            self.serial.write(character)
        else:
            print 'Serial is not open!'
            self.serial.close()
            self.serial.open()
            self.serial.write(character)

    def rear_set_rpm(self, rpm):
        rpm_string = 'run -s ' + str(rpm).zfill(4) + ' -f 5 -pi\n'
        # pwm_string = "run -s 3 -f 5-pi\n"
        self.serial.write(rpm_string)
        # for character in pwm_string:
        #     self.serial_write_character(character)
        #     time.sleep(0.001)  # fixes comms issue

    def set_velocity(self, input_velocity):
        # self.rear_set_pwm(self._m_per_second_to_pwm(input_velocity))
        self.rear_set_rpm(input_velocity*0.213)
        # for GEAR 6Th the coef vel -> pwm = 0.31

    def stop(self):
        pwm_string = 'run -stop\n'
        for character in pwm_string:
            self.serial_write_character(character)
            time.sleep(0.001)  # fixes comms issue

    def _m_per_second_to_pwm(self, m_per_second):

        if pwm > PWM_MAX:
            return PWM_MAX
        elif pwm < PWM_MIN:
            return PWM_STOP  #PWM_MIN
        else:
            return pwm
