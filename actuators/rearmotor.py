import serial, time, numpy
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.ADC as ADC

CURRENT_SENSE_PORT = 'P9_38'
RPM_SENSE_PORT = 'P9_40'
REAR_MOTOR_PORT = '/dev/ttyO1'
COMMUNICATION_FREQUENCY = 115200

PWM_STOP = 1050
PWM_MIN = 1265  # Corresponds to 3600 RPM which is minimum speed for ESCON 50/4
PWM_MAX = 2100

#!!!!!!!Not used anymore!!!!!!

class RearMotor(object):
    serial = None
    def __init__(self):
        UART.setup("UART1")
        ADC.setup()
        self.serial = serial.Serial(port=REAR_MOTOR_PORT, baudrate=COMMUNICATION_FREQUENCY)
        self.serial.close()
        self.serial.open()
        self.GearNr = 1
    def serial_write_character(self, character):
        if self.serial.isOpen():
            self.serial.write(character)
        else:
            print 'Serial is not open!'
            self.serial.close()
            self.serial.open()
            self.serial.write(character)

    def rear_set_pwm(self, pwm):
        pwm_string = str(pwm).zfill(
            4) + '%'  # the '%' symbol indicates to the MSP430 that this is a signal for the Shimano motor
        # print 'pwm_string = %s\n' %(pwm_string)
        for character in pwm_string:
            self.serial_write_character(character)
            time.sleep(0.001)  # fixes comms issue

    def set_velocity(self, input_velocity):
        self.rear_set_pwm(self._m_per_second_to_pwm(input_velocity))

    def stop(self):
        self.rear_set_pwm(PWM_STOP)

    def _m_per_second_to_pwm(self, m_per_second):
        # calculated from shimano speed test 15/8/18. Please see Shimano Motor Validation file for more information.
        if self.GearNr == 1:
            omega = -14.539 * numpy.power(m_per_second, 5) + 166.0 * numpy.power(m_per_second,
                                                                                 4) - 710.33 * numpy.power(
                m_per_second, 3) + 1403.5 * numpy.power(m_per_second, 2) + 3297.1 * m_per_second  # for 1st gear
            pwm = int(round(((omega - 3245) / (17371.0 - 3245)) * (PWM_MAX - PWM_MIN) + PWM_MIN))  # for 1st gear
        elif self.GearNr == 6:
            omega = 1.0694 * numpy.power(m_per_second, 4) - 20.478 * numpy.power(m_per_second,
                                                                                 3) + 136.44 * numpy.power(
                m_per_second, 2) + 1819.1 * m_per_second  # for 6th gear
            pwm = int(round(((omega - 3308) / (17764.0 - 3308)) * (PWM_MAX - PWM_MIN) + PWM_MIN))  # for 6st gear


        # omega = 1.0694 * numpy.power(m_per_second, 4) - 20.478 * numpy.power(m_per_second, 3) + 136.44 * numpy.power(
        #    m_per_second, 2) + 1819.1 * m_per_second #for 6th gear
        # omega = -14.539 * numpy.power(m_per_second, 5) + 166.0 * numpy.power(m_per_second, 4) - 710.33 * numpy.power(
        #     m_per_second, 3) + 1403.5 * numpy.power(m_per_second, 2) + 3297.1 * m_per_second  # for 1st gear
        # pwm = int(round(((omega - 3308) / (17764.0 - 3308)) * (PWM_MAX - PWM_MIN) + PWM_MIN)) #for 6st gear
        # pwm = int(round(((omega - 3245) / (17371.0 - 3245)) * (PWM_MAX - PWM_MIN) + PWM_MIN))  # for 1st gear

        if pwm > PWM_MAX:
            return PWM_MAX
        elif pwm < PWM_MIN:
            return PWM_STOP  #PWM_MIN
        else:
            return pwm

    def read_motor_current(self):
        # Use beaglebone ADC to detect current. Input is scaled from 0-1 where 0 = 0 A and 1 = 10A.
        return round(float(ADC.read(CURRENT_SENSE_PORT) * 10.0), 2)

    def read_motor_rpm(self):
        # Use beaglebone ADC to detect Shimano motor RPM. Input is scaled from 0-1 where 0 = 0 RPM and 1 = 18,000 RPM.
        return round(float(ADC.read(RPM_SENSE_PORT) * 18000.0), 2)
