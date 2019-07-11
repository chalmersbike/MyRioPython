import serial, time
import Adafruit_BBIO.UART as UART
from lowpass import butter_lowpass,butter_lowpass_filter

STEERING_MOTOR_PORT = '/dev/ttyO1'
COMMUNICATION_FREQUENCY = 115200

PWM_MIN = 230  # OLD VALUE=1050
PWM_STOP = 1055
PWM_MAX = 1870  # OLD VALUE=2100

# Filter requirements.
order = 2
fs = 25       # sample rate, Hz
cutoff = 3  # desired cutoff frequency of the filter, Hz

# Get the filter coefficients so we can check its frequency response.
b, a = butter_lowpass(cutoff, fs, order)


class SteeringMotor(object):

    serial = None

    def __init__(self):
        UART.setup("UART1")
        self.serial = serial.Serial(port=STEERING_MOTOR_PORT, baudrate=COMMUNICATION_FREQUENCY)
        self.serial.close()
        self.serial.open()

        self.pwm_list = []

    def serial_write_character(self, character):
        if self.serial.isOpen():
            self.serial.write(character)
        else:
            print 'Serial is not open!'
            self.serial.close()
            self.serial.open()
            self.serial.write(character)

    def steer_set_pwm(self, pwm):
        # Filter test
        self.pwm_list.append(pwm)
        self.pwm_filt_ucon = butter_lowpass_filter(self.pwm_list, cutoff, fs, order)
        self.pwm_filt = int(max(min(PWM_MAX,self.pwm_filt_ucon[-1]),PWM_MIN))
        pwm_string = str(self.pwm_filt).zfill(4) + '#'
        # print 'pwm = ' + str(pwm) + ' ; pwm_string_filt = ' + pwm_string

        pwm_string = str(pwm).zfill(4) + '#'
        self.pwm_str = pwm_string
        # #print 'pwm_string = %s\n' %(pwm_string)
        # print 'pwm_string = ' + pwm_string

        for character in pwm_string:
            self.serial_write_character(character)
            time.sleep(0.001)  # fixes comms issue

    def set_angular_velocity(self, angular_velocity):
        self.steer_set_pwm(self._rad_per_second_to_pwm(-angular_velocity)) # Use a minus sign to spin counter-clockwise for a positive angular speed

    def stop(self):
        self.steer_set_pwm(PWM_STOP)

    def _rad_per_second_to_pwm(self, rad_per_second):
        # updated from hysteresis test 15/8/18
        if rad_per_second > 0:
            pwm = 1040 - (rad_per_second / 7.091) * (1040 - 230)
        elif rad_per_second < 0:
            pwm = 1070 - (rad_per_second / 7.02) * (1870 - 1070)
        else:
            pwm = 1055

        pwm = int(pwm)

        if pwm > PWM_MAX:
            return PWM_MAX
        elif pwm < PWM_MIN:
            return PWM_MIN
        else:
            return pwm
    def read_UART(self):
        signal_read = self.serial.read(5)
        if abs(int(signal_read[0:3]) - int(self.pwm_str[0:3])) > 2:
            print 'The Reading doesnt match  Read: ' + signal_read + '  Sent: '+ self.pwm_str
            return signal_read
        else:
            return 1