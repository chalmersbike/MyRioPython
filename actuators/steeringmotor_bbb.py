
import Adafruit_BBIO.PWM as PWM

PWM_PIN = 'P8_13'
PWM_FREQUENCY = 50 # 50Hz = 20ms period


class SteeringMotor(object):

    def __init__(self):
        PWM.start(PWM_PIN, duty_cycle=0, frequency=PWM_FREQUENCY)

    def set_angular_velocity(self, angular_velocity):
        pulse_width = self._rad_per_second_to_pulse_width(angular_velocity)
        duty_cycle = self._pulse_width_to_duty_cycle(pulse_width)
        self.set_duty_cycle(duty_cycle)

    def _rad_per_second_to_pulse_width(self, rad_per_second):
        return 85.18196 * rad_per_second + 1587.33

    def _pulse_width_to_duty_cycle(self, pulse_width):
        duty_cycle = ((pulse_width / 1000.0) / 20) * 100  # convert to ms, then find % of total PWM period

    def set_duty_cycle(self, duty_cycle):
        PWM.set_duty_cycle("P8_13", duty_cycle)
