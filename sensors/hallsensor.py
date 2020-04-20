import math

import Adafruit_BBIO.GPIO as GPIO
from time import sleep, time

INPUT_PORT = 'P9_30'  # Old value is 9_12, while the pin is broken by Yixiao :-(
WHEEL_DIAMETER = 0.694  # m (Tyre marking 40-622 = ID of 622mm) + 4mm to center of magnet
TYRE_RATIO = 0.7 / WHEEL_DIAMETER
NUMBER_OF_SENSORS = 3
MAX_ELAPSE_BETWEEN_PULSES = 1  # seconds. Stationary if longer then this.

WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
DISTANCE_BETWEEN_SENSORS = WHEEL_CIRCUMFERENCE / NUMBER_OF_SENSORS


class HallSensor(object):
    last_time_measured = 0.0
    velocity = 0.0
    elapse = 0.0

    def __init__(self):
        GPIO.setup(INPUT_PORT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self._initialize_interrupt()

    def _initialize_interrupt(self):
        self.last_time_measured = time()
        # GPIO.add_event_detect(INPUT_PORT, GPIO.FALLING, callback=self.update_velocity)
        GPIO.add_event_detect(INPUT_PORT, GPIO.RISING, callback=self.update_velocity,bouncetime=30) # added a 5ms bouncetime to avoid counting the same falling edge multiple times

    def update_velocity(self, *args):
        time_measured = time()
        self.elapse = time_measured - self.last_time_measured
        self.velocity = (0.0 if not self.elapse
                         else 0.0 if self.elapse > MAX_ELAPSE_BETWEEN_PULSES
        else (DISTANCE_BETWEEN_SENSORS / self.elapse) * TYRE_RATIO)
        self.last_time_measured = time_measured
        # print 'MAGNET DETECTED!I AM UPDATING THE VELOCITY BY CALCULATING THE TIME INTERVAL'

    def get_velocity(self):
        self.elapse = time() - self.last_time_measured
        if self.elapse > MAX_ELAPSE_BETWEEN_PULSES:
            self.velocity = 0.0
            return self.velocity
        else:
            return self.velocity

        return self.velocity
    def get_velocity_kmh(self):
        return (self.velocity * 3.6) or 0  # velocity is measured in km/h

