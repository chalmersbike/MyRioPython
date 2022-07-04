from param import *
import math
import Adafruit_BBIO.GPIO as GPIO
from time import sleep, time


class HallSensor(object):
    last_time_measured = 0.0
    velocity = 0.0
    elapse = 0.0

    def __init__(self):
        if hallSensor_pullUpDown == 'up':
            GPIO.setup(hallSensor_port, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        elif hallSensor_pullUpDown == 'down':
            GPIO.setup(hallSensor_port, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        else:
            print("Hall sensor : Chosen Hall sensor pull-up or pull-down type is not valid : %s. Choosing pull-up instead" %(hallSensor_pullUpDown))
            GPIO.setup(hallSensor_port, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self._initialize_interrupt()

    def _initialize_interrupt(self):
        self.last_time_measured = time()
        if hallSensor_edgeDetection == 'rising':
            GPIO.add_event_detect(hallSensor_port, GPIO.RISING, callback=self.update_velocity,bouncetime=hallSensor_bounceTime)
        elif hallSensor_edgeDetection == 'falling':
            GPIO.add_event_detect(hallSensor_port, GPIO.FALLING, callback=self.update_velocity,bouncetime=hallSensor_bounceTime)
        else:
            print("Hall sensor : Chosen Hall sensor edge detection type is not valid : %s. Choosing rising edge instead" %(hallSensor_edgeDetection))
            GPIO.add_event_detect(hallSensor_port, GPIO.RISING, callback=self.update_velocity,bouncetime=hallSensor_bounceTime)

    def update_velocity(self, *args):
        time_measured = time()
        self.elapse = time_measured - self.last_time_measured
        self.velocity = (0.0 if not self.elapse
                         else 0.0 if self.elapse > hallSensor_maxElapseBetweenPulses
        else (hall_Sensor_distanceBetweenMagnets / self.elapse) * tyre_ratio)
        self.last_time_measured = time_measured
        print('MAGNET DETECTED!I AM UPDATING THE VELOCITY BY CALCULATING THE TIME INTERVAL')

    def get_velocity(self):
        self.elapse = time() - self.last_time_measured
        if self.elapse > hallSensor_maxElapseBetweenPulses:
            self.velocity = 0.0
            return self.velocity
        else:
            return self.velocity

    def get_velocity_kmh(self):
        return (self.velocity * kmph2mps) or 0  # velocity is measured in km/h

