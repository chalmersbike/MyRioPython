from param import *
import math
from time import sleep, time


class HallSensor(object):
    last_time_measured = 0.0
    velocity = 0.0
    elapse = 0.0

    def __init__(self,session):
        self.fpga_HallPeriod = session.registers['Hall Period (uSec/pulse)']
        self.fpga_HallEdgeDetection = session.registers['Hall Edge Detection ']
        self.Hall_count = session.registers['Hall counts']
        session.registers['Bounce time (uSec)'] = 20000
        session.registers['Hall Edge Detection '] = "rising"
        self._initialize_interrupt()

        print('HALL INIT')
    def _initialize_interrupt(self):
        self.last_time_measured = time()

        if hallSensor_edgeDetection == 'rising':
            self.fpga_HallEdgeDetection.write(1)
        elif hallSensor_edgeDetection == 'falling':
            self.fpga_HallEdgeDetection.write(0)
        else:
            print("Hall sensor : Chosen Hall sensor edge detection type is not valid : %s. Choosing rising edge instead" %(hallSensor_edgeDetection))
            self.fpga_HallEdgeDetection.write('rising')

    def get_velocity(self, ref_velocity):
        self.elapse = self.fpga_HallPeriod.read()*1e-6
        # print(self.elapse)
        # print('Loops in Hall \n')
        # print(self.Hall_count)
        if self.elapse > hallSensor_maxElapseBetweenPulses:
            self.velocity = 0.0
        else:
            self.velocity = (0.0 if not self.elapse
                             else 0.0 if self.elapse > hallSensor_maxElapseBetweenPulses
            else (hall_Sensor_distanceBetweenMagnets / self.elapse) * tyre_ratio)
        # print(self.velocity)
        # if self.velocity > 1.2 * ref_velocity:
        #     self.velocity = ref_velocity
        return self.velocity

    def get_velocity_kmh(self):
        return (self.velocity * kmph2mps) or 0  # velocity is measured in km/h

