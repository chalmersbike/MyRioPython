from param import *
import math
import time

class Encoder(object):
    encoder = None

    def __init__(self,session):
        self.fpga_EncoderCounts = session.registers['Encoder Counts']
        self.fpga_EncoderPositionReset = session.registers['Encoder Position Reset']
        self.calibrate()
        print('ENCODER INIT')
    def calibrate(self):
        self.fpga_EncoderPositionReset.write(True)

    def get_angle(self):
        return -self.fpga_EncoderCounts.read() * steeringEncoder_TicksToRadianRatio
