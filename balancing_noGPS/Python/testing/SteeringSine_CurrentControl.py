from actuators import SteeringMotor as Steering
from sensors import  Encoder, SafetyStop
import time
import numpy as np
from utils import PID
import math
import csv

Ts = 0.002
deg2rad = 3.14/180.0

try:
    safety_stop = SafetyStop()
    mot = Steering()
    enc = Encoder()
    freq = 10#Hz (0.1,0.2,0.5,1,2,5,10,20,50)
    omega = 2*math.pi*freq
    amplitude = 0.8 # A

    timestr = time.strftime("%Y%m%d-%H%M%S")
    results = open('tests_steering_sine_CurrentControl/freq%f_amplitude%f_%s.csv' %(freq,amplitude,timestr), 'wb')

    writer = csv.writer(results)
    writer.writerow(('Time (s)', 'Reference Current (A)', 'Measured Steering Angle (rad)'))

    mot.set_current(0)
    input1 = raw_input('Encoder initial position set. Press Enter to start the motor!')
    mot.enable()

    print 'Time \t\t\t\t Reference Current \t Measured Steering Angle'

    starttime = time.time()
    while time.time() - starttime < 20/freq and not safety_stop.button_check():
    # while time.time() - starttime < 1000 and not safety_stop.button_check():
        looptime = time.time()
        delta = enc.get_angle()

        current = amplitude * math.cos(omega * (time.time() - starttime))
        # current = 0.01*(time.time()-starttime)

        mot.set_current(current)
        # mot.set_current(0)

        print str(time.time() - starttime) + ' \t\t ' + str(current) + ' \t\t ' + str(delta)
        writer.writerow((time.time() - starttime, current, delta))

        timeremain = Ts - time.time() + looptime
        if timeremain > 0:
            time.sleep(timeremain)
    mot.stop()
except (ValueError, KeyboardInterrupt):
    mot.stop()
    print"The Control loop has been terminated"
