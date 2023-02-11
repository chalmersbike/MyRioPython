from actuators import Steering
from sensors import Encoder, SafetyStop
import time
import numpy as np
DDeltaRate = 0.05
AngleRange = 0.4
# AngleRange = 9999

try:
    mot = Steering()
    starttime = time.time()
    # safety_stop = SafetyStop()
    enc = Encoder()
    direction = 1
    TestSpeed = 0.1
    AngVel = TestSpeed
    AngVelOld = 0
    mot.set_angular_velocity(AngVel)
    prevAng = 0.0
    print 'Time;' +'delta;' + 'angVelref;'+'Realized'
    while time.time() - starttime < 900: #and not safety_stop.button_check():
        looptime = time.time()
        delta = enc.get_angle()
        RealizedAngVel = (delta - prevAng)/0.04

        if delta > AngleRange:
            AngVel = -TestSpeed
            direction = -1

        elif delta < -AngleRange:
            AngVel = TestSpeed
            direction = 1

        else:
            if direction == 1:
                AngVel = TestSpeed
            elif direction == -1:
                AngVel = -TestSpeed
            else:
                print "Exception in Direction Flag"


        prevAng = delta
        timeremain = 0.04 - time.time() + looptime
        if timeremain > 0:
            time.sleep(timeremain)
        if abs(AngVel - AngVelOld)> DDeltaRate:
           AngVel = AngVelOld + np.sign(AngVel - AngVelOld) * DDeltaRate

        mot.set_angular_velocity(AngVel)
        AngVelOld = AngVel
        print str(time.time() - starttime) + ';  ' + str(delta) + ';  ' + str(AngVel) + '; ' + str(RealizedAngVel)
    mot.stop()
except (ValueError, KeyboardInterrupt):
    mot.stop()