from steeringmotor import SteeringMotor
# from sensor import  Encoder

import time
starttime = time.time()
mot = SteeringMotor()
# enc = Encoder()
direction = 1
TestSpeed = 0.4
AngVel = TestSpeed
# mot.set_angular_velocity(AngVel)
prevAng = 0.0
try:
    temptime = time.time()
    while time.time() - starttime < 80:
        looptime = time.time()

        if looptime - temptime < 5.0:
            AngVel = direction * TestSpeed
        else:
            temptime = time.time()
            direction = -direction
            AngVel = direction * TestSpeed
        mot.set_angular_velocity(AngVel)


        timeremain = 0.04 - time.time() + looptime
        if timeremain > 0:
            # print "I am sleeping"
            time.sleep(timeremain)
            Flag = mot.read_UART()
            if Flag != 1:
                print 'TIME= ' + str(time.time() - starttime) + ' input= ' + str(AngVel) + '  Output= ' + str(Flag)


    mot.stop()
except (ValueError, KeyboardInterrupt):
    mot.stop()