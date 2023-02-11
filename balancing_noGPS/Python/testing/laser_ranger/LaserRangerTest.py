from sensors import DualLaserRanger
import time
a = DualLaserRanger()
while 1:
    print a.get_y()
    time.sleep(0.5)
