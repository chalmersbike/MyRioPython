import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
Pin_enable = 'P9_22'
#set polarity to 1 on start:
#PWM.start("P9_14", 50, 2000, 1)

#PWM.start(channel, duty, freq=2000, polarity=0)
#duty values are valid 0 (off) to 100 (on)

# PWM.start("P9_14", 25,100)
PWM.start("P9_21", 25,100)
# PWM.set_duty_cycle("P9_14", 25.5)
# PWM.set_frequency("P9_14", 10)
GPIO.setup(Pin_enable, GPIO.OUT)
GPIO.output(Pin_enable, GPIO.HIGH)
print "BB Set, now sleeping"
time.sleep(100)
# PWM.stop("P9_14")
GPIO.setup(Pin_enable, GPIO.OUT)
GPIO.output(Pin_enable, GPIO.HIGH)
PWM.stop("EHRPWM1A")
PWM.cleanup()

print "BB cleaned up"
