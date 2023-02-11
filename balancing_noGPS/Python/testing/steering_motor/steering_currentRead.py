import Adafruit_BBIO.ADC as ADC
import time

ADC.setup()

for i in range(0, 100, 1):
    print(ADC.read('P9_39'))
    time.sleep(0.1)
