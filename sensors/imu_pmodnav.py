import time

from magum import Magum

RAD = 'rad'
GYROSCOPE_RANGE = 1000  # represents the range of the gyroscope. It can assume the following values: 250,500,1000,
# 2000 dps (degrees per second).
FS_DOUBLE_ENABLED = False  # enables (1) or disables (0) the fsdouble function to double the scale range of the
# gyroscope
ACCELEROMETER_RANGE = 2  # represents the range of the accelerometer. In can assume 2 for +/- 2g, 4 for +/- 4g or 8
# for +/- 8g
NOISE_ENABLED = True  # enables (1) or disables (0) the lnoise (high sensitivity, low noise) function for the
# accelerometer sensor. When active the maximum signal that can be measured is +/- 4g.
CALIBRATION_SAMPLES = 50


class IMU(object):
    magum = None
    calibration_array = None

    def __init__(self):
        self.magum = Magum(GYROSCOPE_RANGE, int(FS_DOUBLE_ENABLED),
                           ACCELEROMETER_RANGE, int(NOISE_ENABLED))
        self.calibrate()

    def calibrate(self):
        self.calibration_array = self.magum.calibrateSens(CALIBRATION_SAMPLES)

    def get_roll_angle(self):
        try:
            return self.magum.compFilter(DT=0.02, axisOffset=self.calibration_array, uM=RAD)[0]
        except IOError:
            print 'IOERROR'
            self.magum.toStandby('g')
            self.magum.toStandby('a')
            time.sleep(0.3)
            self.magum.toActive('g')
            self.magum.toActive('a')
            return self.magum.compFilter(DT=0.02, axisOffset=self.calibration_array, uM=RAD)[0]

    def get_roll_angular_velocity(self):
        try:
            return self.magum.readGData(self.calibration_array, RAD)[1]
        except IOError:
            print 'IOERROR'
            self.magum.toStandby('g')
            self.magum.toStandby('a')
            # time.sleep(0.3)
            self.magum.toActive('g')
            self.magum.toActive('a')
            return self.magum.readGData(self.calibration_array, RAD)[1]
