import Adafruit_BBIO.GPIO as GPIO
from param import *
from sensors import Encoder, HallSensor, IMU, SafetyStop, GPS
from record_path_latlon import RecordPathLatLon
import pysnooper

class Bike(object):
    # @pysnooper.snoop()
    def __init__(self, debug='True'):
        # Initialize sensors and actuators
        self.safety_stop = SafetyStop()
        self.encoder = Encoder()
        self.hall_sensor = HallSensor()
        self.imu = IMU()
        if gps_use:
            self.gps = GPS()

        # Run bike and controllers
        self.pathRecorder = RecordPathLatLon(self)
        # self.controller.startup()
        self.pathRecorder.run()

    # Safety Stop
    def emergency_stop_check(self):
        return self.safety_stop.button_check()

    # Steering Encoder
    def get_handlebar_angle(self):
        return self.encoder.get_angle()

    # Hall Sensor
    def get_velocity(self):
        return self.hall_sensor.get_velocity()

    # IMU
    def get_imu_data(self, velocity, delta_state, phi):
        return self.imu.get_imu_data(velocity, delta_state, phi)

    # GPS
    if gps_use:
        def get_gps_data(self):
            return self.gps.get_position()

    def stop_all(self):
        GPIO.cleanup()


try:
    bike = Bike(debug=False)
except (ValueError, KeyboardInterrupt):
    GPIO.cleanup()
    exc_msg = '\n Error detected by record_path program ...'
    print(exc_msg)
    print ValueError