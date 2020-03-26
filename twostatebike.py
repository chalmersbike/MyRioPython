import Adafruit_BBIO.GPIO as GPIO
from sensors import Potentiometer
from actuators import RearMotorDrive, Steering
from twostate_controller import Controller
# from twostate_controller_wtout_pred import Controller

from sensors import Encoder, HallSensor, IMU, SafetyStop, GPS
#from kalman_filter import Kalman_filter as kf
#import pysnooper

class Bike(object):
    # @pysnooper.snoop()
    def __init__(self, debug='True'):
        self.safety_stop = SafetyStop()
        self.encoder = Encoder()
        self.hall_sensor = HallSensor()
        self.rear_motor = RearMotorDrive()
        self.steering_motor = Steering()
        self.a_imu = IMU()
        # self.gps = GPS()
        self.potent = Potentiometer()
        # self.kf = kf(self)
        self.laser_ranger = DualLaserRanger()
        if not debug:
            self.controller = Controller(self)
            self.controller.startup()
            self.controller.run()
            # self.controller.start()

    def set_velocity(self, input_velocity):
        self.rear_motor.set_velocity(input_velocity)

    def stop(self):
        self.steering_motor.stop()
        self.rear_motor.stop()

    def get_handlebar_angle(self):
        return self.encoder.get_angle()

    def get_handlebar_angular_velocity(self):
        return self.handlebar_angular_velocity

    def set_handlebar_angular_velocity(self, angular_velocity):
        self.steering_motor.set_angular_velocity(angular_velocity)
        self.handlebar_angular_velocity = angular_velocity

    def get_velocity(self):
        return self.hall_sensor.get_velocity()

    def get_imu_data(self):
        return self.a_imu.get_imu_data()

    def emergency_stop_check(self):
        return self.safety_stop.button_check()

    def stop_all(self):
        self.stop()
        # self.set_handlebar_angular_velocity(0)
        self.steering_motor.stop()
        GPIO.cleanup()
