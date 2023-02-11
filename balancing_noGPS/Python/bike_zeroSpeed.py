import Adafruit_BBIO.GPIO as GPIO
from param import *
from sensors import Encoder, HallSensor, IMU, SafetyStop, Potentiometer, DualLaserRanger, GPS
from actuators import DriveMotor, SteeringMotor
from controller_zeroSpeed import Controller
import pysnooper

class Bike(object):
    # @pysnooper.snoop()
    def __init__(self):
        # Initialize sensors and actuators
        self.safety_stop = SafetyStop()
        self.encoder = Encoder()
        self.hall_sensor = HallSensor()
        self.imu = IMU()
        self.steering_motor = SteeringMotor()

        # Run bike and controllers
        self.controller = Controller(self)
        # self.controller.startup()
        self.controller.run()

    # Safety Stop
    def emergency_stop_check(self):
        return self.safety_stop.button_check()

    # Steering Encoder
    def get_handlebar_angle(self):
        return self.encoder.get_angle()

    # IMU
    def get_imu_data(self, velocity, delta_state, phi):
        return self.imu.get_imu_data(velocity, delta_state, phi)

    # Steering Motor
    def set_handlebar_angular_velocity(self, angular_velocity):
        self.steering_motor.set_angular_velocity(angular_velocity)
        self.handlebar_angular_velocity = angular_velocity

    def get_handlebar_angular_velocity(self):
        return self.handlebar_angular_velocity

    # Stop bike
    def stop(self):
        self.steering_motor.stop()

    def stop_all(self):
        self.stop()
        GPIO.cleanup()