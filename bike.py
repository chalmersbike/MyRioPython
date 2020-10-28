import Adafruit_BBIO.GPIO as GPIO
from param import *
from sensors import Encoder, HallSensor, IMU, SafetyStop, Potentiometer, DualLaserRanger, GPS
from actuators import DriveMotor, SteeringMotor
from controller import Controller
import pysnooper

class Bike(object):
    # @pysnooper.snoop()
    def __init__(self, debug=True, recordPath=False):
        # Initialize sensors and actuators
        self.safety_stop = SafetyStop()
        self.encoder = Encoder()
        self.hall_sensor = HallSensor()
        self.imu = IMU()
        if potentiometer_use:
            self.potent = Potentiometer()
        if gps_use:
            self.gps = GPS()
        if laserRanger_use:
            self.laser_ranger = DualLaserRanger()
        self.drive_motor = DriveMotor()
        self.steering_motor = SteeringMotor()

        # Run bike and controllers
        self.controller = Controller(self,recordPath)
        # self.controller.startup()
        self.controller.run()

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

    # Laser Ranger
    if laserRanger_use:
        def get_laserRanger_data(self):
            return self.laser_ranger.get_y()

    # Potentiometer
    if potentiometer_use:
        def get_potentiometer_value(self):
            return self.potent.read_pot_value()

    # Drive Motor
    def set_velocity(self, input_velocity):
        self.drive_motor.set_velocity(input_velocity)

    # Steering Motor
    def set_handlebar_angular_velocity(self, angular_velocity):
        self.steering_motor.set_angular_velocity(angular_velocity)
        self.handlebar_angular_velocity = angular_velocity

    def get_handlebar_angular_velocity(self):
        return self.handlebar_angular_velocity

    # Stop bike
    def stop(self):
        self.steering_motor.stop()
        self.drive_motor.stop()

    def stop_all(self):
        self.stop()
        GPIO.cleanup()