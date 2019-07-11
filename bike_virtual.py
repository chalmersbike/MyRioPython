import Adafruit_BBIO.GPIO as GPIO

from actuators import RearMotor, SteeringMotor
from controller import Controller
from sensors import Encoder, HallSensor, A_IMU, SafetyStop

CHEAT_VELOCITY = 1
CHEAT_IMU = 1
CHEAT_STEERING_ANG = 1


class Bike_Virtual(object):

    def __init__(self, debug='True'):
        self.virtual_speed = 3
        self.virtual_IMU = [0.1, 0.1, 0.1, 0.001, 0.001, 9.9]
        self.virtual_steering_ang = 0.02
        self.safety_stop = SafetyStop()
        self.encoder = Encoder()
        self.hall_sensor = HallSensor()
        self.rear_motor = RearMotor()
        self.steering_motor = SteeringMotor()
        self.a_imu = A_IMU()
        if not debug:
            self.controller = Controller(self)
            self.controller.start()

    def set_velocity(self, input_velocity):
        self.rear_motor.set_velocity(input_velocity)

    def stop(self):
        self.rear_motor.stop()

    def get_handlebar_angle(self):
        if CHEAT_STEERING_ANG:
            return self.virtual_steering_ang
        else:
            return self.encoder.get_angle()

    def get_handlebar_angular_velocity(self):
        return self.handlebar_angular_velocity

    def set_handlebar_angular_velocity(self, angular_velocity):
        self.steering_motor.set_angular_velocity(angular_velocity)
        self.handlebar_angular_velocity = angular_velocity

    def get_velocity(self):
        if CHEAT_VELOCITY:
            return self.virtual_speed
        else:
            return self.hall_sensor.get_velocity()

    def get_imu_data(self):
        if CHEAT_IMU:
            return self.virtual_IMU
        else:
            return self.a_imu.get_imu_data()

    def emergency_stop_check(self):
        return self.safety_stop.button_check()

    def stop_all(self):
        self.stop()
        self.set_handlebar_angular_velocity(0)
        GPIO.cleanup()
