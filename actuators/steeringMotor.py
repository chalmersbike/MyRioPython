from param import *
import serial, time
import Adafruit_BBIO.PWM as PWM
import time
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import numpy as np


class SteeringMotor(object):
    def __init__(self):
        ADC.setup()
        GPIO.setup(steeringMotor_PinEnable, GPIO.OUT)
        GPIO.output(steeringMotor_PinEnable, GPIO.LOW)
        if debug:
            print('Steering Motor : Enable pin set to DISABLE')
        PWM.start(steeringMotor_Channel, steeringMotor_IdleDuty, steeringMotor_Frequency)
        PWM.set_duty_cycle(steeringMotor_Channel, steeringMotor_IdleDuty)
        if debug:
            print('Steering Motor : PWM started')

    def set_PWM_duty_cycle(self,duty_cycle):
        # Constrain duty cycle between steeringMotor_PWMforMinOutput and steeringMotor_PWMforMaxOutput
        if duty_cycle > steeringMotor_PWMforMaxOutput:
            duty_cycle = steeringMotor_PWMforMaxOutput
            print("Steering Ang.Vel saturated +")
        elif duty_cycle < steeringMotor_PWMforMinOutput:
            duty_cycle = steeringMotor_PWMforMinOutput
            print("Steering Ang.Vel saturated -")
        # Set PWM duty cycle
        PWM.set_duty_cycle(steeringMotor_Channel, duty_cycle)

    # @pysnooper.snoop()
    def set_angular_velocity(self, angular_velocity):
        rpm_conversion = -angular_velocity * rads2rpm * steeringMotor_GearRatio  # To comply with RighHandRule
        # duty_cycle = steeringMotor_IdleDuty + rpm_conversion * 40.0 / 1000.0
        # rpm_conversion = 0
        duty_cycle = np.interp(rpm_conversion, [steeringMotor_SpeedMinOutput, steeringMotor_SpeedMaxOutput],
                               [steeringMotor_PWMforMinOutput, steeringMotor_PWMforMaxOutput])
        self.set_PWM_duty_cycle(duty_cycle)

    def set_current(self, current):
        # duty_cycle = steeringMotor_IdleDuty + current * 40.0 / 1.0 # steeringMotor_PWMforMinOutput% = -1A ; steeringMotor_PWMforMaxOutput% = 1A
        duty_cycle = np.interp(rpm_conversion, [steeringMotor_CurrentMinOutput, steeringMotor_CurrentMaxOutput],
                               [steeringMotor_PWMforMinOutput, steeringMotor_PWMforMaxOutput])
        self.set_PWM_duty_cycle(duty_cycle)

    def stop(self):
        GPIO.output(steeringMotor_PinEnable, GPIO.LOW)
        GPIO.cleanup()
        PWM.stop(steeringMotor_Channel)
        PWM.cleanup()

    def enable(self):
        GPIO.output(steeringMotor_PinEnable, GPIO.HIGH)

    def disable(self):
        GPIO.output(steeringMotor_PinEnable, GPIO.LOW)

    def read_steer_current(self):
        return ADC.read(steering_current_port) * steering_max_current