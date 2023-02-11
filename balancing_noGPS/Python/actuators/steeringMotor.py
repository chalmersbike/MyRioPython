from param import *
import serial, time
import time
import numpy as np

# print('FPGA_DUTYCYCLE overwritten for DEBUGGING!!!!')

class SteeringMotor(object):
    def __init__(self,session):
        # print('FPGA_DUTYCYCLE overwritten for DEBUGGING!!!!')
        self.fpga_SteeringEnable = session.registers['Steering Enable']
        self.fpga_SteeringEnable.write(False)
        if debug:
            print('Steering Motor : Enable pin set to DISABLE')

        self.fpga_SteeringPWM = session.registers['Steering PWM Duty Cycle']
        self.fpga_SteeringFrequency = session.registers['Steering PWM Frequency']
        self.fpga_SteeringFrequency.write(steeringMotor_Frequency)
        self.fpga_SteeringPWM.write(steeringMotor_IdleDuty)
        if debug:
            print('Steering Motor : PWM started')

        self.fpga_SteeringCurrent = session.registers['Steering Current']

    def set_PWM_duty_cycle(self,duty_cycle):
        # Constrain duty cycle between steeringMotor_PWMforMinOutput and steeringMotor_PWMforMaxOutput
        if duty_cycle > steeringMotor_PWMforMaxOutput:
            duty_cycle = steeringMotor_PWMforMaxOutput
            print("Steering Ang.Vel saturated +")
        elif duty_cycle < steeringMotor_PWMforMinOutput:
            duty_cycle = steeringMotor_PWMforMinOutput
            print("Steering Ang.Vel saturated -")
        # Set PWM duty cycle
        # self.fpga_SteeringPWM.write(72) # DEBUG ONLY!
        # print('Ask for %.3f, but give 72 dutycycle!' %(duty_cycle))
        self.fpga_SteeringPWM.write(duty_cycle)
        # print('Duty Cycle is %f' %(duty_cycle))

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
        self.fpga_SteeringEnable.write(False)
        self.fpga_SteeringPWM.write(steeringMotor_IdleDuty)

    def enable(self):
        self.fpga_SteeringEnable.write(True)
        print('STR MTR ENABLED!')

    def disable(self):
        self.fpga_SteeringEnable.write(False)
        # print('STR MTR DISABLED!')

    def read_steer_current(self):
        return (self.fpga_SteeringCurrent.read()*1.221e-3-2)*2
