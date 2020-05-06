#!/usr/bin/env python
from param import *
from sensors import Encoder, HallSensor, SafetyStop
import Adafruit_BBIO.GPIO as GPIO
from sensors import IMU
from actuators import DriveMotor, SteeringMotor
from time import sleep
import time
# INITIAL_SPEED = 3.9
class Test(object):
    def __init__(self):
        self.safety_stop = SafetyStop()
        input1 = raw_input('Press the EStop and Enter, test the emergency stop is detectable!')
        ESign = self.safety_stop.button_check()
        while not ESign:

            print('The EStop was not pressed!')
            time.sleep(0.5)
            ESign = self.safety_stop.button_check()
        if ESign:
            print('Estop detected, please reset it if you want to test the motors')

        input1 = raw_input('Press ENTER or #sample to start hallsensor test,rotate the rear wheel for the reading!')

        self.hall_sensor = HallSensor()
        start_time = time.time()
        if not input1:
            for x in range(1,20):
                print('Time=%f\t Vel = %f\n' % (time.time()-start_time,self.hall_sensor.get_velocity()))
                time.sleep(0.5)
        else:
            for x in range(1,int(input1)):

                print('Time=%f\t Vel = %f\n' % (time.time()-start_time, self.hall_sensor.get_velocity()))
                time.sleep(0.5)


        input1 = raw_input('Press ENTER to start Encoder test,rotate the handle bar for the reading!')
        self.encoder = Encoder()
        start_time = time.time()
        print('ENCODER FREQNECY = %f \n' % (self.encoder.encoder.frequency))
        if not input1:
            for x in range(1,20):
                print('Time=%f\t delta = %f\n' % (time.time() - start_time, 57.29577*self.encoder.get_angle()))

                time.sleep(0.5)
        else:
            for x in range(1,int(input1)):
                # print self.encoder.get_angle()
                print('Time=%f\t delta = %f\n' % (time.time() - start_time, 57.29577 * self.encoder.get_angle()))
                time.sleep(0.5)

        input1 = raw_input('Press ENTER to start IMU test, move the bike body for the reading!')
        self.imu = IMU(horizontal = False)
        # self.imu = IMU(horizontal = True)
        start_time = time.time()
        if not input1:
            for x in range(1,20):
                self.imudata = self.imu.get_imu_data()
                print('Time=%f\t'% (time.time() - start_time))
                print(self.imudata)
                # print(self.imudata)
                time.sleep(0.5)
        else:
            for x in range(1,int(input1)):
                # self.bike.get_imu_data()
                self.imudata = self.imu.get_imu_data()
                print('Time=%g\t'% (time.time() - start_time))
                # print(self.imudata)
                print('Phi_CompFilter_RollAccComp=%g\tPhi_CompFilter = %g\tGyroX = %g\tAccX = %g\tAyRollComp = %g\tAy = %g\tAz = %g\tGyroIntg = %g\t' % (
                    self.imudata[0], self.imudata[1], self.imudata[2], self.imudata[3],
                    self.imudata[4], self.imudata[5], self.imudata[6], self.imudata[7]))

                time.sleep(0.5)

        input1 = raw_input('If all the tests are passed, then you may check the rear motor and steering motor. HOWEVER THEY ARE RISKY! PRESS ENTER TO CONTINUE')
        if not input1:
            # print 'Now we are going to test the rear motor, the gear is at LEVEL %i \t' %  RearMotorDrive().GearNr
            input1 = raw_input('enter a velocity between 1-3.9, the test will last 30 secs')
            self.rear_motor = RearMotorDrive()
            if not input1:
                print('Press enter to exit')
                exit()
            elif float(input1) >1 and  float(input1) < 10:
                input_velocity = float(input1)
                raw_input('You need to be careful on the velocity reading, if they deviate too much, maybe there is problem in gear setting, see rearmotor.py!')
                start_time = time.time()
                self.rear_motor.set_velocity(input_velocity)
                time_now = start_time
                while time_now  - start_time < 30:
                    time_now = time.time()
                    self.imudata = self.imu.get_imu_data()
                    print('Time=%f\tVel = %f\tphi = %f\tdelta = %f\t' % (
                    time_now - start_time, self.hall_sensor.get_velocity(), 57.29577 *self.imudata[0], 57.29577 *  self.encoder.get_angle()))
                    # Without IMU Reading
                    # print 'Time=%f\tVel = %f\tdelta = %f\tCurrent = %f\tRPM = %f\t' % (
                    #     time_now - start_time, self.hall_sensor.get_velocity(),
                    #     57.29577 * self.encoder.get_angle(), self.rear_motor.read_motor_current(),
                    #     self.rear_motor.read_motor_rpm())

                    if self.safety_stop.button_check():
                        self.rear_motor.stop()
                        print('Emergency Enabled')
                        exit()
                        break
                    time.sleep(0.1)
                self.rear_motor.stop()

                print('Now all tests has been done, except the steering motor, please do it carefully by running the easy_start and disconnect the rear motor')
            else:
                print('Test Finished')
        else:
            print('It is done, the process will be killed, GOOD LUCK!')
