#!/usr/bin/env python
from param import *
from sensors import Encoder, HallSensor, SafetyStop
from sensors import IMU
from actuators import DriveMotor, SteeringMotor
from time import sleep
import time
from nifpga import Session
from numpy import rad2deg
# INITIAL_SPEED = 3.9
class Test(object):
    def __init__(self):
        # with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_ip6b6zVtH2s.lvbitx","RIO0") as session:  # balancing Control New
        with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_4.lvbitx","RIO0") as session:  # balancing Control V5
            self.safety_stop = SafetyStop(session)
            input1 = input('Press the EStop and Enter, test the emergency stop is detectable!')
            ESign = self.safety_stop.button_check()
            # while not ESign:
            if not ESign:
                print('The EStop was not pressed!')
                # time.sleep(0.5)
                # ESign = self.safety_stop.button_check()
            if ESign:
                print('Estop detected, please reset it if you want to test the motors')

            input1 = input('Press ENTER or #sample to start hallsensor test,rotate the rear wheel for the reading!')

            self.hall_sensor = HallSensor(session)
            start_time = time.time()
            if not input1:
                for x in range(1,20):
                    print('Time=%f\t Vel = %f\n' % (time.time()-start_time,self.hall_sensor.get_velocity(4)))
                    time.sleep(0.5)
            else:
                for x in range(1,int(input1)):

                    print('Time=%f\t Vel = %f\n' % (time.time()-start_time, self.hall_sensor.get_velocity(4)))
                    time.sleep(0.5)


            input1 = input('Press ENTER to start Encoder test,rotate the handle bar for the reading!')
            self.encoder = Encoder(session)
            start_time = time.time()
            # print('ENCODER FREQNECY = %f \n' % (self.encoder.frequency))
            if not input1:
                for x in range(1,20):
                    print('Time=%f\t delta = %f\n' % (time.time() - start_time, 57.29577*self.encoder.get_angle()))

                    time.sleep(0.5)
            else:
                for x in range(1,int(input1)):
                    # print self.encoder.get_angle()
                    print('Time=%f\t delta = %f\n' % (time.time() - start_time, 57.29577 * self.encoder.get_angle()))
                    time.sleep(0.5)

            input1 = input('Press ENTER to start IMU test, move the bike body for the reading!')
            self.imu = IMU(session, horizontal = False)
            # self.imu = IMU(session, horizontal = True)
            # self.imu = IMU(session, horizontal = -8, pitch_horizontal = -7)

            start_time = time.time()
            phi = 0.0
            if not input1:
                for x in range(1,20):
                    self.imudata = self.imu.get_imu_data(0,0,0)
                    print('Time=%f\t'% (time.time() - start_time))
                    print(self.imudata)
                    # print(self.imudata)
                    phi = self.imudata[0]
                    time.sleep(0.5)
            else:
                for x in range(1,int(input1)):
                    print('Time=%g\t'% (time.time() - start_time))

                    self.imudata = self.imu.get_imu_data(0,0,phi)
                    print(self.imudata[0])
                    print('Phi_CompFilter=%g deg\tPhi_gyro_int = %g deg\tGyroX = %g\tGyroY = %g\tGyroZ = %g\tAx = %g\tAy = %g\tAz = %g\t' % (
                        rad2deg(self.imudata[0]), rad2deg(self.imudata[1]), self.imudata[2], self.imudata[3],
                        self.imudata[4], self.imudata[5], self.imudata[6], self.imudata[7]))
                    phi = self.imudata[0]
                    # self.imudata = self.imu.get_reading()
                    # print('Time=%g \t Ax = %g\tAy = %g\tAz = %g\t GyroX = %g\tGyroY = %g\tGyroZ = %g\t' % (
                    #         (self.imudata[0]), (self.imudata[1]), self.imudata[2], self.imudata[3],
                    #         self.imudata[4], self.imudata[5], self.imudata[6]))

                    time.sleep(0.5)



            input1 = input('If all the tests are passed, then you may check the rear motor and steering motor. HOWEVER THEY ARE RISKY! PRESS ENTER TO CONTINUE')
            if not input1:
                # print 'Now we are going to test the rear motor, the gear is at LEVEL %i \t' %  RearMotorDrive().GearNr
                input1 = input('enter a velocity between 1-3.9, the test will last 30 secs')
                self.rear_motor = DriveMotor()
                if not input1:
                    print('Press enter to exit')
                    exit()
                elif float(input1) >1 and  float(input1) < 10:
                    input_velocity = float(input1)
                    input('You need to be careful on the velocity reading, if they deviate too much, maybe there is problem in gear setting, see rearmotor.py!')
                    start_time = time.time()
                    self.rear_motor.set_velocity(input_velocity)
                    time_now = start_time
                    while time_now  - start_time < 30:
                        time_now = time.time()
                        # self.imudata = self.imu.get_imu_data()
                        # print('Time=%f\tVel = %f\tphi = %f\tdelta = %f\t' % (
                        # time_now - start_time, self.hall_sensor.get_velocity(), 57.29577 *self.imudata[0], 57.29577 *  self.encoder.get_angle()))
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
