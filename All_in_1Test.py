#!/usr/bin/env python
from sensors import Encoder, HallSensor, IMU, SafetyStop, GPS
import Adafruit_BBIO.GPIO as GPIO

from actuators import RearMotorDrive, SteeringMotor
from controller import Controller
from time import sleep
import time

# INITIAL_SPEED = 3.9

class Test(object):
    def __init__(self):
        # Safety stop
        self.safety_stop = SafetyStop()
        input1 = raw_input('Press the EStop and Enter, test the emergency stop is detectable!')
        ESign = self.safety_stop.button_check()
        while not ESign:
            print 'The EStop was not pressed!'
            time.sleep(0.5)
            ESign = self.safety_stop.button_check()
        if ESign:
            print 'Estop detected, please reset it if you want to test the motors'

        # Hall sensor
        number_samples_HallSensor = raw_input('Input the number of samples of press ENTER for 20 samples for the Hall sensor test, rotate the rear wheel for the reading! ')
        self.hall_sensor = HallSensor()
        start_time = time.time()
        if not number_samples_HallSensor:
            number_samples_HallSensor = 20

        for x in range(1,int(number_samples_HallSensor)+1):
            print 'Time=%f\t Vel = %f\n' % (time.time()-start_time,self.hall_sensor.get_velocity())
            time.sleep(0.5)

        # Encoder
        number_samples_Encoder = raw_input('Input the number of samples of press ENTER for 20 samples for the encoder test, rotate the handle bar for the reading! ')
        self.encoder = Encoder()
        start_time = time.time()
        print 'ENCODER FREQNECY = %f \n' % (self.encoder.encoder.frequency)
        if not number_samples_Encoder:
            number_samples_Encoder = 20
        for x in range(1,int(number_samples_Encoder)+1):
            print 'Time=%f\t delta = %f\n' % (time.time() - start_time, 57.29577 * self.encoder.get_angle())
            time.sleep(0.5)

        # IMU
        number_samples_IMU = raw_input('Input the number of samples of press ENTER for 20 samples for the IMU test, move the bike body for the reading! ')
        self.a_imu = IMU()
        start_time = time.time()
        if not number_samples_IMU:
            number_samples_IMU = 20
        for x in range(1,int(number_samples_IMU)+1):
            self.imudata = self.a_imu.get_imu_data()
            print 'Time = %g\tTemp = %g\tAx = %g\tAy = %g\tAz = %g\tGx = %g\tGy = %g\tGz = %g\t' % (time.time() - start_time, self.imudata[0], self.imudata[1], self.imudata[2], self.imudata[3],self.imudata[4], self.imudata[5], self.imudata[6])
            time.sleep(0.5)

        # GPS
        number_samples_GPS = raw_input('Input the number of samples of press ENTER for 20 samples for the GPS test, move the GPS for the reading! ')
        gps = GPS()
        start_time = time.time()
        if not number_samples_GPS:
            number_samples_GPS = 20
        for x in range(1,int(number_samples_GPS)+1):
            self.gpspos = gps.get_position()
            print 'Time=%f\t' % (time.time() - start_time)
            print 'Temp=%g\tAx = %g\t' % (self.gpspos[0], self.gpspos[1])
            time.sleep(1)

        # Drive motor
        input1 = raw_input('If all the tests are passed, then you may check the rear motor and steering motor. HOWEVER THEY ARE RISKY! PRESS ENTER TO CONTINUE')
        if not input1:
            # print 'Now we are going to test the rear motor, the gear is at LEVEL %i \t' %  RearMotorDrive().GearNr
            input1 = raw_input('Input a velocity between 1 and 5 m/s, the test will last 30 secs ')
            self.rear_motor = RearMotorDrive()
            if not input1:
                print 'Press enter to exit'
                exit()
            elif float(input1) >= 1 and  float(input1) <= 5:
                input_velocity = float(input1)
                raw_input('You need to be careful on the velocity reading, if they deviate too much, maybe there is problem in gear setting, see rearmotordrive.py!')
                start_time = time.time()
                self.rear_motor.set_velocity(input_velocity)
                time_now = start_time
                while time_now  - start_time < 30:
                    time_now = time.time()
                    self.imudata = self.a_imu.get_imu_data()
                    print 'Time=%f\tVel = %f\tphi = %f\tdelta = %f\t' % (
                    time_now - start_time, self.hall_sensor.get_velocity(), 57.29577 *self.imudata[0], 57.29577 *  self.encoder.get_angle())
                    # Without IMU Reading
                    # print 'Time=%f\tVel = %f\tdelta = %f\tCurrent = %f\tRPM = %f\t' % (
                    #     time_now - start_time, self.hall_sensor.get_velocity(),
                    #     57.29577 * self.encoder.get_angle(), self.rear_motor.read_motor_current(),
                    #     self.rear_motor.read_motor_rpm())

                    if self.safety_stop.button_check():
                        self.rear_motor.stop()
                        print 'Emergency Enabled'
                        exit()
                        break
                    time.sleep(0.1)
                self.rear_motor.stop()

                print 'Now all tests has been done, except the steering motor, please do it carefully by running the easy_start and disconnect the rear motor'
            else:
                print 'Test Finished'
        else:
            print 'It is done, the process will be killed, GOOD LUCK!'
