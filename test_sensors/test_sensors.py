import sys
sys.path.append('../')

print 'Testing the sensors'

from sensors import Encoder, HallSensor, IMU, SafetyStop, GPS
import Adafruit_BBIO.GPIO as GPIO

from actuators import RearMotorDrive, SteeringMotor
from time import sleep
import time
import csv


# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")


# Test Safety stop
safety_stop = SafetyStop()
input1 = raw_input('Press the EStop and Enter, test the emergency stop is detectable!')
ESign = safety_stop.button_check()
while not ESign:
    print 'The EStop was not pressed!'
    time.sleep(0.5)
    ESign = safety_stop.button_check()
if ESign:
    print 'Estop detected, the testing of the sensors (Hall sensor, encoder, IMU, GPS) will now begin.'


# Test Hall sensor
# Setup CSV file
results_hallsensor = open('%s-SensorTest_HallSensor.csv' % timestr, 'wb')
writer_hallsensor = csv.writer(results_hallsensor)
writer_hallsensor.writerow(('Time (s)', 'Speed (m/s)'))

number_samples_HallSensor = raw_input('Input the number of samples of press ENTER for 20 samples for the Hall sensor test, rotate the rear wheel for the reading! ')
hall_sensor = HallSensor()
start_time = time.time()

if not number_samples_HallSensor:
    number_samples_HallSensor = 20
for x in range(1, int(number_samples_HallSensor) + 1):
    hall_sensor_velocity = hall_sensor.get_velocity()
    print 'Time=%f\t Vel = %f' % (time.time() - start_time, hall_sensor_velocity)

    # Write to CSV file
    writer_hallsensor.writerow((time.time() - start_time, hall_sensor_velocity))

    time.sleep(0.04)


# Test Encoder
# Setup CSV file
results_encoder = open('%s-SensorTest_Encoder.csv' % timestr, 'wb')
writer_encoder = csv.writer(results_encoder)
writer_encoder.writerow(('Time (s)', 'Steering angle (deg)'))

number_samples_Encoder = raw_input('Input the number of samples of press ENTER for 20 samples for the encoder test, rotate the handle bar for the reading! ')
encoder = Encoder()
start_time = time.time()
print 'ENCODER FREQNECY = %f\n' % (encoder.encoder.frequency)
if not number_samples_Encoder:
    number_samples_Encoder = 20
for x in range(1, int(number_samples_Encoder) + 1):
    encoder_angle = 57.29577 * encoder.get_angle()
    print 'Time=%f\t delta = %f' % (time.time() - start_time, encoder_angle)

    # Write to CSV file
    writer_encoder.writerow((time.time() - start_time, encoder_angle))

    time.sleep(0.04)


# Test IMU
# Setup CSV file
results_imu = open('%s-SensorTest_IMU.csv' % timestr, 'wb')
writer_imu = csv.writer(results_imu)
writer_imu.writerow(('Time (s)', 'ax (mg)','ay (mg)', 'az (mg)', 'gx (deg/s)', 'gy (deg/s)', 'gz (deg/s)'))

number_samples_IMU = raw_input('Input the number of samples of press ENTER for 20 samples for the IMU test, move the bike body for the reading! ')
a_imu = IMU()
start_time = time.time()
if not number_samples_IMU:
    number_samples_IMU = 20
for x in range(1, int(number_samples_IMU) + 1):
    imudata = a_imu.get_imu_data()
    print 'Time = %g\tTemp = %g\tAx = %g\tAy = %g\tAz = %g\tGx = %g\tGy = %g\tGz = %g' % (
    time.time() - start_time, imudata[0], imudata[1], imudata[2], imudata[3], imudata[4],
    imudata[5], imudata[6])

    # Write to CSV file
    writer_imu.writerow((time.time() - start_time, imudata[0], imudata[1], imudata[2], imudata[3], imudata[4],imudata[5], imudata[6]))
    
    time.sleep(0.04)


# Test GPS
# Setup CSV file
results_gps = open('%s-SensorTest_GPS.csv' % timestr, 'wb')
writer_gps = csv.writer(results_gps)
writer_gps.writerow(('Time (s)', 'x (m)', 'y (m)'))

number_samples_GPS = raw_input('Input the number of samples of press ENTER for 20 samples for the GPS test, move the GPS for the reading! ')
gps = GPS()
start_time = time.time()
if not number_samples_GPS:
    number_samples_GPS = 20
for x in range(1, int(number_samples_GPS) + 1):
    gpspos = gps.get_position()
    print 'Time=%f\tx=%g\ty = %g' % (time.time() - start_time, gpspos[0], gpspos[1])

    # Write to CSV file
    writer_gps.writerow((time.time() - start_time, gpspos[0], gpspos[1]))

    time.sleep(1)


print 'Tests are done, the process will now be killed.'