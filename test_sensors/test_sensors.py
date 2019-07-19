sys.path.append('../')

print 'Testing the sensors'

from sensors import Encoder, HallSensor, IMU, SafetyStop, GPS
import Adafruit_BBIO.GPIO as GPIO

from actuators import RearMotorDrive, SteeringMotor
from controller import Controller
from time import sleep
import time


# Data logging setup
timestr = time.strftime("%Y%m%d-%H%M%S")


# Test Safety stop
self.safety_stop = SafetyStop()
input1 = raw_input('Press the EStop and Enter, test the emergency stop is detectable!')
ESign = self.safety_stop.button_check()
while not ESign:
    print 'The EStop was not pressed!'
    time.sleep(0.5)
    ESign = self.safety_stop.button_check()
if ESign:
    print 'Estop detected, the testing of the sensors (Hall sensor, encoder, IMU, GPS) will now begin.'


# Test Hall sensor
# Setup CSV file
results_hallsensor = open('SensorTest_HallSensor-%s.csv' % timestr, 'wb')
self.writer_hallsensor = csv.writer(results_hallsensor)
self.writer_hallsensor.writerow('Time (s)', 'Speed (m/s)')

number_samples_HallSensor = raw_input('Input the number of samples of press ENTER for 20 samples for the Hall sensor test, rotate the rear wheel for the reading! ')
self.hall_sensor = HallSensor()
start_time = time.time()

if not number_samples_HallSensor:
    number_samples_HallSensor = 20
for x in range(1, int(number_samples_HallSensor) + 1):
    self.hall_sensor_velocity = self.hall_sensor.get_velocity()
    print 'Time=%f\t Vel = %f\n' % (time.time() - start_time, self.hall_sensor_velocity)

    # Write to CSV file
    self.writer_hallsensor.writerow((time.time() - start_time, self.hall_sensor_velocity))

    time.sleep(0.5)


# Test Encoder
# Setup CSV file
results_encoder = open('SensorTest_Encoder-%s.csv' % timestr, 'wb')
self.writer_encoder = csv.writer(results_encoder)
self.writer_encoder.writerow('Time (s)', 'Steering angle (deg)')

number_samples_Encoder = raw_input('Input the number of samples of press ENTER for 20 samples for the encoder test, rotate the handle bar for the reading! ')
self.encoder = Encoder()
start_time = time.time()
print 'ENCODER FREQNECY = %f \n' % (self.encoder.encoder.frequency)
if not number_samples_Encoder:
    number_samples_Encoder = 20
for x in range(1, int(number_samples_Encoder) + 1):
    self.encoder_angle = 57.29577 * self.encoder.get_angle()
    print 'Time=%f\t delta = %f\n' % (time.time() - start_time, self.encoder_angle)

    # Write to CSV file
    self.writer_encoder.writerow((time.time() - start_time, self.encoder_angle))

    time.sleep(0.5)


# Test IMU
# Setup CSV file
results_imu = open('SensorTest_IMU-%s.csv' % timestr, 'wb')
self.writer_imu = csv.writer(results_imu)
self.writer_imu.writerow('Time (s)', 'ax (mg)','ay (mg)', 'az (mg)', 'gx (deg/s)', 'gy (deg/s)', 'gz (deg/s)')

number_samples_IMU = raw_input('Input the number of samples of press ENTER for 20 samples for the IMU test, move the bike body for the reading! ')
self.a_imu = IMU()
start_time = time.time()
if not number_samples_IMU:
    number_samples_IMU = 20
for x in range(1, int(number_samples_IMU) + 1):
    self.imudata = self.a_imu.get_imu_data()
    print 'Time = %g\tTemp = %g\tAx = %g\tAy = %g\tAz = %g\tGx = %g\tGy = %g\tGz = %g\t' % (
    time.time() - start_time, self.imudata[0], self.imudata[1], self.imudata[2], self.imudata[3], self.imudata[4],
    self.imudata[5], self.imudata[6])

    # Write to CSV file
    self.writer_imu.writerow((time.time() - start_time, self.imudata[0], self.imudata[1], self.imudata[2], self.imudata[3], self.imudata[4],self.imudata[5], self.imudata[6]))
    
    time.sleep(0.04)


# Test GPS
# Setup CSV file
results_gps = open('SensorTest_GPS-%s.csv' % timestr, 'wb')
self.writer_gps = csv.writer(results_gps)
self.writer_gps.writerow('Time (s)', 'x (m)', 'y (m)')

number_samples_GPS = raw_input('Input the number of samples of press ENTER for 20 samples for the GPS test, move the GPS for the reading! ')
gps = GPS()
start_time = time.time()
if not number_samples_GPS:
    number_samples_GPS = 20
for x in range(1, int(number_samples_GPS) + 1):
    self.gpspos = gps.get_position()
    print 'Time=%f\tx=%g\ty = %g\t' % (time.time() - start_time, self.gpspos[0], self.gpspos[1])

    # Write to CSV file
    self.writer_gps.writerow((time.time() - start_time, self.gpspos[0], self.gpspos[1]))

    time.sleep(1)


print 'Tests are done, writing to a CSV file ...'


# Write to CSV
# Hall sensor
self.writer_encoder.writerow(
    (self.time_count_gaining_speed, calculation_time, pid_velocity_reference, self.velocity, self.states[0],
     self.states[1],
     self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
     self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
     self.extra_data[4], self.extra_data[5], self.extra_data[6], current_reading, rpm_reading, self.extra_data[7]))

# Encoder
# IMU
self.writer_encoder.writerow(
    (self.time_count_gaining_speed, calculation_time, pid_velocity_reference, self.velocity, self.states[0],
     self.states[1],
     self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
     self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
     self.extra_data[4], self.extra_data[5], self.extra_data[6], current_reading, rpm_reading, self.extra_data[7]))

# GPS
self.writer_encoder.writerow(
    (self.time_count_gaining_speed, calculation_time, pid_velocity_reference, self.velocity, self.states[0],
     self.states[1],
     self.states[2], self.lqr_balance_control_signal, self.pid_velocity_control_signal,
     self.extra_data[0], self.extra_data[1], self.extra_data[2], self.extra_data[3],
     self.extra_data[4], self.extra_data[5], self.extra_data[6], current_reading, rpm_reading, self.extra_data[7]))


print 'Writing done, the process will now be killed.'