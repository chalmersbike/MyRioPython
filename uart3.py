import time

from balancing_noGPS.Python.actuators.driveMotor import DriveMotor

session = 0
motor = DriveMotor(session)

while True :
    motor.set_velocity(3)
    time.sleep(5)
    motor.stop()
