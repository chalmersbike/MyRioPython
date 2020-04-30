from actuators import Steering
import time

class Test(object):
    def __init__(self):
        self.steering_motor_drive = Steering()
        self.steering_motor_drive.enable()
        angular_velocity = 0.05
        start_time = time.time()
        while (time.time()-start_time)<5:
            self.steering_motor_drive.set_angular_velocity(angular_velocity)  # will cause the motor to rotate the wheel to the right
        self.steering_motor_drive.set_angular_velocity(0)
        self.steering_motor_drive.stop()

test = Test()