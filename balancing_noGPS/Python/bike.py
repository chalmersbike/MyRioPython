import time
from param import *
from sensors import Encoder, HallSensor, IMU, SafetyStop, Klm_Estimator #, Potentiometer, DualLaserRanger
from actuators import SteeringMotor
from controller import Controller
from vesc_gps_resource_v2 import VESC_GPS
import pysnooper
from nifpga import Session
from GainScheduling import GainScheduling as GS

class Bike(object):
    # @pysnooper.snoop()
    def __init__(self, debug=True, recordPath=False, reverse=False, straight=False, path_file_arg='', rollref_file_arg='', steeringdist_file_arg='', simulate_file=''):

        # with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_4.lvbitx","RIO0") as session:  # balancing Control V5
        #     with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_20220510.lvbitx",
            with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingV6_5.lvbitx",
                         "RIO0") as session:  # balancing Control V5
                # Initialize sensors and actuators
                if simulate_file == '':
                    self.safety_stop = SafetyStop(session)
                    self.encoder = Encoder(session)
                    self.hall_sensor = HallSensor(session)
                    self.imu = IMU(session, horizontal=False, Kalman = False)
                    if potentiometer_use:
                        self.potent = Potentiometer(session)
                    # if gps_use:
                    #     self.gps = GPS(session)
                    if laserRanger_use:
                        self.laser_ranger = DualLaserRanger(session)

                    # self.drive_motor = DriveMotor(session)
                    self.drive_gps_joint = VESC_GPS(session)
                    self.steering_motor = SteeringMotor(session)
                    if gps_use:
                        # ini_gps_output = self.drive_gps_joint.get_latlon()
                        self.lat_0 = 0.0
                        self.lon_0 = 0.0
                        print('Waiting for valid GPS data for State Estimator')
                        while self.lat_0 == 0.0 and self.lon_0 == 0.0:
                            ini_gps_output = self.drive_gps_joint.get_position()
                            self.lat_0 = ini_gps_output[2]
                            self.lon_0 = ini_gps_output[3]
                            print([self.lat_0, self.lon_0])
                        self.Klm_Estimator = Klm_Estimator(self.lat_0, self.lon_0)
                    if dynamicalGainScheduling:
                        # self.GainSche = GS(PiPoly = [ -0.3250, 3.5636, -14.2051, 23.0150],
                        #                    IiPoly = [0.0, 0.0, 0.0, 0.0],
                        #                    PoPoly = [-0.0113, 0.1233, -0.4798, 1.6257])
                        self.GainSche = GS(PiPoly = [ -0.3250, 3.5636, -14.2051, 23.0150],
                                           IiPoly = [0.0, 0.0, 0.0, 0.0],
                                           PoPoly = [-0.0133, 0.1453, -0.5710, 2.0043])
                        

                # Run bike and controllers
                self.controller = Controller(self,recordPath,reverse,straight,path_file_arg,rollref_file_arg,steeringdist_file_arg,simulate_file)
                # self.controller.startup()
                self.controller.run()

    # if simulate_file == '':
    # Safety Stop
    def emergency_stop_check(self):
        return self.safety_stop.button_check()

    # Steering Encoder
    def get_handlebar_angle(self):
        return self.encoder.get_angle()

    # Hall Sensor
    # @pysnooper.snoop()
    def get_velocity(self, ref_velocity):
        return self.hall_sensor.get_velocity(ref_velocity)

    # IMU
    def get_imu_data(self, velocity, delta_state, phi):
        return self.imu.get_imu_data(velocity, delta_state, phi)

    # GPS
    if gps_use:
        def get_gps_data(self):
            return self.drive_gps_joint.get_position()

    # Laser Ranger
    if laserRanger_use:
        def get_laserRanger_data(self):
            return self.laser_ranger.get_y()

    # Potentiometer
    if potentiometer_use:
        def get_potentiometer_value(self):
            return self.potent.read_pot_value()

    # Drive Motor
    def set_velocity(self, input_velocity):
        self.drive_gps_joint.set_velocity(input_velocity)

    def set_current_const6_5A(self):
        # current_binary = b'\x02\x05\x06\x00\x00\x13\x88\x8b%\x03'  # 5.0
        # current_binary = b'\x02\x05\x06\x00\x00\x13\xec\xa7\x07\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x14PX\xc7\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x14\xb4\xe5m\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x15\x18\xa2:\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x15|\x8e\x18\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x15\xe0\xcc-\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x16Dl\x10\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x16\xa8P\xb2\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x17\x0c\x96\xed\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x17p)\xf6\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x17\xd4\xdc\x98\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x188\xf0\x04\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x18\x9c\x05j\x03'
        # current_binary = b'\x02\x05\x06\x00\x00\x19\x00tn\x03'
        current_binary = b'\x02\x05\x06\x00\x00\x19dXL\x03'  # 6.5
        current_binary = self.drive_gps_joint.set_current_binary(drive_current_ref)
        if current_control_enable:
            self.drive_gps_joint.heart_pipe_parent.send(current_binary)
            print(' --> %f  A current set for drive motor!!!' % (drive_current_ref) )

    # Steering Motor
    def set_handlebar_angular_velocity(self, angular_velocity):
        self.steering_motor.set_angular_velocity(angular_velocity)
        self.handlebar_angular_velocity = angular_velocity

    def get_handlebar_angular_velocity(self):
        return self.handlebar_angular_velocity

    def get_vesc_data(self):
        return self.drive_gps_joint.send_out_readings()
    # Stop bike
    def stop(self):
        self.steering_motor.stop()
        self.drive_gps_joint.stop()
        self.drive_gps_joint.stop_GPS_loop.set()

    def stop_all(self):
        self.stop()
