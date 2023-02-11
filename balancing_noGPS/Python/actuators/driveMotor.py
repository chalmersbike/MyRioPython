from param import *
import serial, time, numpy
import pyvisa

debug = 0

class DriveMotor(object):
    serial = None

    def __init__(self, session):
        self.rm = pyvisa.ResourceManager()
        self.instr = self.rm.open_resource('ASRL2::INSTR') # change back
        self.instr.baud_rate = 115200 #driveMotor_CommunicationFrequency  change back

        if debug:
            print('Drive Motor : Serial port opened')
        self.Time = -1 # Initialize time to -1 as a way to check that we correctly read from the controller after starting the logging

    def serial_write_character(self, character):
        try:
            self.instr.write(character)
        except:
            print('Drive Motor : Serial port is not open!')

    def rear_set_rpm(self, rpm):
        # rpm_string = 'run -s ' + str(rpm).zfill(4) + ' -f 5 -pi\n'
        # rpm_string = 'run -s ' + str(rpm).zfill(4) + ' -f 9999 -pi\n' # '-s' sets the speed of the motor, '-f' sets the maximum torque/current. Setting '-f' to a very large value will allow the motor to use the maximum current possible
        rpm_string = 'run -s ' + str(rpm).zfill(4) + ' -f 9999\n' # '-s' sets the speed of the motor, '-f' sets the maximum torque/current. Setting '-f' to a very large value will allow the motor to use the maximum current possible
        self.instr.write(rpm_string)

    def set_velocity(self, input_velocity):
        # self.rear_set_pwm(self._m_per_second_to_pwm(input_velocity))
        self.rear_set_rpm(input_velocity*0.25)
        # for GEAR 6Th the coef vel -> pwm = 0.31
        # self.rear_set_rpm(input_velocity * 0.31)

    def stop(self):
        stop_string = 'run -stop\n'
        self.instr.write(stop_string)
        # for character in pwm_string:
        #     self.serial_write_character(character)
        #     time.sleep(0.001)  # fixes comms issue

    def startLog(self):
        self.logReady = False
        self.instr.flush()
        startLog_string = "log -p " + str(1.0/driveMotor_logFrequency) + " -k\n"
        self.instr.write(startLog_string)
        while not self.logReady:
            self.readLog()
            if self.Time != -1:
                self.logReady = True
        print('Drive motor : Log started')

    def stopLog(self):
        stopLog_string = "\n"
        self.instr.write(stopLog_string)

    def getLog(self):
        self.readLog()
        return self.Time, self.refSpeed, self.measSpeed, self.motorCurrent, self.busCurrent, self.busVoltage

    def readLog(self):
        try:
            # Read through all received lines until we find the last (most recent) one
            while self.instr.bytes_in_buffer > 0:
                buffer = self.instr.read_raw()  # Read data from the motor controller
            # print(buffer)

            # Process data
            logData = [x.strip() for x in buffer.split('; ')]
            # print(logData)
            self.process_logData(logData) # Process data
        except Exception as e:
            # print(e)
            pass

    def process_logData(self, line):
        try:
            self.Time = float(line[0])
            self.refSpeed = float(line[1])
            self.averageRefSpeed = float(line[2])
            self.SRefSpeed = float(line[3]) # What is that ? What does the 'S.' mean ?
            self.measSpeed = float(line[4])
            self.motorRevs = float(line[5])
            self.motorCurrent = float(line[6])
            self.IntErr = float(line[7]) # What is that ?
            self.maxBusCurrent = float(line[8])
            self.busCurrent = float(line[9])
            self.busVoltage = float(line[10])
            self.PSUState = float(line[11])
            self.outputTorque = float(line[12])
            self.outputForce = float(line[13])
            self.outputForceDerivative = float(line[14])
            self.temperature = float(line[15])
            self.FETTemperature = float(line[16])
        except Exception as e:
            # print(e)
            pass