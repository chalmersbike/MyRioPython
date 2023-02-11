import pyvisa
import time

input_velocity = 2.0
rpm = input_velocity*0.2
driveMotor_CommunicationFrequency = 115200
#self.instr.baud_rate = driveMotor_CommunicationFrequency
str_run = 'run -s ' + str(rpm).zfill(4) + ' -f 9999\n'
str_stop = 'run -stop\n'
print(" I am here")
rm = pyvisa.ResourceManager()
port_b = rm.open_resource('ASRL2::INSTR')
    #port_a.query('IDN?')
port_b.baud_rate = 115200
print('Running the drive motor')
#while True:
port_b.write(str_run)
time.sleep(7)
    # print('Stopping the drive motor')
port_b.write(str_stop)
print('After writing the data for 10 seconds')
rm.close()
#port_a.close()
port_b.close()