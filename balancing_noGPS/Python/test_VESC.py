from param import *
import serial, time, numpy
import pyvesc
import pysnooper
import pyvisa
import signal
import sys
import math
import warnings
import pickle

def load_lookup_table(filename):
    with open(filename, 'rb') as f:
        data = pickle.load(f)
    return data['lookup_table'], data['params']


def interpolate(lookup_table, start, step, x):
    index = int((x - start) / step)
    return lookup_table[index]

CURRENT_CONTROL = True
SPEED_CONTROL = False
rm = pyvisa.ResourceManager()
instr_VESC = rm.open_resource('ASRL2::INSTR')  # change back
instr_VESC.baud_rate = driveMotor_CommunicationFrequency
instr_VESC.flush(pyvisa.constants.VI_READ_BUF_DISCARD)

if CURRENT_CONTROL:
    lookup_table, params = load_lookup_table('./actuators/lookup_table_current.pkl')
    start, stop, step = params
    print(start, stop, step)
    interp_value = 0.0
    combined_command = interpolate(lookup_table, start, step, interp_value)
elif SPEED_CONTROL:
    lookup_table, params = load_lookup_table('./actuators/lookup_table_speed.pkl')
    start, stop, step = params
    print(start, stop, step)
    interp_value = 1.5
    combined_command = interpolate(lookup_table, start, step, interp_value)


# heart_beat_msg = pyvesc.protocol.interface.encode(
#                             pyvesc.messages.setters.SetRPM(int(0)))
# read_sensor_msg = pyvesc.protocol.interface.encode_request(pyvesc.messages.getters.GetValues())
# # combined_command = heart_beat_msg + read_sensor_msg
# combined_command = read_sensor_msg + heart_beat_msg
# # combined_command =  read_sensor_msg
print(combined_command)
for ind in range(0,200):
    instr_VESC.write_raw(combined_command)
    time.sleep(0.1)
    print('Received Bytes')
    print(instr_VESC.bytes_in_buffer)
    readraw = instr_VESC.read_bytes(79)
    # readraw = instr_VESC.read_raw()
    (vesc_sensor_response, consumed) = pyvesc.protocol.interface.decode(readraw)
    # print(vesc_sensor_response.rotor_pos)
    # print(instr_VESC.bytes_in_buffer)
    print(vesc_sensor_response.v_in)
    print(vesc_sensor_response.rpm)
    # time.sleep(5)
    time.sleep(0.01)
