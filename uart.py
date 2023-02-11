import pyvisa
import time
rm = pyvisa.ResourceManager()
rm.list_resources()
#my_instrument = rm.open_resource('ASRL2::INSTR')
# my_instrument2  = rm.open_resource('ASRL2::INSTR')
instr = rm.open_resource('ASRL1::INSTR')
PMTK_SET_BAUD_115200 = "$PMTK251,115200*1F" # 115200 bps
instr.baud_rate = 460800
# instr.baud_rate = 9600
# instr.write(PMTK_SET_BAUD_115200)
# instr.baud_rate = 460800
instr.flush(pyvisa.constants.VI_READ_BUF_DISCARD)
while True:
    if (instr.bytes_in_buffer) > 0:
        buffer_string = instr.read_raw().decode('utf-8',errors='ignore').split('\r\n')  # Read data from the GPS
        print(buffer_string)

    time.sleep(0.1)
# print(instr.read_raw().decode('utf-8',errors='ignore').split('\r\n'))
instr.close()
