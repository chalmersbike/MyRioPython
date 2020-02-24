from __future__ import with_statement
from __future__ import absolute_import
import pyvesc
from pyvesc import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition
import Adafruit_BBIO.UART as UART
import serial
import time

# Set your serial port here (either /dev/ttyX or COMX)
serialport = "/dev/ttyO1"

def get_values_example():
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        try:
            # Optional: Turn on rotor position reading if an encoder is installed
            ser.write(pyvesc.encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))
            while True:
                # Set the ERPM of the VESC motor
                #    Note: if you want to set the real RPM you can set a scalar
                #          manually in setters.py
                #          12 poles and 19:1 gearbox would have a scalar of 1/228
                # print "I am setting the rpm"
                msg = pyvesc.encode(SetRPM(1500))
                print str(msg)
                ser.write(msg)

                # Request the current measurement from the vesc
                # ser.write(pyvesc.encode_request(GetValues))
                #
                # # Check if there is enough data back for a measurement
                # if ser.in_waiting > 61:
                #     (response, consumed) = pyvesc.decode(ser.read(61))
                #
                #     # Print out the values
                #     try:
                #         print response.rpm
                #
                #     except:
                #         # ToDo: Figure out how to isolate rotor position and other sensor data
                #         #       in the incoming datastream
                #         #try:
                #         #    print(response.rotor_pos)
                #         #except:
                #         #    pass
                #         pass

                time.sleep(0.1)

        except KeyboardInterrupt:
            # Turn Off the VESC
            ser.write(pyvesc.encode(SetCurrent(0)))

if __name__ == u"__main__":
    get_values_example()
