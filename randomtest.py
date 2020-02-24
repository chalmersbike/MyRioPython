
#!/usr/bin/python
import Adafruit_BBIO.UART as UART
import serial
# from serial import Serial
import time
port = "/dev/ttyS1"
baudrate = 115200
UART.setup("UART1")
write_str = "ident\n"
# INPUT_PORT = 'P9_24'
# OUTPUT_PORT = 'P9_26'
# UART.setup(INPUT_PORT, UART.IN)
# UART.setup(OUTPUT_PORT, UART.OUT)
ser1 = serial.Serial(port, baudrate)
ser1.close()
ser1.open()
print("Serial is open")
ser1.write(write_str)
print("Command Sent!")
if ser1.isOpen():
     while 1:
         print("Serial is open")
         ser1.write(write_str)
#         time.sleep(1)
#         print "data has been sent!"
#     # print (ser1.readline())
ser1.close()
print "Serial is closed"