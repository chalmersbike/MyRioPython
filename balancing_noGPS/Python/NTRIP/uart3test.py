# import Adafruit_BBIO.UART as UART
import serial
# UART.setup("UART3")
ser = serial.Serial(port="/dev/ttyO5", baudrate=115200)
# ser = serial.Serial(port="/dev/ttyO1", baudrate=115200)
ser.close()
ser.open()
# ser1.close()
# ser1.open()
if ser.isOpen():
    print "Serial is open!"
    ser.write("Hello World!\r\n")
    print 'Words sent'
    print(ser.readline())
ser.close()