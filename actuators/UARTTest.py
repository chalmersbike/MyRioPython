import serial, time, numpy
import Adafruit_BBIO.UART as UART
REAR_MOTOR_PORT = '/dev/ttyO1'
UART.setup("UART1")
COMMUNICATION_FREQUENCY = 115200
serial1 = serial.Serial(port=REAR_MOTOR_PORT, baudrate=COMMUNICATION_FREQUENCY)
serial1.close()
serial1.open()
character = 'HELLO WORLD\n'
while 1:
    serial1.write(character)
    # time.sleep(0.1)
# print serial1.readline()