import serial, time
import Adafruit_BBIO.UART as UART

REAR_MOTOR_PORT = '/dev/ttyS1'
COMMUNICATION_FREQUENCY = 115200

UART.setup("UART1")
ser = serial.Serial(port=REAR_MOTOR_PORT, baudrate=COMMUNICATION_FREQUENCY)
ser.close()
ser.open()

ser.write("ident\n")
#ser.write("run -s 3 -f 5 -pi\n")
#ser.write("run -s 0 -f 5 -pi\n")

ser.close()