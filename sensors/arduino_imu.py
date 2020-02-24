import serial, time
import Adafruit_BBIO.UART as UART

SERIAL_PORT = '/dev/ttyACM0'
COMMUNICATION_FREQUENCY = 115200


class A_IMU(object):

    serial = None
    serial_ready = False
    data_ready = False

    def __init__(self):
        self.serial = serial.Serial(port=SERIAL_PORT, baudrate=COMMUNICATION_FREQUENCY, timeout=0) # timeout = 0 means it is non-blocking
        self.serial.close()
        self.serial.open()

        time_start_calib = time.time()
        while not self.serial_ready:
            if self.serial.inWaiting():
                data = self.serial.readline()
                if data.strip() == 'READY':
                    # self.serial.read(self.serial.inWaiting())  # flush
                    self.serial_ready = True
            else:
                # If calibration is not done after 10s then assume it failed
                # and restart the Arduino by closing and opening the serial communication
                if (time.time() - time_start_calib) > 15:
                    self.serial.close()
                    self.serial.open()
                    time_start_calib = time.time()
                    print 'Calibration took more than 15s, resetting the serial communication and the Arduino to restart calibration'
                else:
                    time.sleep(0.5)
                    print 'Waiting for calibration'
        while not self.data_ready:
            while self.serial.inWaiting() < 54:  # wait for string to appear #32/34/81
                time.sleep(0.01)  # 0.1
                self.serial.write('1')
            data = self.serial.read(54).strip()  # read data in buffer
            if data.startswith("@") and data.endswith("$") and data.count("#") == 7 and data is not None: #11
                self.serial.read(self.serial.inWaiting())  # flush
                self.data_ready = True
            print 'Waiting for valid data'
            time.sleep(0.01)     # 0.5

    def process_data(self, data):
        data = data.strip()  # remove trailing '\r\n'
        if data.startswith("@") and data.endswith("$") and data.count("#") == 7:  # check data is valid #11
            check1 = data.translate(None, '@$').split('#')  # remove data separators and split
            return [float(check1[0]), float(check1[1]), float(check1[2]), float(check1[3]), float(check1[4]),
                    float(check1[5]), float(check1[6]), float(check1[7])]  # convert and return
        else:
            print 'ERROR'
            print(data)

    def get_imu_data(self):

        if self.data_ready:
            self.serial.write('1')  # tell arduino to send data
            while self.serial.inWaiting() < 54: #81
                time.sleep(0.001)
            # data = self.serial.read(54)  # read single data string in buffer #81499
            self.data = self.serial.readline()
            self.processed_data = self.process_data(self.data) # process the data

            if self.processed_data is None:
                return self.get_imu_data()
            else:
                return self.processed_data  # return processed data
