from param import *
import serial, time, numpy
# from pyvesc import VESC
import pyvesc
import pysnooper
import pyvisa
import signal
import threading
import sys
import multiprocessing as mul
# import pysnooper
import warnings

# rpm = 0
# avg_motor_current = 0
# v_in = 0
# avg_input_current = 0
class DriveMotor(object):
    serial = None

    def __init__(self, session, rm):
        # self.rm = pyvisa.ResourceManager()
        # self.instr = self.rm.open_resource('ASRL1::INSTR')
        self.instr = rm.open_resource('ASRL2::INSTR')  # change back
        self.instr.baud_rate = driveMotor_CommunicationFrequency

        # self.instr.read_termination = '\n'
        # self.VESCmotor = VESC(serial_port=driveMotor_port,baudrate=driveMotor_CommunicationFrequency,start_heartbeat=False)

        # Start heartbeat threat
        # self.heart_beat_thread = threading.Thread(target=self._heartbeat_cmd_func, daemon=True)
        # self._stop_heartbeat = threading.Event()
        # self.heart_beat_thread.start()


        ## Test Multi-Process
        self.last_vesc_data = [0, 0, 0, 0]
        self.heart_pipe_parent, self.heart_pipe_child = mul.Pipe()
        self.vescdata_pipe_parent, self.vescdata_pipe_child = mul.Pipe()
        if read_vesc_data:
            self.read_sensor_msg = pyvesc.protocol.interface.encode_request(pyvesc.messages.getters.GetValues())

        self.heart_beat_process = mul.Process(target=self._heartbeat_vsec_read,
                                              args=(self.heart_pipe_child, self.vescdata_pipe_child,),
                                              daemon=True)
        self._stop_heartbeat = mul.Event()
        self.heart_beat_process.start()




            # self.read_vesc_state = threading.Thread(target=self.read_all_sensors, daemon=True)
            # self.read_vesc_state.start()



        if debug:
            print('Drive Motor : Serial port opened')
        self.Time = -1 # Initialize time to -1 as a way to check that we correctly read from the controller after starting the logging


        # self.rpm = 0
        # self.avg_motor_current = 0
        # self.v_in = 0
        # self.avg_input_current = 0
    # @pysnooper.snoop()
    def _heartbeat_vsec_read(self, pipe_heart, pipe_data):
        """
        Continuous function calling that keeps the motor alive
        """
        # start_heart_beat = False
        # start_vesc_data_query = False
        start_heart_beat = False
        start_vesc_data_query = False
        nr_of_empty_loop = 0
        t_last_query = time.time()
        t_last_heart = time.time()
        query_sent = False

        while not self._stop_heartbeat.is_set():
            while pipe_heart.poll():
                cmd_heart = pipe_heart.recv()
                if cmd_heart == 'start_heart_beat':
                    start_heart_beat = True
                elif cmd_heart == 'stop_heart_beat':
                    start_heart_beat = False
                else:
                    self.instr.write_raw(cmd_heart)

            while pipe_data.poll():
                cmd_data = pipe_data.recv()
                if cmd_data == 'start_vesc_query':
                    start_vesc_data_query = True
                    t_last_query = time.time()
                elif cmd_data == 'stop_vesc_query':
                    start_vesc_data_query = False


            t_left_next_beat = time.time() - t_last_heart
            if start_heart_beat and (t_left_next_beat > 0.1):
                self.instr.write_raw(pyvesc.messages.setters.alive_msg)
                t_last_heart = time.time()
                # print('Alive MSG Sent!!!!')

            if start_vesc_data_query:
                slp_time = time.time() - t_last_query
                if not query_sent:
                    self.instr.write_raw(self.read_sensor_msg)
                    query_sent = True
                if slp_time < 0.01:
                    time.sleep(0.01 - slp_time)
                buffer_size = self.instr.bytes_in_buffer
                print(buffer_size)
                if buffer_size == 78:
                    t_last_query = time.time()
                    readraw = self.instr.read_bytes(78)
                    query_sent = False
                    (self.vesc_sensor_response, consumed) = pyvesc.protocol.interface.decode(readraw)
                    pipe_data.send([self.vesc_sensor_response.rpm,
                               self.vesc_sensor_response.avg_motor_current,
                               self.vesc_sensor_response.v_in,
                               self.vesc_sensor_response.avg_input_current])
                    # print('DATA Received!!!!')
                    if nr_of_empty_loop is not 0:
                        nr_of_empty_loop = 0
                elif buffer_size == 156:
                    readraw = self.instr.read_bytes(156)  # Bytes type

                    warnings.warn('DOUBLE DATA Received!!!!')
                    # print('DOUBLE DATA Received!!!!')
                    readraw = readraw[78:156]
                    query_sent = False
                    t_last_query = time.time()
                    (self.vesc_sensor_response, consumed) = pyvesc.protocol.interface.decode(readraw)
                    if self.vesc_sensor_response is not None:
                        pipe_data.send([self.vesc_sensor_response.rpm,
                                        self.vesc_sensor_response.avg_motor_current,
                                        self.vesc_sensor_response.v_in,
                                        self.vesc_sensor_response.avg_input_current])

                    if nr_of_empty_loop is not 0:
                        nr_of_empty_loop = 0
                else:
                    nr_of_empty_loop += 1
                    if nr_of_empty_loop >= 5:
                        self.instr.flush(pyvisa.constants.VI_READ_BUF_DISCARD)
                        t_last_query = time.time()
                        query_sent = False
                        nr_of_empty_loop = 0
                        warnings.warn('Multiple Failure in VESC DATA!!!!')
                        print('Multiple Failure in VESC DATA!!!!')

            else:
                time.sleep(0.1)







    def stop_heartbeat(self):
        self.heart_pipe_parent.send('stop_heart_beat')
        self._stop_heartbeat.set()
        # self.heart_beat_thread.join()
        self.heart_beat_process.join(3)

    # @pysnooper.snoop()
    # def read_vesc_data_loop(self, pipe):
    #     t_last_query = time.time()
    #     start_vesc_data_query = False
    #     while not self._stop_heartbeat.is_set():
    #         while pipe.poll():
    #             cmd_heart = pipe.recv()
    #             if cmd_heart == 'start_vesc_query':
    #                 start_vesc_data_query = True
    #             elif cmd_heart == 'stop_vesc_query':
    #                 start_vesc_data_query = False
    #         if start_vesc_data_query:
    #             slp_time = time.time() - t_last_query
    #             if slp_time < 0.01:
    #                 time.sleep(0.01 - slp_time)
    #             self.instr.write_raw(self.read_sensor_msg)
    #             if self.instr.bytes_in_buffer >= 78:
    #                 t_last_query = time.time()
    #                 readraw = self.instr.read_bytes(78)
    #                 (self.vesc_sensor_response, consumed) = pyvesc.protocol.interface.decode(readraw)
    #                 pipe.send([self.vesc_sensor_response.rpm,
    #                            self.vesc_sensor_response.avg_motor_current,
    #                            self.vesc_sensor_response.v_in,
    #                            self.vesc_sensor_response.avg_input_current])
                    # rpm = self.vesc_sensor_response.rpm
                    # avg_motor_current = self.vesc_sensor_response.avg_motor_current
                    # v_in = self.vesc_sensor_response.v_in
                    # avg_input_current = self.vesc_sensor_response.avg_input_current
                ###################
    # @pysnooper.snoop()
    def rear_set_rpm(self, rpm):
        # self.VESCmotor.set_rpm(int(rpm))
        self.heart_pipe_parent.send(pyvesc.protocol.interface.encode(pyvesc.messages.setters.SetRPM(int(rpm))))
        # self.instr.write_raw(pyvesc.protocol.interface.encode(pyvesc.messages.setters.SetRPM(int(rpm))))
        # if input_velocity > 0:
        #     self.heart_pipe_parent.send('start_heart_beat')
        # else:
        #     self.heart_pipe_parent.send('stop_heart_beat')

    def set_velocity(self, input_velocity):
        # self.rear_set_pwm(self._m_per_second_to_pwm(input_velocity))
        self.rear_set_rpm(input_velocity*600) # Maxime's previous setting, might work for RED bike
        # self.rear_set_rpm(input_velocity * 900)
        # self.rear_set_rpm(input_velocity * 1300)
        # print('VESC : Set speed')
        # self.VESCmotor.start_heartbeat()
        # for GEAR 6Th the coef vel -> pwm = 0.31
        # self.rear_set_rpm(input_velocity * 0.31)

    def stop(self):
        print('VESC : Stop')
        self.set_velocity(0)
        self.stop_heartbeat()

    def retrieve_vesc_data(self):
        while self.vescdata_pipe_parent.poll():
            self.last_vesc_data = self.vescdata_pipe_parent.recv()
        return self.last_vesc_data

    # def read_all_sensors(self):
    #     global rpm, avg_motor_current, v_in, avg_input_current
    #     self.read_sensor_msg = pyvesc.protocol.interface.encode_request(pyvesc.messages.getters.GetValues())
    #     while not self._stop_heartbeat.isSet():
    #         self.instr.write_raw(self.read_sensor_msg)
    #         time.sleep(0.01)
    #         if self.instr.bytes_in_buffer >= 78:
    #             readraw = self.instr.read_bytes(78)
    #             (self.vesc_sensor_response, consumed) = pyvesc.protocol.interface.decode(readraw)
    #             rpm = self.vesc_sensor_response.rpm
    #             avg_motor_current = self.vesc_sensor_response.avg_motor_current
    #             v_in = self.vesc_sensor_response.v_in
    #             avg_input_current = self.vesc_sensor_response.avg_input_current




