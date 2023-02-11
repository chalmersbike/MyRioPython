from nifpga import Session
import time

with Session(".balancing_noGPS/drive_steering/FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_9KQYwNHlq+s.lvbitx", "RIO0") as session:
    print("Session started")

#Declaration of the session registers
    fpga_SteeringWriteDutyCycle = session.registers['Duty Cycle (%)']
    fpga_SteeringWriteFrequency = session.registers['Frequency (Hz)']

#Begin the execution of the FPGA VI
    session.abort()
    session.run()
    session.fpga_vi_state()
    print(session.fpga_vi_state())
    print(" I am here")
#PWM loop
    while True:
        fpga_SteeringWriteFrequency.write(5e4)
        print(fpga_SteeringWriteFrequency)
        fpga_SteeringWriteDutyCycle.write(80)
        print(fpga_SteeringWriteDutyCycle.read())
        time.sleep(0.5)