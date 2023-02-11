from nifpga import Session
import time

# with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_O+w-wwa0L-U.lvbitx", "RIO0") as session:
# with Session("./FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_9KQYwNHlq+s.lvbitx", "RIO0") as session:
with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_yn8cRoiCUew.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/balancingnogps_FPGATarget_FPGAbalancingnoG_Vx2edSru1AQ.lvbitx", "RIO0") as session:
# with Session("../FPGA Bitfiles/fpgadrivesteerin_FPGATarget_FPGAdrivesteerin_jCOIVVECmVE.lvbitx", "RIO0") as session:


    print("Session started")

#Declaration of the session registers
    # fpga_SteeringWriteDutyCycle = session.registers['Duty Cycle (%)']
    # fpga_SteeringWriteFrequency = session.registers['Frequency (Hz)']
    # fpga_SteeringEnable = session.registers['Write']
    fpga_SteeringWriteDutyCycle = session.registers['Steering PWM Duty Cycle']
    fpga_SteeringWriteFrequency = session.registers['Steering PWM Frequency']
    fpga_SteeringEnable = session.registers['Steering Enable']

#Begin the execution of the FPGA VI
    session.abort()
    session.run()
    print("Session is running")
    #time.sleep(3)

#Steering loop
    #fpga_SteeringEnable.write(False)
    print("Steering is desactivated")

    time.sleep(3)

    print("Steering is activated")
    fpga_SteeringEnable.write(True)
    #fpga_SteeringWriteFrequency.write(500)
    #fpga_SteeringWriteDutyCycle.write(51)
    #print("Steering ha been tricked")
    print("Steering is NOW activated")
    #fpga_SteeringEnable.write(False)
    fpga_SteeringWriteDutyCycle.write(50)
    fpga_SteeringWriteFrequency.write(1000)

    time.sleep(7)

    fpga_SteeringEnable.write(True)
    fpga_SteeringWriteDutyCycle.write(50)
    fpga_SteeringEnable.write(False)

#     print("Steering is going right")
#     time.sleep(5)
#
#
#     fpga_SteeringWriteFrequency.write(1e3)
#     fpga_SteeringWriteDutyCycle.write(49)
#     print(fpga_SteeringWriteFrequency.read())
#     print(fpga_SteeringWriteDutyCycle.read())
#     print("Steering is going left")
#     time.sleep(3)
#
#
#     fpga_SteeringWriteDutyCycle.write(51)
#     print(fpga_SteeringWriteDutyCycle.read())
#     time.sleep(3)
#
#     print("Steering is neutral")
#     fpga_SteeringWriteDutyCycle.write(50)
#     time.sleep(5)
#
#     print("Steering is desactivate")
#     fpga_SteeringEnable.write(False)
# """
#     while True:
#         fpga_SteeringWriteFrequency.write(1e3)
#         #fpga_SteeringWriteDutyCycle.write(49)
#         print(fpga_SteeringWriteFrequency.read())
#         fpga_SteeringWriteDutyCycle.write(51)
#         print(fpga_SteeringWriteDutyCycle.read())
#         time.sleep(0.5)
#         fpga_SteeringWriteDutyCycle.write(50)
#         time.sleep(0.5)
# """