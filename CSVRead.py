import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from math import pi as PI
RAD_TO_DEG = 57.295
MAX_HANDLEBAR_ANGLE = ((3.0 / 3.0) * PI) / 6.0  # rad, 50 deg
csvFile = open("./ExpData/BikeData-20190506-104250.csv", "r")
# csvFile = open("./ExpData/BikeData-20190424-164117.csv", "r")
reader = csv.reader(csvFile)
result = []

for item in reader:
    if reader.line_num == 1:
        header = item
        continue
    result.append(item)
lenresult = len(result)
# Polynomial coefficients, should be CHANGED in consistence with the LQR control
P1 = [1.2183, -12.7460, 47.7502, -83.5453]
P2 = [0.0358, -0.4217, 3.2164, -0.2920]
P3 = [0.3092, -3.2593, 12.3393, -20.5164]
# No constant gains used
fixed_gains = 0
gains = [-20.8204, 5.0703, -5.1136]  # for 2m/s 25hz

references = [0, 0, 0]
deadband = 0.0
TIME_CONSTANT = 0.04
complementary_coef = 0.985
b = 1.095  # length between wheel centers [m]
CP_comp = 1
states = [0, 0, 0]


Cal_Wrong_Timestep_list = []
# print(result)
csvFile.close()
phi = []
phi.append(float(result[0][4]))
dDeltacmd = []
dDelta_realized = []
for k in range(0, len(header)):
    exec ('%s = []' % (header[k].replace(' ', '')))


for j in range(0, lenresult):
    for i in range(0, len(header)):
        exec('%s.append(float(result[%i][%i]))' % (header[i].replace(' ', ''), j, i))
    delta_state = Delta[j]
    # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
    phi_d_state = PhiDot[j]
    a_y = ay_roll_comp[j]
    velocity = MeasuredVelocity[j]

    states[0] = Phi[j]
    states[1] = Delta[j]
    states[2] = PhiDot[j]
    delta_state = Delta[j]

    CP_acc_g = CP_comp * ((velocity * velocity) / b) * math.tan(delta_state * 0.94) * (
                1 / 9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
    phi_acc = math.atan2(a_y + CP_acc_g * math.cos(phi[j]), az[j] - CP_acc_g * math.sin(phi[j]))
    # phi_acc=math.atan2(ay,  az)
    phi_state = complementary_coef * (phi[j] + phi_d_state * TIME_CONSTANT) + (
                1 - complementary_coef) * phi_acc
    phi.append(phi_state)
    # Calculate the control Input
    if velocity > 4.0:
        velocity_saturated = 4.0
    elif velocity < 1.5:
        velocity_saturated = 1.5
    else:
        velocity_saturated = velocity

    vel = velocity_saturated

    if fixed_gains == 0:
        vel3 = vel * vel * vel
        vel2 = vel * vel
        gain1 = P1[0] * vel3 + P1[1] * vel2 + P1[2] * vel + P1[3]
        gain2 = P2[0] * vel3 + P2[1] * vel2 + P2[2] * vel + P2[3]
        gain3 = P3[0] * vel3 + P3[1] * vel2 + P3[2] * vel + P3[3]
        feedback_gains =  [gain1, gain2, gain3]
    else:
        feedback_gains =  gains

    lqr_balance_control_signal = sum([(r - x) * k for r, x, k in zip(
        references, states, feedback_gains
    )])

    if Delta[j] >= MAX_HANDLEBAR_ANGLE or Delta[j] <= -MAX_HANDLEBAR_ANGLE:
        ddelta_com = 0
    elif lqr_balance_control_signal > deadband or lqr_balance_control_signal < -deadband:
        ddelta_com = lqr_balance_control_signal
    else:
        ddelta_com = 0
    dDeltacmd.append(ddelta_com)
    if ControlInput[j] != ddelta_com:
        Cal_Wrong_Timestep_list.append(j)
    if j >= 1:
        dDelta_realized.append((Delta[j]-Delta[j-1])/(Time[j] - Time[j-1]))

print [phi_state, delta_state, phi_d_state]

fig1 = plt.figure()
figphi = fig1.add_subplot(111)
figphi.plot(Time, np.array(Phi)*RAD_TO_DEG, '.', label='Online Phi Estimation')
figphi.plot(Time, np.array(phi[1:phi.__len__()])*RAD_TO_DEG, '-', label='Offline CompFilter')
figphi.plot(Time, np.array(phi_roll_comp)*RAD_TO_DEG, label='Phi Arduino integration')
plt.xlabel('Time(s)')
plt.ylabel('Phi(deg)')
plt.legend(loc='lower right')
plt.grid()
plt.title('Phi State Estimation')

fig2 = plt.figure()
figcmd = fig2.add_subplot(111)
figcmd.plot(Time, np.array(ControlInput)*RAD_TO_DEG, '.', label='Online')
figcmd.plot(Time, np.array(dDeltacmd)*RAD_TO_DEG, '-', label='Offline')
figcmd.plot(Time[0:Time.__len__()-1], np.array(dDelta_realized)*RAD_TO_DEG, label='Realized')
plt.xlabel('Time(s)')
plt.ylabel('AngularVel(deg /sec)')
plt.title('Handlebar Control')
plt.legend(loc='lower right')
plt.grid()
plt.show()

fig3 = plt.figure()
figcmd = fig3.add_subplot(111)
figcmd.plot(Time, np.array(Delta)*RAD_TO_DEG, '.', label='Encoder Reading')
figcmd.plot(Time, np.array(dDeltacmd)*RAD_TO_DEG, '-', label='Offline control calculation')
figcmd.plot(Time, np.array(Phi)*RAD_TO_DEG, '.', label='OnlinePhi')
figcmd.plot(Time[0:Time.__len__()-1], np.array(dDelta_realized)*RAD_TO_DEG, label='True Steering Rate')
plt.xlabel('Time(s)')
plt.ylabel('Angle(deg) AngularVel(deg /sec)')
plt.title('Handlebar Angle and AngularVelocity')
plt.legend(loc='lower right')
plt.grid()
plt.show()