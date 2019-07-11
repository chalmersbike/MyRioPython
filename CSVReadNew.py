import csv

import math

csvFile = open("BikeData-20190424-101517.csv", "r")
reader = csv.reader(csvFile)
result = []

for item in reader:
    if reader.line_num == 1:
        header = item
        continue
    result.append(item)

P1 = [1.2183, -12.7460, 47.7502, -83.5453]
P2 = [0.0358, -0.4217, 3.2164, -0.2920]
P3 = [0.3092, -3.2593, 12.3393, -20.5164]
fixed_gains = 0
gains = [-20.8204, 5.0703, -5.1136]  # for 2m/s 25hz
references = [0, 0, 0]
deadband = 0.0
Cal_Wrong_Timestep_list = []
# print(result)
csvFile.close()
phi = []
phi.append(float(result[0][4]))
for j in range(0, len(result)):
    for i in range(0, len(header)):
        exec('%s = float(result[j][%d])' % (header[i].replace(' ', ''), i))
    delta_state = Delta
    # imu_data=[phi_roll_compensated, phi_uncompensated, phi_dot, a_x, a_y_roll_compensated, a_y, a_z]
    phi_d_state = PhiDot
    ay = ay_roll_comp
    velocity = MeasuredVelocity
    TIME_CONSTANT = 0.04
    complementary_coef = 0.985
    b = 1.095  # length between wheel centers [m]
    states = [0, 0, 0]
    states[0] = Phi
    states[1] = Delta
    states[2] = PhiDot
    delta_state = Delta
    CP_comp = 1
    CP_acc_g = CP_comp * ((velocity * velocity) / b) * math.tan(delta_state * 0.94) * (
                1 / 9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
    phi_acc = math.atan2(ay + CP_acc_g * math.cos(phi[j]), az - CP_acc_g * math.sin(phi[j]))
    # phi_acc=math.atan2(ay,  az)
    phi_state = complementary_coef * (phi[j] + phi_d_state * TIME_CONSTANT) + (
                1 - complementary_coef) * phi_acc
    phi.append(phi_state)

    # CP_acc_g = CP_comp * ((velocity * velocity) / b) * math.tan(delta_state * 0.94) * (
    #             1 / 9.81)  # 0.94 = sin( lambda ) where lambda = 70 deg
    # phi_acc = math.atan2(ay + CP_acc_g * math.cos(states[0]), az - CP_acc_g * math.sin(states[0]))
    # # phi_acc=math.atan2(ay,  az)
    # phi_state = complementary_coef * (states[0] + phi_d_state * TIME_CONSTANT) + (
    #             1 - complementary_coef) * phi_acc

    # Calculate the control Input
    if velocity > 4.0:
        velocity_saturated = 4.0
    elif velocity < 1.5:
        velocity_saturated = 1.5
    else:
        velocity_saturated = velocity

    vel = velocity_saturated

    # return [np.interp(velocity, K_VELOCITY_LOOKUP, gain['LQR_gain_multipliers']) for gain in K_MATRIX]
    # return [-18.7580, 4.1700, -4.9389]
    # return [-22.3223,    5.3046,   -5.4850]  # for 2m/s 50hz
    # return [-15.9870,    6.0476,   -3.9479]  # for 2.5m/s 50hz
    # return [-12.3295,    6.6724,   -3.0655]  # for 3m/s 50hz
    # return [-10.0619,    7.2404,   -2.5222]  # for 3.5m/s 50hz

    # return [-20.8204,    5.0703,   -5.1136]  # for 2m/s 25hz
    # return [-14.8850,    5.7641,   -3.6725]  # for 2.5m/s 25hz
    # return [-11.4480,    6.3461,   -2.8422]  # for 3m/s 25hz
    # return [-11.4480,    6.3461,   -2.8422]  # for 3.5m/s 25hz


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

    if lqr_balance_control_signal > deadband or lqr_balance_control_signal < -deadband:
        ddelta_com = lqr_balance_control_signal
    else:
        ddelta_com = 0

    if ControlInput != ddelta_com:
        Cal_Wrong_Timestep_list.append(j)
# phi_state = imu_data[0] # for only roll compensated phi
# phi_state = imu_data[1] for uncompensated phi

print [phi_state, delta_state, phi_d_state]


