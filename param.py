from math import pi as PI
# from scipy import signal
import numpy as np
INITIAL_SPEED = 4  # m/s
fixed_gains = 0 # 1 for velocity independent LQR gains, 0 for velocity dependent LQR gains
gains= [-20.8204,    5.0703,   -5.1136]  # for 2m/s 25hz
rad2deg = 180 / 3.1415
deg2rad = 3.1415 / 180
# gains= [-47.9208,    4.9023,   -9.1986]  # for 2m/s 25hz different too aggressive!

# P1 = [2.4486,  -25.4536,   91.9354, -125.0781]
# P2 = [0.1273,   -1.3272,    5.6469,   -2.2087]
# P3 = [0.5993,   -6.2280,   22.4694,  -30.5709]
#Original Parameters From Umur
# P1 = [ 1.2183,  -12.7460,   47.7502,  -83.5453]
# P2 = [0.0358,   -0.4217,    3.2164,   -0.2920]
# P3 = [0.3092,   -3.2593,   12.3393,  -20.5164]

# # New Parameters R 42 Q diag 100 100 10
# P1 = [-1.9517,  20.3042,   -73.5315,  100.1292]
# P2 = [0.1005,   -1.0573,    4.1598,   -1.0215]
# P3 = [-0.4772,   4.9647,   -17.9717,  24.4728]

# Parameter R = 42 Q = diag(50 50 10) for box in the middle
# P1 = [-1.3974,   14.6162,  -53.4552,   74.3334]
# P2 = [0.1037,   -1.1157,    4.8481,   -2.6266]
# P3 = [-0.3221,    3.3679,  -12.2996,   17.1063]
#
# New Parameters R 2000 Q diag 2000 10 100 for box in
# Rparam = 2000
# Qparam = [2000, 10, 100]
# Box = [0.5195, 0.4964] # h, a
# P1 = [ -1.1224,  11.7186,  -42.9249,   60.6360]
# P2 = [0.0750,   -0.8172,   3.7497,   -1.3173]
# P3 = [-0.2583,   2.6967,    -9.8781,    13.9537]

# New Parameters R 0.2336 Q as follows for box in
# Rparam = 0.2336
# Qparam = [15.9549, 5.1214, -5.7298, 5.1214, 27.6373, 21.6896, -5.7298, 21.6896, 27.3988] # Q = [15.9549 5.1214 -5.7298 ; 5.1214 27.6373 21.6896 ; -5.7298 21.6896 27.3988];
# Box = [0.5195, 0.4964] # h, a
# P1 = [-3.5154, 37.0583, -136.3263,  187.6935]
# P2 = [0.3379, -3.6754, 14.8332, -13.0876]
# P3 = [-0.7947, 8.1516, -29.1581, 42.1145]

#New Parameters R 2.336 Q as follows for box in
# Rparam = 2.336
# Qparam = [15.9549, 5.1214, -5.7298, 5.1214, 27.6373, 21.6896, -5.7298, 21.6896, 27.3988] # Q = [15.9549 5.1214 -5.7298 ; 5.1214 27.6373 21.6896 ; -5.7298 21.6896 27.3988];
# Box = [0.5195, 0.4964] # h, a
# P1 = [ -1.9767, 20.7729,  -76.3035,  106.3873]
# P2 = [0.1567,-1.6784 ,   7.3679,   -5.4964]
# P3 = [-0.4650,4.7961, -17.1230,   24.2448]

#New Parameters R 23.36 Q as follows for box in middle
# Rparam = 23.36
# Qparam = [15.9549, 5.1214, -5.7298, 5.1214, 27.6373, 21.6896, -5.7298, 21.6896, 27.3988] # Q = [15.9549 5.1214 -5.7298 ; 5.1214 27.6373 21.6896 ; -5.7298 21.6896 27.3988];
# Box = [0.5195, 0.4964] # h, a
# P1 = [-1.4040, 14.7197,  -53.9723,   75.0765]
# P2 = [0.1095, -1.1466,    4.9706,   -2.6981]
# P3 = [-0.3270,    3.4175,  -12.4113,   17.2802]

# #New Parameters R 1 Q as follows for box in middle
# Rparam = 1
# Qparam = [1, 1,  0, 1, 1,  0,   0,   0, 1]
# Box = [0.5195, 0.4964] # h, a
# P1 = [ -1.2121,13.2710  ,-50.5152   ,71.3617]
# P2 = [ 0.1436 ,  -1.4308 ,   5.5836,   -3.0983]
# P3 = [-0.2931  ,  3.1692 , -11.8310  , 16.5872]

# #New Parameters R 1 Q as follows for box in middle
# Rparam = 1
# Qparam = [0.1, 0.1,  0, 0.1, 0.1,  0,   0,   0, 0.1]
# Box = [0.5195, 0.4964] # h, a
# P1 = [-1.1486, 12.2238,   -45.5709,    63.9170]
# P2 = [0.1011,    -1.0887,     4.5456,    -2.0222]
# P3 = [-0.2649,     2.8193,   -10.4953,    14.7167]


#New Parameters R 1 Q as follows for box in middle
Rparam = 1
Qparam = [0.1, 0.01,  0, 0.01, 0.01,  0,   0,   0, 0.1]
Box = [0.5195, 0.4964] # h, a
P1 = [ -1.1624,12.1370 , -44.4112  , 61.7715]
P2 = [0.0830 ,  -0.9069 ,   3.9876  , -1.4832]
P3 = [ -0.2677,    2.7953,  -10.2177 ,  14.2146]


# P3 = [ 0, 0,  0 , 0]


# #New Parameters R 1 Q as follows for box in middle
# Rparam = 1
# Qparam = [10, 10,  0, 10, 10,  0,   0,   0, 10]
# Box = [0.5195, 0.4964] # h, a
# P1 = [-1.4956, 17.0935,  -66.9665,   95.0564]
# P2 = [0.1845,   -1.8449,    7.3462,   -5.3212]
# P3 = [-0.4316,  4.5361,  -16.2728, 22.5458]


# #New Parameters R 1 Q as follows for box in middle
# Rparam = 1
# Qparam = [100, 100,  0, 100, 100,  0,   0,   0, 100]
# Box = [0.5195, 0.4964] # h, a
# P1 = [-2.4653,28.9936,-115.3505,161.5329]
# P2 = [0.3372,-3.5359,13.6810,-12.3322]
# P3 = [-0.7396,7.5465,-26.5185,37.5230]

# #New Parameters R 1 Q as follows for box in middle
# Rparam = 1
# Qparam = [100, 0,  0, 0, 100,  100,   0,   1000, 100]
# Box = [0.5195, 0.4964] # h, a
# P1 = [-3.3862, 35.5649,  -130.5284,   181.3757]
# P2 = [0.3171 ,   -3.4620 ,   14.1438 ,  -12.2904]
# P3 = [-0.7623  ,   7.8332 ,  -28.1096 ,   40.8275]

# # New Parameters R 1 Q as follows for box in middle
# Rparam = 1
# Qparam = [5, 5,  0, 5, 5,  0,   0,   0, 5]
# Box = [0.5195, 0.4964] # h, a
# P1 = [-1.3713, 15.4549,   -59.9794,    85.0412]
# P2 = [0.1707,    -1.6808,     6.6050,    -4.3829]
# P3 = [-0.3745,     3.9852,   -14.4978,    20.1074]


# # New Parameters R 2000 Q diag 2000 10 100
# P1 = [-1.5174,  15.7778,  -57.3463,   79.3139]
# P2 = [0.0771,   -0.8339,    3.7868,   -0.2679]
# P3 = [-0.3357,   3.4901,  -12.6850,   17.5443]

# New Parameters R 42 Q diag 50 50 10
# P1 = [-1.8671,  19.4317,  -70.4737,   96.1203]
# P2 = [0.1068,   -1.1259,    4.7892,   -1.2735]
# P3 = [-0.4135,    4.3027,  -15.5888,   21.2629]

# gains= [-14.8850,    5.7641,   -3.6725]  # for 2.5m/s 25hz
# gains = [-11.4480, 6.3461, -2.8422]  # for 3m/s 25h# z
# gains= [-11.4480,    6.3461,   -2.8422]  # for 3.5m/s 25hz
# gains= [  -22.4049,    5.0941,   -5.3617] #manual input
TEST_TIME = 5555.0  # seconds
start_up_interval = 5 # second, wait for several seconds before real starting up
speed_up_time = 3.0 # seconds
deadband = 0.0/57.29577  # rad/s deadband for steering



CURRENT_SENSE_PORT = 'P9_38'
RPM_SENSE_PORT = 'P9_40'
# Controller Parameters
# CONTROLLER_FREQUENCY = 100 # 25  #50  # Hz
CONTROLLER_FREQUENCY = 50 # 25  #50  # Hz
sample_time = 1.0/CONTROLLER_FREQUENCY

# Test Parameters
# MAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad, 50 deg
LowSpeedMAX_HANDLEBAR_ANGLE = ((5.0 / 3.0) * PI) / 6.0  # rad, 30 deg
HighSpeedMAX_HANDLEBAR_ANGLE = ((1.0/ 3.0) * PI) / 6.0   # rad 10 deg
HIGHSPEEDBOUND = 3  # m/s
MAX_HANDLEBAR_ANGLE = LowSpeedMAX_HANDLEBAR_ANGLE
MIN_HANDLEBAR_ANGLE = - MAX_HANDLEBAR_ANGLE
MAX_HANDLEBAR_ANGLE_ADJUST = 0  # Switch on/off the handle bar MAX angle range dynamic adjustment
HANDLEBAR_CORRECTION_ANGVEL = 0  # When the handlebar is out of tolerance range, drag it back to the region

MAX_LEAN_ANGLE = 0.6  # rad (35 deg)
MIN_LEAN_ANGLE = - MAX_LEAN_ANGLE
comp_coef= 0.985 # 0.985 # coefficient of complementary filter
centripetal_compensation=1 # 1 to turn centripetal acceleration compensation on, 0 to turn it off
roll_compensation = 1 # 1 to turn roll acceleration compensation on, 0 to turn it off


# PID Velocity Controller Parameters
pid_velocity_active = False  # Boolean to control if PID controller is used
pid_velocity_reference = INITIAL_SPEED  # m/s
pid_velocity_P = 3.0
pid_velocity_I = 0.01
pid_velocity_D = 0.0
pid_velocity_sample_time = 1.0 / CONTROLLER_FREQUENCY

# Path Tracking
PATH_TYPE = 'NONE'  # [NONE, CIRCLE, STRAIGHT]
# PATH_TYPE = 'STRAIGHT'  # [NONE, CIRCLE, STRAIGHT]
PATH_RADIUS = 20.0  # m
time_path_stay = 10.0 # s
time_path_slope = 10.0 # s
slope = 1/100.0
path_sine_amp = 0.15 # m
path_sine_freq = 1/10.0 # Hz
path_choice = 'pot'
#path_choice = 'overtaking'
#path_choice = 'sine'

# PID Lateral Position Controller Parameters
pid_lateral_position_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
pid_lateral_position_P = 0.2
pid_lateral_position_I = 0.0
pid_lateral_position_D = 0.0
pid_lateral_position_sample_time = 1.0 / CONTROLLER_FREQUENCY

# PID Direction Controller Parameters
pid_direction_reference = 0.0  # PID code uses constant reference setpoint and feedback. Since this controller doesn't have a constant setpoint, we only use the feedback value.
pid_direction_P = 0.8
pid_direction_I = 0.4
pid_direction_D = 0.0
pid_direction_sample_time = 1.0 / CONTROLLER_FREQUENCY

# PID Steering Controller Parameters
pid_steering_reference = 0.0  # PID code uses constant reference setpoint and feedback. Since this controller doesn't have a constant setpoint, we only use the feedback value.
pid_steering_P = 10.0
pid_steering_I = 10.0
pid_steering_D = 0.0
pid_steering_sample_time = 1.0 / CONTROLLER_FREQUENCY


# Filter requirements.
order = 2
fs = CONTROLLER_FREQUENCY       # sample rate, Hz
cutoff = 8# desired cutoff frequency of the filter, Hz
windowSize = 10

# Get the filter coefficients so we can check its frequency response.
# b, a = butter_lowpass(cutoff, fs, order)
# !!! TEST !!!

# PID Balance Controller Parameters
pid_balance_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
# pid_balance_P = 0.7
# pid_balance_P = 5.0
pid_balance_P = 1.57
pid_balance_I = 0.0
pid_balance_D = 0.0
# pid_balance_P = 3.418/5
# pid_balance_I = 1.327
# pid_balance_D = 0.0646
pid_balance_sample_time = 1.0 / CONTROLLER_FREQUENCY

# PID Balance Outer Loop Controller Parameters
pid_balance_outerloop_reference = 0.0  # error = Reference - feedback_value. In Simulink error is directly lateral error so we set Reference = zero and feeback_value = - lateral_error.
# pid_balance_outerloop_P = 1.0
pid_balance_outerloop_P = 0.57
pid_balance_outerloop_I = 0.0
pid_balance_outerloop_D = 0.0
pid_balance_outerloop_sample_time = 1.0 / CONTROLLER_FREQUENCY

# STEERING ANGLE CONTROL
pid_steeringangle_P = 20.0
pid_steeringangle_I = 0.0
pid_steeringangle_D = 0.0
pid_steeringangle_sample_time = 1.0 / CONTROLLER_FREQUENCY



b = 1.095
h = 0.5195
c = 0.06
a = 0.4964
g = 9.81
m = 45
J = 0.27
num11 = h*a
num12 = h*a * b * g
num13 = - b * h**2
den11 = a**2 * g
den12 = -h
num21 = h*a**2
num22 = b * h**2
num23 = - a * b * h**2
den21 = -a**2 * g
den22 = h
# x1 = (num11 * v**2 * delta + num12 * phi + num13 *v * Phi_dot)/(den11 * v + den12 * v**3)
# x2 = (num21 * v * delta + num22 * v * phi - num23 * Phi_dot)/(den21 * v + den22 * v**3)
# x1 = h*(a* v**2 *u + a * b * g * phi - b * h * v * Phi_dot)/(a**2 * g * v - h * v**3)
# x2 = h*(a**2 * v * u + b * h * (v * phi - a * Phi_dot) / (- a **2 * g * v + h * v**3))

# State-space matrices of the bike
# Without front wheel inertia
# A_bike = np.matrix([[0, g/h],[1, 0]])
# B_bike = np.matrix([[1],[0]])
# # C_bike = C_bike_V0 + C_bike_V1 * V + C_bike_V * V**2
# C_bike_V0 = np.matrix([[0, 0]])
# C_bike_V1 = np.matrix([[a/(b*h), 0]])
# C_bike_V2 = np.matrix([[0, 1/(b*h)]])
# D_bike = np.matrix([[0]])

# WIth front wheel inertia
A_bike = np.matrix([[0, g/h],[1, 0]])
B_bike = np.matrix([[1],[0]])
# C_bike = C_bike_V0 + C_bike_V1 * V + C_bike_V * V**2
C_bike_V0 = np.matrix([[0, g*J/(m*h**3)]])
C_bike_V1 = np.matrix([[a/(b*h), 0]])
C_bike_V2 = np.matrix([[0, 1/(b*h)]])
D_bike = np.matrix([[J/(m*h**2)]])


# smith_delay = 2
L = np.array([8.0886, 25.7510])
A = np.array([[1.0151, 0.7592], [0.0402, 1.0151]])
B = np.array([[0.0402], [0.0008]])
D_param = 0
smith_delay = 0
# D_param = 0.6
#smith_delay = 1
# smith_x = np.zeros([2, smith_delay])
# smith_delta = np.zeros([1, smith_delay*2])
W = np.zeros([2, smith_delay])
for i in range(1, smith_delay+1):
    W[0:2, i-1:i] = np.dot(np.linalg.matrix_power(A, (smith_delay-i)),  B)
lw = L.dot(W)
LAd = L.dot(np.linalg.matrix_power(A,smith_delay))
print "Matrice calculation done"
print "W", W, '\t','lw', lw, 'LAd', LAd

Qparam = [10, -9, -9, 10]
Rparam = 1