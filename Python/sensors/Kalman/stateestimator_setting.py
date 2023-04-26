from sympy import symbols, lambdify, Matrix, N
from sympy import sqrt, sin, cos, tan, atan, atan2, sec
import numpy as np
from numpy import deg2rad, rad2deg
from .kalman_param import *


sig_digits = 2

# # # # # # # #
# Parameters  #
# # # # # # # #
Ts = symbols('Ts')
dv = symbols('dv')
# a = symbols('a')
# b = symbols('b')
# c = symbols('c')
# d = symbols('d')
# h = symbols('h')
# h_imu = symbols('h_imu')
# g = symbols('g')
# lambda_ = symbols('lambda_')
# r_o = symbols('r_o')
# N_rps = symbols('N_rps')
# R_earth = symbols('R_earth')
# alpha_0 = symbols('alpha_0')
# gamma_0 = symbols('gamma_0')

# dv = 0



Jxx = h**2
Jxz = -a*h


params = Matrix([
    dv,
])
# a,
# b,
# c,
# d,
# h,
# h_imu,
# g,
# lambda_,
# r_o,
# N_rps,
# R_earth,
# alpha_0,
# gamma_0,

# # # # # #
# States  #
# # # # # #
px = symbols('px')
py = symbols('py')
v = symbols('v')
psi = symbols('psi')
phi = symbols('phi')
delta = symbols('delta')
dphi = symbols('dphi')

x = Matrix([px, py, v, psi, phi, delta, dphi])



# # # # # #
# Control #
# # # # # #
ddelta = symbols('ddelta')

u = ddelta



# # # # #
# Misc  #
# # # # #
delta_eff = delta * sin(lambda_)
beta = atan(a/b * tan(delta_eff))
theta = psi + beta



# # # # # # # # # # # # #
# Discrete Motion Model #
# # # # # # # # # # # # #
fx = N(x + Ts * Matrix([
    v * cos(theta),     # x
    v * sin(theta),     # y
    dv,     # v
    (v * tan(delta_eff)) / b,       #Psi
    dphi,       #phi
    ddelta,     #delta
    g/h*phi + a*v*sin(lambda_)/(b*h)*ddelta + (v **2 *h - a*c*g)* sin(lambda_) /(b*h**2) * delta ,
]), sig_digits)
# ( g*h*sin(phi) + v**2 * h/b * tan(delta_eff) - g*a*c/b * delta_eff - v * Jxz/b * 1/cos(delta_eff)**2 * sin(lambda_) * ddelta ) / Jxx,
Fx = N(fx.jacobian(x), sig_digits)

f = lambdify((params, Ts, x, u), [fx, Fx], ['numpy', 'sympy'])
Q = np.diag([ 1, 1, 0.01, 0.01, deg2rad(0.01)*1, deg2rad(1)*2, deg2rad(1)*5 ]) ** 2



# # # # # # # # # # # # # # # # # # # # # # # #
# # Discrete Motion Model (No Roll Dynamics)  #
# # # # # # # # # # # # # # # # # # # # # # # #
# fx = x + Ts * Matrix([
#     v * cos(theta),
#     v * sin(theta),
#     dv,
#     ddelta,
#     0,
#     (v * tan(delta_eff)) / b,
#     0,
# ])
# Fx = fx.jacobian(x)
#
# f_no_roll = lambdify((params, Ts, x, u), [fx, Fx], 'numpy')



# # # # # # #
# IMU Model #
# # # # # # #
G = g/h_imu * phi + (h_imu*v**2 - g*a*c)*sin(lambda_)/(b*h_imu**2) * delta + a*v*sin(lambda_)/(b*h_imu) * ddelta
B = v**2 / b * tan(delta_eff)
    # * sqrt( b**2 + (d * tan(delta_eff) )**2 )

h_ = N(Matrix([
    dv/g,
    (-h_imu * G + B * cos(phi) + g*sin(phi))/g,
    (-B * sin(phi) + g*cos(phi))/g,
    dphi,
    v/b * tan(delta_eff) * sin(phi),
    v/b * tan(delta_eff) * cos(phi),
]), sig_digits)
H_ = N(h_.jacobian(x),sig_digits)

himu = lambdify((params, x, u), [h_, H_], ['numpy', 'math', 'sympy'])
# bimu = np.array([[ -0.37212, 0.19665, -0.042264, -1.7385e-05, -0.00017173, 0.00026568 ]]).T
# Rimu = np.diag([ 0.00015271, 0.00011635, 0.00017337, 0.0000063271, 0.0000038019, 0.0000027898])
# Rimu = np.diag([ 0.00015271, 0.00011635, 0.00017337, 0.0003271, 0.00038019, 0.00027898])
Rimu = np.diag([0.0062, 0.0062, 0.0175, 2e-4, 0.0035, 0.0022])

# # # # # # #
# ENC Model #
# # # # # # #
h_ = N(Matrix([
    delta
]), sig_digits)
H_ = N(h_.jacobian(x), sig_digits)

henc = lambdify((params, x, u), [h_, H_], ['numpy', 'math', 'sympy'])
# benc = -1.7473e-17
Renc = 9.1935e-10



# # # # # # #
# RPS Model #
# # # # # # #
h_ = N(Matrix([
    v
]), sig_digits)
H_ = N(h_.jacobian(x), sig_digits)

hrps = lambdify((params, x, u), [h_, H_], ['numpy', 'math', 'sympy'])
# crps = 1.083796867151537
# brps = 5.986e-15
Rrps = 1.8586e-05
#
# rps_vel_to_dt = lambdify((params, v), 2*np.pi*r_o / (N_rps * v + 1e-32), 'numpy')
#
# def rps_filter(Yrps, Yrps_prev):
#     if (np.isnan(Yrps)):
#         return (Yrps, Yrps_prev)
#     elif (np.isnan(Yrps_prev).any()):
#         Yrps_prev = Yrps * np.ones(len(Yrps_prev))
#     else:
#         Yrps_prev = np.roll(Yrps_prev, 1)
#         Yrps_prev[0] = Yrps
#
#     return (np.mean(Yrps_prev), Yrps_prev)


# # # # # # #
# GPS Model #
# # # # # # #
h_ = N(Matrix([
    (py / R_earth + alpha_0) * rad2deg(1),
    (px / (R_earth * cos(alpha_0)) + gamma_0) * rad2deg(1),
    v,
    theta
]), 6)
H_ = N(h_.jacobian(x), sig_digits)

hgps = lambdify((params, x, u), [h_, H_], ['numpy', 'math', 'sympy'])
# bgps = np.array([[ -5.6413e-16, 2.6814e-15, 0, 0 ]]).T
Rgps = np.diag([ 1.7572e-13, 1.2505e-13, 2.1567e-05, 9.5228e-05]) # Low version


# gps_y_to_lat = lambdify((params, py), py / R_earth + gamma_0, ['numpy', 'math', 'sympy'])
# gps_x_to_long = lambdify((params, px), px / (R_earth * cos(gamma_0)) + alpha_0, ['numpy', 'math', 'sympy'])
#
# def gps_snh(Ygps, Ygps_prev, dt):
#     if (np.isnan(Ygps[[0,1]]).any()):
#         return (Ygps, False)
#     elif (np.isnan(Ygps_prev[[0,1]]).any()):
#         return (Ygps, True)
#
#     lon_prev = Ygps_prev[0]
#     lat_prev = Ygps_prev[1]
#
#     lon = Ygps[0]
#     lat = Ygps[1]
#
#     dlon = lon - lon_prev
#     dlat = lat - lat_prev
#
#     v = np.sqrt(dlon**2 + dlat**2) / dt
#     theta = np.arctan2(dlat, dlon + 1e-32)
#
#     return (np.array([
#         lon,
#         lat,
#         v,
#         theta,
#     ]), True)



# # # # # # # # # # # # #
# Black Bike Parameters #
# # # # # # # # # # # # #
black_bike_parameters = lambdify((
    'dv = 0'
), params, 'numpy')

# 'a = 0.6687',
# 'b = 1.15',
# 'c = 0.06',
# 'd = 0.4',
# 'h = 0.534',
# 'h_imu = 0.215',
# 'g = 9.81',
# 'lambda_ = deg2rad(66)',
# 'r_o = 0.694 / 2',
# 'N_rps = 3',
# 'R_earth = 6357000',
# 'alpha_0 = deg2rad(57.7814)',
# 'gamma_0 = deg2rad(12.7735)',


#
# def Imu_model(param,in0,deltadot_sym):
#     from numpy import sin, cos, tan, reshape, mat,array
#     in1 = in0
#     dv = 0
#     delta_sym = in1[5,:]
#     phi_sym = in1[4,:]
#     phidot_sym = in1[6,:]
#     v_sym = in1[2,:]
#     t2 = cos(phi_sym)
#     t3 = sin(phi_sym)
#     t4 = v_sym**2
#     t5 = delta_sym*9.135454576426009e-1
#     t6 = tan(t5)
#     t7 = t6**2
#     t8 = t2*t6*v_sym*8.681309141418526e-1
#     t9 = t3 *t6 *v_sym *8.681309141418526e-1
#     t10 = t3 *t4 *t6 *8.849448666073931e-2
#     t11 = t2 *t4 *t6 *8.849448666073931e-2
#     t12 = -t10
#     h_symImu = mat([dv *(1.0e+2 / 9.81e+2) , -phi_sym+t3+t11-deltadot_sym *v_sym *5.40602064740626e-2-delta_sym *(t4 *3.688730480248411-6.752895752850453) *2.191641182466871e-2 , t2+t12 , phidot_sym , t9 , t8]).T
#
#     t13 = t7 *9.135454576426009e-1
#     t14 = t13+9.135454576426009e-1
#     H_symImu = mat(reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,deltadot_sym *(-5.40602064740626e-2)-delta_sym *v_sym *1.616874726306643e-1+t2 *t6 *v_sym *1.769889733214786e-1,t3 *t6 *v_sym *(-1.769889733214786e-1),0.0,t3 *t6 *8.681309141418526e-1,t2 *t6 *8.681309141418526e-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2+t12-1.0,-t3-t11,0.0,t8,-t9,0.0,t4 *(-8.084373631533214e-2)+t2 *t4 *t14 *8.849448666073931e-2+1.479992443285267e-1,t3 *t4 *t14 *(-8.849448666073931e-2),0.0,t3 *t14 *v_sym *8.681309141418526e-1,t2 *t14 *v_sym *8.681309141418526e-1,0.0,0.0,0.0,1.0,0.0,0.0],[6,7]))
#     return (h_symImu, H_symImu)

# himu = Imu_model
# Imu_model([],np.mat([1,1,1,1,1,1,1]).T,1)