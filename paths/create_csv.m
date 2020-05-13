clear all
close all
clc


%% Generate path
Ts = 1e-3; % [s] Sampling time
sim_time = 100; % [s] Total path duration
time_balance = 0; % [s] Time at start of path to let bike balance
v = 4; % [m/s] Bike forward speed

% Select path and its characteristics
radius = 50; % radius
slope = 1/1000; % slope
path = 4;    % 1: Straight Path   2: Circle    3: _/-\_
             % 4: Sinusoidal path    5: _/-\_ integrated from heading
             % 6: Forward turn and back   7: Step in y axis
paths;
path_time = Ts * ((1:length(path_x)) - 1)';
path_psi = [0 ; atan2(diff(path_y),diff(path_x))];

figure;plot(path_x,path_y)


%% Export to CSV
csv_name = 'sinusoidal_path.csv';
writematrix([path_time path_x path_y path_psi],csv_name);