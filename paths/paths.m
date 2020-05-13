% Define the time vector for the simulation
% Remove 10s from sim_time to give time to the bike to balance
time_array = 0:Ts:(sim_time-time_balance);
length1 = floor((sim_time-time_balance)/Ts);

switch path    
    case 1
        % Straight Path ===================================================
        path_x = v*time_array;
        path_y = 0*path_x; 
    
    case 2 
        % Circle ==========================================================
        path_x = radius * sin(time_array * v / radius);
        path_y = radius * cos(time_array * v / radius ) - radius;
        
    case 3
        % _/-\_ ===========================================================
        path_x = v*time_array;
        path_y = 0*path_x;
        path_y(floor(length1/5)+1:floor(2*length1/5)) = slope*Ts*(0:(floor(length1/5)-1));
        path_y(floor(2*length1/5)+1:floor(3*length1/5)) = path_y(floor(2*length1/5));
        path_y(floor(3*length1/5)+1:floor(4*length1/5)) = path_y(floor(3*length1/5)) - slope*Ts*(0:(floor(length1/5)-1));
        
    case 4
        % Sinusoidal Path =================================================
        path_x = v*time_array;
%         path_y = sin(path_x*1);
        path_y = 0.1*sin(path_x/30);
        
    case 5
        % _/-\_ integrated from heading ===================================
        heading_path = 0*time_array;
        heading_path(floor(length1/5)+1:floor(2*length1/5)) = atan(slope);
        heading_path(floor(3*length1/5)+1:floor(4*length1/5)) = -atan(slope);
        
        path_x = cumtrapz(time_array,v*cos(heading_path));
        path_y = cumtrapz(time_array,v*sin(heading_path));
        
    case 6 
        % Forward turn and back ==========================================================
        path_x1 = v*time_array(1:floor(length1/3));
        path_y1 = 0*path_x1;
        path_x3 = fliplr(path_x1);
        path_x3(1) = [];
        path_y3 = -2*radius*ones(size(path_x3));
        path_x4 = path_x3-(path_x3(1)-path_x3(end));
        path_x4(1) = [];
        path_y4 = -2*radius*ones(size(path_x4));
        
        path_x2 = radius * sin(2*pi*time_array(1:floor(length1/3)) / time_array(floor(length1/3)) / 2) + path_x1(end);
        path_y2 = radius * cos(2*pi*time_array(1:floor(length1/3)) / time_array(floor(length1/3)) / 2) - radius;
        path_x2(1) = [];
        path_y2(1) = [];
%         angle2 = linspace(0,pi,floor(length1/3));
%         path_x2 = radius * sin(angle2 * v / radius)+(2*radius);
%         path_x2(1) = [];
%         path_y2 = radius * cos(angle2 * v / radius)-radius;        
%         path_y2(1) = [];
        
%         path_x1 = linspace(0,2*radius,floor(length1/3));
%         path_y1 = 0*path_x1;
%         path_x3 = linspace(2*radius,0,floor(length1/3));
%         path_x3(1) = [];
%         path_y3 = -2*radius*ones(size(path_x3));   
% 
%         angle2 = linspace(0,pi,floor(length1/3));
%         path_x2 = radius * sin(angle2 * v / radius)+(2*radius);
%         path_x2(1) = [];
%         path_y2 = radius * cos(angle2 * v / radius)-radius;        
%         path_y2(1) = [];
        
        path_x = [path_x1 path_x2 path_x3 path_x4];
        path_y = [path_y1 path_y2 path_y3 path_y4];
        
    case 7
        % Step in y axis ===================================
        path_x = v*time_array;
        path_y = 0.15*(time_array>50);
end

% Add a time of time_balance of going straight at start of path to give time to the bike to balance
path_x = [v*(0:Ts:(time_balance-Ts)) time_balance*v+path_x];
path_y = [0*(0:Ts:(time_balance-Ts)) path_y];

path_x = path_x';
path_y = path_y';
% path_y = movmean(path_y,200);