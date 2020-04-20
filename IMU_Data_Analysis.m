if ~exist('imudata')
    imudata = readtable('IMUData-20190730-082201.csv');
end
max_T = max(imudata.temp);
min_T = min(imudata.temp);
step_size = 0.5;
total_length = ceil((max_T-min_T+2e-4)/step_size) ;
real_increment_step = (max_T-min_T+2e-4)/total_length;
temprange = min_T-1e-4:real_increment_step:max_T+1e-4;
imu_T = cell(length(temprange)-1,1);
for i = 1:size(temprange,2)
    imu_T{i}.Nrrows = 0;
end
for i = 1:size(imudata,1)
    indx = ceil((imudata.temp(i) - min_T + 1e-4)/real_increment_step);
        len_current_table = imu_T{indx}.Nrrows;
    for j = 1:size(imudata.Properties.VariableNames,2)
        imu_T{indx}.(imudata.Properties.VariableNames{j})(len_current_table+1) = ...
            imudata.(imudata.Properties.VariableNames{j})(i);
        imu_T{indx}.Nrrows = len_current_table+1;
    end
end
    
    