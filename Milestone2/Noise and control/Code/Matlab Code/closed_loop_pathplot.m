clear all;

filename = '/home/eslam/catkin_workspace/src/t2/scripts/Dimensions2_4.csv';
data = readtable(filename);

columnData_X = data.positions_x; 
columnData_X_clean = columnData_X(~isnan(columnData_X));

columnData_Y = data.positions_y; 
columnData_Y_clean = columnData_Y(~isnan(columnData_Y));

columnData_YAW = data.yaw; 
columnData_YAW_clean = columnData_YAW(~isnan(columnData_YAW));

columnData_X_noised = data.Noisy_x; 
columnData_X_noised_clean = columnData_X_noised(~isnan(columnData_X_noised));

columnData_Y_noised = data.Noisy_y; 
columnData_Y_noised_clean = columnData_Y_noised(~isnan(columnData_Y_noised));

columnData_YAW_noised = data.Noisy_yaw; 
columnData_YAW_noised_clean = columnData_YAW_noised(~isnan(columnData_YAW_noised));


columnData_X_filtered = data.filtered_x; 
columnData_X_filtered_clean = columnData_X_filtered(~isnan(columnData_X_filtered));

columnData_Y_filtered = data.filtered_y; 
columnData_Y_filtered_clean = columnData_Y_filtered(~isnan(columnData_Y_filtered));

columnData_YAW_filtered = data.filtered_yaw; 
columnData_YAW_filtered_clean = columnData_YAW_filtered(~isnan(columnData_YAW_filtered));

columnData_time = data.Time_Sec;
columnData_time_clean = columnData_time(~isnan(columnData_time));

%true value plot
plot(columnData_X_clean,columnData_Y_clean , '-b');
xlabel('X');
ylabel('Y');
grid on;
hold on;

%noise plot
plot(columnData_X_noised_clean,columnData_Y_noised_clean, '-r');
hold on;

%filter plot
plot(columnData_X_filtered_clean,columnData_Y_filtered_clean, '-g');
hold on;