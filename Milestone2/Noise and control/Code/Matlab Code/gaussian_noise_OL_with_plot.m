% Specify the file name
filename = '/home/eslam/catkin_workspace/src/t2/scripts/line_with_turn.xlsx';
newFilename = '/home/eslam/catkin_workspace/src/t2/scripts/NoisyOdom3.csv';

% Specify the header for the column
header = {'Noisy_X', 'Noisy_Y', 'Time'};

% Read the data from the CSV file
data = readtable(filename);
Noise_data = readtable(newFilename);

% Access data by column name
columnData_X = data.positions_X; 
columnData_X_clean = columnData_X(~isnan(columnData_X));

columnData_Y = data.positions_y; 
columnData_Y_clean = columnData_Y(~isnan(columnData_Y));

%columnData_yaw = data.Yaw;

columnData_time = data.Time_Sec;
columnData_time_clean = columnData_time(~isnan(columnData_time));


columnData_NX = Noise_data.Noisy_X;
columnData_NY = Noise_data.Noisy_Y;
%columnData_NYaw = Noise_data.Noisy_Yaw;

xPointsM= awgn(columnData_X_clean, 43.428, "measured");     % old = 0.0313
yPointsM= awgn(columnData_Y_clean, 46.3, "measured");       % old = 46.3
%yawPointsM= awgn(columnData_yaw,0.075,"measured");



plot(columnData_X_clean,columnData_Y_clean , '-b');
xlabel('X');
ylabel('Y');
grid on;
hold on;

plot(xPointsM,yPointsM , '-r');
xlabel('X');
ylabel('Y');
grid on;
hold on;

% Combine the header and the data
dataWithHeader = [header; num2cell(xPointsM), num2cell(yPointsM), num2cell(columnData_time_clean)];

% Write the data with header to a new CSV file
writecell(dataWithHeader, newFilename);