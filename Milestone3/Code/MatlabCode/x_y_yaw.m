clear;
clc;

% Specify the file name
filename = '/home/eslam/catkin_workspace/src/t2/scripts/Dimensions33.csv';

% Read the data from the CSV file
data = readtable(filename);

columnData_X = data.positions_x_odom; 
columnData_X_clean = columnData_X(~isnan(columnData_X));

columnData_Y = data.positions_y_odom; 
columnData_Y_clean = columnData_Y(~isnan(columnData_Y));

columnData_yaw = data.yaw_odom;
columnData_yaw_clean = columnData_yaw(~isnan(columnData_yaw));

columnData_time = data.Time_Sec;
columnData_time_clean = columnData_time(~isnan(columnData_time));

data_rms_x = data.rms_x_odom;
data_rms_x_clean = data_rms_x(~isnan(data_rms_x));

data_rms_y = data.rms_y_odom;
data_rms_y_clean = data_rms_y(~isnan(data_rms_y));

data_rms_yaw = data.rms_yaw_odom;
data_rms_yaw_clean = data_rms_yaw(~isnan(data_rms_yaw));

xPointsM= awgn(columnData_X_clean, 30.78, "measured");     % old = 0.0313
yPointsM= awgn(columnData_Y_clean, 25.9, "measured");       % old = 46.3
yawPointsM= awgn(columnData_yaw_clean,0.075,"measured");



% Apply Noise to x y yaw and store it in a New CSV called NoisyOdom.csv for line path------------------

A = [ 1 0 0; 
      0 1 0;
      0 0 1];   

B = [0 1 1;
     0 1 1;
     1 0 0;]; 

H = [1;
     0;
     0];          % Observation matrix

Q = 0.629*eye(3);     % Process noise covariance to be applied to the whole matrix [2 x 2]

R = 10*eye(3);           % Measurement noise covariance old 0.1

x = [0; 
     0;
     0];         % Initial state estimate for x, y directions and yaw.

P = eye(3);         % Initial error covariance to be large number


columnData_YN = yPointsM;
filtered_yPoints = zeros(size(columnData_YN));
for i = 1:length(columnData_YN)
    % Predictionstep
    x = A.*x;                        % Predicted state estimate
    P = A.*P.*A' + Q;              % Predicted error covariance
    % P = A.*P. *A' + Q;               % Predicted error covariance
    
    % Update step
    K = P.*H'/((H.*P).*H' + R);          % Kalman gain
    % K = eye(2).*H'/(H.*eye(2).*H' + R);  % Kalman gain (using identity matrix for P)

    x = x + K.*(columnData_YN(i) - H.*x); % Updated state estimate
    P = P - K.*(H.*P);           % Updated error covariance  (use TIMES (.) for elementwise multiplication.)
    
    filtered_yPoints(i) = x(1);    % Store filtered position
end

columnData_XN = xPointsM;
filtered_xPoints = zeros(size(columnData_XN));
for i = 1:length(columnData_XN)
    % Predictionstep
    x = A.*x;                        % Predicted state estimate
    P = A.*P.*A' + Q;              % Predicted error covariance
    % P = A.*P. *A' + Q;               % Predicted error covariance
    
    % Update step
    K = P.*H'/((H.*P).*H' + R);          % Kalman gain
    % K = eye(2).*H'/(H.*eye(2).*H' + R);  % Kalman gain (using identity matrix for P)

    x = x + K.*(columnData_XN(i) - H.*x); % Updated state estimate
    P = P - K.*(H.*P);           % Updated error covariance  (use TIMES (.) for elementwise multiplication.)
    
    filtered_xPoints(i) = x(1);    % Store filtered position
end

columnData_YawN = yawPointsM;
filtered_yawPoints = zeros(size(columnData_YawN));
for i = 1:length(columnData_YawN)
    % Predictionstep
    x = A.*x;                        % Predicted state estimate
    P = A.*P.*A' + Q;              % Predicted error covariance
    % P = A.*P. *A' + Q;               % Predicted error covariance
    
    % Update step
    K = P.*H'/((H.*P).*H' + R);          % Kalman gain
    % K = eye(2).*H'/(H.*eye(2).*H' + R);  % Kalman gain (using identity matrix for P)

    x = x + K.*(columnData_YawN(i) - H.*x); % Updated state estimate
    P = P - K.*(H.*P);           % Updated error covariance  (use TIMES (.) for elementwise multiplication.)
    
    filtered_yawPoints(i) = x(1);    % Store filtered position
end


%{
%true value plot
plot(columnData_X_clean,columnData_Y_clean , '-b');
xlabel('odom_x');
ylabel('odom_y');
grid on;
hold on;
%}

%noise plot
plot(xPointsM,yPointsM, '-r');
xlabel('noised x');
ylabel('noised y');
grid on;
hold on;

%filter plot
plot(filtered_xPoints,filtered_yPoints, '-g');
xlabel('filtered x');
ylabel('filtered y');
grid on;
hold on;

legend ('noisy signal','filtered signal')

overall_RMS_x = mean(data_rms_x_clean)
overall_RMS_y = mean(data_rms_y_clean)
overall_RMS_yaw = mean(data_rms_yaw_clean)


new_data = table(columnData_X_clean, columnData_Y_clean, columnData_yaw_clean, xPointsM, yPointsM, yawPointsM, filtered_xPoints, filtered_yPoints, filtered_yawPoints, ...
                 'VariableNames', {'x_odom', 'y_odom', 'yaw_odom', 'x_noisy', 'y_noisy', 'yaw_noisy', 'x_filtered', 'y_filtered', 'yaw_filtered'});

new_file = '/home/eslam/catkin_workspace/src/t2/scripts/change_lane_path_odom_noise_filter.csv';

writetable(new_data, new_file);