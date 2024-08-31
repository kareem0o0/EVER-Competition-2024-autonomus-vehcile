% Define the file paths
csvFilePath = '/home/eslam/catkin_workspace/src/t2/scripts/Dimensions33.csv';
saveDir = '/home/eslam/catkin_workspace/src/t2/scripts/Graphs3';

% Create the directory if it doesn't exist
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

% Read the CSV file
data = readtable(csvFilePath);

% Extract data
time = data.Time_Sec;
positions_x_odom = data.positions_x_odom;
positions_y_odom = data.positions_y_odom;
yaw_odom = data.yaw_odom;
velocity = data.velocity;
acceleration = data.acceleration;
rms_x_odom = data.rms_x_odom;
rms_y_odom = data.rms_y_odom;
rms_yaw_odom = data.rms_yaw_odom;
velocity_cmd = data.velocity_cmd;
steering_angle = data.steering_angle;

% Function to save plots
savePlot = @(fig, name) saveas(fig, fullfile(saveDir, name), 'png');

% Plot and save graph for x and y that out from odom
fig = figure;
plot(positions_x_odom, positions_y_odom);
title('X and Y from Odometry');
xlabel('X from Odometry');
ylabel('Y from Odometry');
grid on;
savePlot(fig, 'xy_odom.png');
close(fig);

% Plot and save graph for x out from odom with time
fig = figure;
plot(time, positions_x_odom);
title('X from Odometry over Time');
xlabel('Time (s)');
ylabel('X from Odometry');
grid on;
savePlot(fig, 'x_odom_time.png');
close(fig);

% Plot and save graph for y out from odom with time
fig = figure;
plot(time, positions_y_odom);
title('Y from Odometry over Time');
xlabel('Time (s)');
ylabel('Y from Odometry');
grid on;
savePlot(fig, 'y_odom_time.png');
close(fig);

% Plot and save graph for yaw out from odom with time
fig = figure;
plot(time, yaw_odom);
title('Yaw from Odometry over Time');
xlabel('Time (s)');
ylabel('Yaw from Odometry');
grid on;
savePlot(fig, 'yaw_odom_time.png');
close(fig);

% Plot and save graph for velocity with time
fig = figure;
plot(time, velocity);
title('Velocity over Time');
xlabel('Time (s)');
ylabel('Velocity');
grid on;
savePlot(fig, 'velocity_time.png');
close(fig);

% Plot and save graph for acceleration with time
fig = figure;
plot(time, acceleration);
title('Acceleration over Time');
xlabel('Time (s)');
ylabel('Acceleration');
grid on;
savePlot(fig, 'acceleration_time.png');
close(fig);

% Plot and save graph for RMS_x with time
fig = figure;
plot(time, rms_x_odom);
title('RMS X from Odometry over Time');
xlabel('Time (s)');
ylabel('RMS X from Odometry');
grid on;
savePlot(fig, 'rms_x_odom_time.png');
close(fig);

% Plot and save graph for RMS_y with time
fig = figure;
plot(time, rms_y_odom);
title('RMS Y from Odometry over Time');
xlabel('Time (s)');
ylabel('RMS Y from Odometry');
grid on;
savePlot(fig, 'rms_y_odom_time.png');
close(fig);

% Plot and save graph for RMS_yaw with time
fig = figure;
plot(time, rms_yaw_odom);
title('RMS Yaw from Odometry over Time');
xlabel('Time (s)');
ylabel('RMS Yaw from Odometry');
grid on;
savePlot(fig, 'rms_yaw_odom_time.png');
close(fig);

% Plot and save graph for velocity_cmd with time
fig = figure;
plot(time, velocity_cmd);
title('Commanded Velocity over Time');
xlabel('Time (s)');
ylabel('Commanded Velocity');
grid on;
savePlot(fig, 'velocity_cmd_time.png');
close(fig);

% Plot and save graph for steering_angle with time
fig = figure;
plot(time, steering_angle);
title('Steering Angle over Time');
xlabel('Time (s)');
ylabel('Steering Angle');
grid on;
savePlot(fig, 'steering_angle_time.png');
close(fig);
