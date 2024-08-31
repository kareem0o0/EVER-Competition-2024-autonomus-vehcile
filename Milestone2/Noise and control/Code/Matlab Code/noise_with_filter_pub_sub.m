clear all;
clc;

rosshutdown
rosinit

[startpub, msg4] = rospublisher("/startSimulation", "std_msgs/Bool");
msg4.Data = 1;
send(startpub, msg4);

[odomPub1, msg1] = rospublisher("/odom1", "std_msgs/Float32MultiArray");
[odomPub2, msg2] = rospublisher("/odom2", "std_msgs/Float32MultiArray");
[odomPub3, msg3] = rospublisher("/odom3", "std_msgs/Float32MultiArray");



A = [ 1 0 0; 
      0 1 0;
      0 0 1];   

B = [0 1 1;
     0 1 1;
     1 0 0;]; 

H = [1;
     1;
     1];          % Observation matrix

Q = 0.629*eye(3);     % Process noise covariance to be applied to the whole matrix

R = 0.9*eye(3);           % Measurement noise covariance  old 0.1

x = [0; 
     0;
     0];         % Initial state estimate for x, y directions and yaw.

P = eye(3);         % Initial error covariance to be large number



% quat to euler

function yaw_deg = quat2yaw(quat)
    
    q_x = quat(1);
    q_y = quat(2);
    q_z = quat(3);
    q_w = quat(4);
    
    yaw_rad = atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y^2 + q_z^2));
    yaw_deg = rad2deg(yaw_rad);
end


while 1

    odomSub = rossubscriber("/odom","DataFormat","struct");
    odomMsg = receive(odomSub,10);
    pose = odomMsg.Pose.Pose;
    vel_x = odomMsg.Twist.Twist.Linear.X;
    vel_y = odomMsg.Twist.Twist.Linear.Y;
    
    xposd = pose.Position.X;
    yposd = pose.Position.Y;
    orientation_x = pose.Orientation.X;
    orientation_y = pose.Orientation.Y;
    orientation_z = pose.Orientation.Z;
    orientation_w = pose.Orientation.W;

    Yaw = quat2yaw([orientation_x, orientation_y, orientation_z, orientation_w]);

    Data_X = xposd;
    Data_Y = yposd;
    Data_yaw = Yaw; 
    
    xPointsM= awgn(Data_X, 43.428,"measured");  % old 0.0313 with 43.428 dB
    yPointsM= awgn(Data_Y, 25.9,"measured");
    yawPointsM= awgn(Data_yaw, 25.9,"measured");   %old 0.075 with 25.9 dB

    Data_XN = xPointsM;
    Data_YN = yPointsM;
    Data_yawN = yawPointsM;

    % Predictionstep
    x = A.*x ;                     % x = A.*x + B.*U;        
    P = A.*P.*A' + Q;              % Predicted error covariance


    % Update step
    K = P.*H'/((H.*P).*H' + R);          % Kalman gain
    
    % K = eye(2).*H'/(H.*eye(2).*H' + R);  % Kalman gain (using identity matrix for P)

    x = x + K.*([Data_XN;Data_YN;Data_yawN] - H.*x); % Updated state estimate
    P = P - K.*(H.*P);           % Updated error covariance  (use TIMES (.) for elementwise multiplication.)
    
    filtered_xPoints = x(1);    % Store filtered x
    filtered_yPoints = x(2,2);    % Store filtered y
    filtered_yawPoints = x(3,3);    % Store filtered yaw
    
   
    
    k1 = yposd
    k2 = yPointsM
    k3_1 = filtered_xPoints;
    k3_2 = filtered_yPoints
    k3_3 = filtered_yawPoints;

    msg1.Data = [k3_1, k3_2, k3_3];
    msg2.Data = [xposd, yposd, Yaw, xPointsM, yPointsM, yawPointsM, filtered_xPoints, filtered_yPoints, filtered_yawPoints];

    send(odomPub1, msg1);  %publishing for path
    send(odomPub2, msg2);  %publishing for CSV file
    
    pause(0.1);
    
end