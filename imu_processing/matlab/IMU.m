clear all
close all
clc
%% Import data

filepath = fullfile('C:\Users\jingy\OneDrive\×ÀÃæ\2019-04-08-18-30-42.bag');
bag = rosbag(filepath);

imu_message = select(bag,'MessageType','sensor_msgs/Imu');
data = readMessages(imu_message);
num_data=size(data,1);


for i = 1:num_data
    Sec(i) = data{i,1}.Header.Stamp.Sec;
    Nsec(i) = data{i,1}.Header.Stamp.Nsec;
end

orientation = zeros(num_data,4);
for i = 1:num_data
    orientation(i,1) = data{i,1}.Orientation.X;
    orientation(i,2) = data{i,1}.Orientation.Y;
    orientation(i,3) = data{i,1}.Orientation.Z;
    orientation(i,4) = data{i,1}.Orientation.W;
end

for i = 1:num_data
    X(i) = data{i,1}.Orientation.X;
    Y(i) = data{i,1}.Orientation.Y;
    Z(i) = data{i,1}.Orientation.Z;
    W(i) = data{i,1}.Orientation.W;
end

angularvelocity = zeros(num_data,3);
for i = 1:num_data
    angularvelocity(i,1) = data{i,1}.AngularVelocity.X;
    angularvelocity(i,2) = data{i,1}.AngularVelocity.Y;
    angularvelocity(i,3) = data{i,1}.AngularVelocity.Z;
end

for i = 1:num_data
    GyroX(i) = data{i,1}.AngularVelocity.X;
    GyroY(i) = data{i,1}.AngularVelocity.Y;
    GyroZ(i) = data{i,1}.AngularVelocity.Z;
end

linearacceleration = zeros(num_data,3);
for i = 1:num_data
    linearacceleration(i,1) = data{i,1}.LinearAcceleration.X;
    linearacceleration(i,2) = data{i,1}.LinearAcceleration.Y;
    linearacceleration(i,3) = data{i,1}.LinearAcceleration.Z;
end

for i = 1:num_data
    AccX(i) = data{i,1}.LinearAcceleration.X;
    AccY(i) = data{i,1}.LinearAcceleration.Y;
    AccZ(i) = data{i,1}.LinearAcceleration.Z;
end

for i = 1:num_data
    roll(i) = atan2(2*(W(i)*X(i)+Y(i)*Z(i)),(1-2*((X(i)^2)*(Y(i)^2))));
    yaw(i) = atan2(2*(X(i)*Y(i)+W(i)*Z(i)),(W(i)^2-Z(i)^2-Y(i)^2+X(i)^2));
    pitch(i) = atan2(2*(W(i)*Z(i)+X(i)*Y(i)),(1-2*((Y(i)^2)*(Z(i)^2))));
    yaw_angle(i) = yaw(i)*180/pi;
end


% Compute accelerometer magnitude
acc_mag = sqrt(AccX.*AccX + AccY.*AccY + AccZ.*AccZ);

% HP filter accelerometer data
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(40), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(40), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% Threshold detection
stationary = acc_magFilt < 0.9;

%% Plot data raw sensor data and stationary periods

figure;
ax(1) = subplot(2,1,1);
    hold on;
    plot([1:num_data], GyroX, 'r');
    plot([1:num_data], GyroY, 'g');
    plot([1:num_data], GyroZ, 'b');
    title('Gyroscope');
    xlabel('Time (s)');
    ylabel('Angular velocity (^\circ/s)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(2,1,2);
    hold on;
    plot([1:num_data], AccX, 'r');
    plot([1:num_data], AccY, 'g');
    plot([1:num_data], AccZ, 'b');
    plot([1:num_data], acc_magFilt, ':k');
    plot([1:num_data], stationary, 'k', 'LineWidth', 2);
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    hold off;
linkaxes(ax,'x');

%% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([AccX AccY AccZ], quaternConj(orientation));

% Plot translational accelerations
figure;
hold on;
plot([1:num_data], acc(:,1), 'r');
plot([1:num_data], acc(:,2), 'g');
plot([1:num_data], acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

%% Compute translational velocities
%set accel in z-dir to be 0
acc(:,3) = 0;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * (1/40);
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end

for i = num_data-50:num_data;
    vel(i,1) = 0;
    vel(i,2) = 0;
    vel(i,3) = 0;
end

% Plot translational velocity
figure;
hold on;
plot([1:num_data], vel(:,1), 'r');
plot([1:num_data], vel(:,2), 'g');
plot([1:num_data], vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

%%  Compute translational position

for i = 1:length(vel)
    vel(i,1) = -sin(yaw(i)) * vel(i,1);
end

% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * (1/40);    % integrate velocity to yield position
end

pos(:,1) = -pos(:,1);
pos(:,2) = pos(:,2);
pos(:,3) = -pos(:,3)/2;

% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot([1:num_data], pos(:,1), 'r');
plot([1:num_data], pos(:,2), 'g');
plot([1:num_data], pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;

%% Plot 3D foot trajectory

posPlot = pos;
quatPlot = orientation;

% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 30;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 60*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', (40 / SamplePlotFreq));
 

 