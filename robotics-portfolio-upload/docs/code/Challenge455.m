% ZackGald
% EGR 455 – Background Subtraction and Robotic Control
clear all
close all
clc

%% Initialize Camera
vid = videoinput('winvideo', 1, 'YUY2_640x480');
set(vid, 'ReturnedColorSpace', 'rgb');
preview(vid);
pause(3.0);
background = rgb2gray(getsnapshot(vid));

%% SnapShot
waitfor(msgbox('Place an object in the field of view, then click OK'));
frame = rgb2gray(getsnapshot(vid));
foreground = background - frame;
foreground = (foreground > 100); % Threshol60d to get rid of noise

%% Calculate Object Location
% Column (x-direction)
columnsum = sum(foreground);
column_number = 1:640;
column_mult = columnsum .* column_number;
column_total = sum(column_mult);
column_location = column_total / sum(sum(foreground));

% Row (y-direction)
rowsum = sum(foreground, 2);
row_number = 1:480;
row_mult = rowsum' .* row_number;
row_total = sum(row_mult);
row_location = row_total / sum(sum(foreground));

%% Display Images and Object Location
figure(1);
imshow(background);
title('Background');

figure(2);
imshow(frame);
title('Frame');

figure(3);
imshow(foreground);
hold on;
plot(column_location, row_location, 'xr', 'MarkerSize', 10);
title('Captured Object’s Center');
hold off;

%% Convert Pixel Coordinates to Physical Coordinates
% Calibration factors
pixel_to_mm_x = 21.6/640; % Example conversion factor
pixel_to_mm_y = 21.6/640; % Example conversion factor

% Physical coordinates (in mm)
 physical_x = (column_location) * pixel_to_mm_x; % Adjust for center
 physical_y = (row_location) * pixel_to_mm_y;   % Invert y-axis
 physical_z = 0; % Assume planar surface

%% Homogeneous Transformation
% Rotation angles
Theta_Z = (-2 * pi) / 180;    % Rotation about Z-axis in radians
Theta_Y = (pi);  % Rotation about Y-axis in radians

% Rotation matrices
R_z = [cos(Theta_Z), -sin(Theta_Z), 0;
       sin(Theta_Z), cos(Theta_Z), 0;
       0, 0, 1];
R_y = [cos(Theta_Y), 0, sin(Theta_Y);
       0, 1, 0;
       -sin(Theta_Y), 0, cos(Theta_Y)];
R0_C = R_z * R_y;

% Translation vector
D0_C = [12; .6; 0]; % Translation values in mm

% Homogeneous transformation matrix
H0_C = [[R0_C, D0_C]; 0, 0, 0, 1];

% Object coordinates in camera frame
PC = [physical_x; physical_y; 0; 1];

% Transform to base frame
P0 = H0_C * PC;
X_BaseFrame = P0(1);
Y_BaseFrame = P0(2);

% Apply error adjustments and offsets
X_Error = 0; %56.0; % Error adjustment (cm)
Y_Error = 0; %3.5;
X_Offset = 0; % Offset correction (cm)
Y_Offset = 0;%10; %1.0;

% Final coordinates in base frame
X_BaseFrame = (X_BaseFrame + X_Error + X_Offset); % Convert to cm
Y_BaseFrame = (Y_BaseFrame + Y_Error + Y_Offset);

disp(['Object "X" location (cm): ', num2str(X_BaseFrame)]);
disp(['Object "Y" location (cm): ', num2str(Y_BaseFrame)]);


%% Robotic Arm Inverse Kinematics
% link1 = 550; % Length of link 1 in mm
% link2 = 700; % Length of link 2 in mm
% 
% % Distance to target
% r = sqrt(physical_x^2 + physical_y^2);
% phi = atan2(physical_y, physical_x);
% 
% % % Check if target is reachable
% % if r > (link1 + link2) || r < abs(link1 - link2)
% %     error('Target position is out of the robotic arm workspace.');
% % end
% 
% % Elbow angle (theta2)
% cos_Theta2 = (r^2 - link1^2 - link2^2) / (2 * link1 * link2);
% Theta2 = acos(cos_Theta2);
% 
% % Shoulder angle (theta1)
% sin_Theta2 = sqrt(1 - cos_Theta2^2);
% kx = link1 + link2 * cos_Theta2;
% ky = link2 * sin_Theta2;
% Theta1 = phi - atan2(ky, kx);
% 
% % Convert to degrees
% Theta1_deg = rad2deg(Theta1);
% Theta2_deg = rad2deg(Theta2);
% 
% disp(['Theta1 (deg): ', num2str(Theta1_deg)]);
% disp(['Theta2 (deg): ', num2str(Theta2_deg)]);

%% Send Joint Variables Over UART

SerPsoC = serialport('COM5', 9600, 'Parity', 'none', 'StopBits', 1, 'DataBits', 8);
X_Afteroff = X_BaseFrame + 20;
Xwhole = fix(X_Afteroff);
Xdec = (X_Afteroff - Xwhole)*100;
Ywhole = fix(Y_BaseFrame);
Ydec = (Y_BaseFrame-Ywhole)*100;


while(1)

    write(SerPsoC, Xwhole, 'uint8');
    pause(3);
    write(SerPsoC, Xdec, 'uint8');
    pause(3);
    write(SerPsoC, Ywhole, 'uint8');
    pause(3);
    write(SerPsoC, Ydec, 'uint8');
    disp('Variables sent successfully!');
    pause(8);
   

end
%%
clear SerPSoC;


%% End
disp('Program completed!');
