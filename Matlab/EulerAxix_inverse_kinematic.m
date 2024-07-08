%% Inverse kinematic, Knowing the final position / Transformation Matrix, Calculate the Joint parameter;
clear all; % Clear all variables
close all; % Close all figures
clc; % Clear screen

%% Defination of Transformation Matrix
T_Matrix = [
       -0.2803   -0.0053    0.9599    0.2991;
       -0.0024    1.0000    0.0048    0.5026;
       -0.9599   -0.0010   -0.2803    0.4983;
         0          0          0      1.0000];

%% Rotation Calculate
% Extraction rotation matrix
R = T_Matrix(1:3, 1:3);

% Calculate the components of a quaternion
qw = sqrt(1 + R(1,1) + R(2,2) + R(3,3)) / 2;
qx = (R(3,2) - R(2,3)) / (4 * qw);
qy = (R(1,3) - R(3,1)) / (4 * qw);
qz = (R(2,1) - R(1,2)) / (4 * qw);

quat = [qw, qx, qy, qz];
qw = quat(1);
qx = quat(2);
qy = quat(3);
qz = quat(4);

% 打印四元数
disp('Quaternion:');
disp(['W = ' num2str(qw)]);
disp(['x = ' num2str(qx)]);
disp(['y = ' num2str(qy)]);
disp(['z = ' num2str(qz)]);

%% 
