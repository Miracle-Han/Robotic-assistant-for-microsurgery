%% 根据 DH Parameter 计算 Transformation matrix
clear all; % Clear all variables
close all; % Close all figures
clc; % Clear screen

% Symbolic joint angle variables
syms q1 q2 q3 q4 q5 q6 q7 real;
syms d0 d1 d2 d3 d4 d5 d6 d7 real;


%% DH parameter [alpha_i; a_i; d_i; theta_i]
% alpha_i: the angle from Zi-1 to Zi measured about Xi-1
% a_i: the distance from Zi-1 to Zi measured along Xi-1
% d_i: the distance from Xi-1 to Xi measured along Zi
% theta_i: the angle from Xi-1 to Xi measured about Zi

% d0 = 0;
% d1 = -(0.1564+0.1284);
% d2 = -(0.0054+0.0064);
% d3 = -(0.2104+0.2104);
% d4 = -(0.0064+0.0064);
% d5 = -(0.2084+0.1059);
% d6 = 0;
% d7 = -(0.1059+0.0615);
% 
% % 定义关节角度
% q1 = 1.245226;
% q2 = -1.014591;
% q3 = -1.626709;
% q4 = 1.306197;
% q5 = -0.040284;
% q6 = 0.686400;
% q7 = -1.036441;
% 
% pi = 3.14;

% defination of DH parameters
% [alpha_i; a_i; d_i; theta_i]
DH_params = [
    pi        0       d0    0;
    pi/2      0       d1    q1;
    pi/2      0       d2    q2 + pi;
    pi/2      0       d3    q3 + pi;
    pi/2      0       d4    q4 + pi;
    pi/2      0       d5    q5 + pi;
    pi/2      0       d6    q6 + pi;
    pi        0       d7    q7 + pi
];


%% rotation for z axis first, then x axis
% T = [cos(theta)             -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a*cos(theta);
%          sin(theta)              cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)  a*sin(theta);
%          0                      sin(alpha)               cos(alpha)              d;
%          0                      0                        0                       1];

T_final_handbook = eye(4);

for i = 1:size(DH_params, 1)
    alpha = DH_params(i, 1);
    a = DH_params(i, 2);
    d = DH_params(i, 3);
    theta = DH_params(i, 4);
    
    % test_value = cos(alpha);

    % Create Homogeneous transform matrices 
    T = [cos(theta)             -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a*cos(theta);
         sin(theta)              cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)  a*sin(theta);
         0                      sin(alpha)               cos(alpha)              d;
         0                      0                        0                       1];

    
    % Chain multiplication
    T_final_handbook = T_final_handbook * T;
    
    if (i==1)
        % 输出当前的 T 矩阵
        fprintf('T matrix for base = \n');
        disp(T_final_handbook);
    else
        % 输出当前的 T 矩阵
        fprintf('T matrix for i = %d:\n', i-1);
        disp(T_final_handbook);
    end


end

disp("T_final is equal to : ")
disp(T_final_handbook)
disp("------------------------------------------------------")


%%
T11 = cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)));
T12 = sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)));
T13 = - cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)));
T14 = d4*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) + d2*sin(q1) + d5*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + d7*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - d6*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - d3*cos(q1)*sin(q2);

T21 = cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)));
T22 = sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)));
T23 = - cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)));
T24 = d4*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - d6*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + d2*cos(q1) + d5*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + d7*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + d3*sin(q1)*sin(q2);

T31 = sin(q7)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)));
T32 = - cos(q7)*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)));
T33 = cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)) - sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5));
T34 = d0 - d1 + d7*(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))) - d5*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)) - d3*cos(q2) - d6*(sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - d4*sin(q2)*sin(q3);

T41 = 0;
T42 = 0;
T43 = 0;
T44 = 1;


