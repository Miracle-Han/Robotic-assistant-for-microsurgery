clear all; % Clear all variables
close all; % Close all figures
clc; % Clear screen

%% Defination
d0 = 0;
d1 = -(0.1564+0.1284);
d2 = -(0.0054+0.0064);
d3 = -(0.2104+0.2104);
d4 = -(0.0064+0.0064);
d5 = -(0.2084+0.1059);
d6 = 0;
d7 = -(0.1059+0.0615);

% 定义关节角度
q1 = 1.245226;
q2 = -1.014591;
q3 = -1.626709;
q4 = 1.306197;
q5 = -0.040284;
q6 = 0.686400;
q7 = -1.036441;

pi = 3.14;

% defination of DH parameters
% [alpha_i; a_i; d_i; theta_i]
DH_params = [
    % pi        0       d0    0;
    pi/2      0       d1    q1;
    pi/2      0       d2    q2 + pi;
    pi/2      0       d3    q3 + pi;
    pi/2      0       d4    q4 + pi;
    pi/2      0       d5    q5 + pi;
    pi/2      0       d6    q6 + pi;
    pi        0       d7    q7 + pi
];

%% Using matlab robot toolbox
% 定义 DH 参数
%         theta   d     a    alpha "rotation joint=0"  offset
L2 = Link([0       d1    0    pi/2  0], 'standard');
L3 = Link([pi      d2    0    pi/2  0], 'standard'); % L3.offset = pi;
L4 = Link([pi      d3    0    pi/2  0], 'standard'); % L4.offset = pi;
L5 = Link([pi      d4    0    pi/2  0], 'standard'); % L5.offset = pi;
L6 = Link([pi      d5    0    pi/2  0], 'standard'); % L6.offset = pi;
L7 = Link([pi      d6    0    pi/2  0], 'standard'); % L7.offset = pi;
L8 = Link([pi      d7    0    pi    0], 'standard'); % L8.offset = pi;

% 创建机器人模型
robot = SerialLink([L2 L3 L4 L5 L6 L7 L8], 'name', 'robotarm');

% 定义关节角度
% Q = [0, 0, 0, 0, 0, 0, 0];
Q = [q1, q2, q3, q4, q5, q6, q7];

% 计算雅可比矩阵
jacobian = robot.jacob0(Q);

% 显示雅可比矩阵
disp('Jacobian Matrix(using toolbox):');
disp(jacobian);

%% T01, T12, T23, T34, T45, T56, T67
% 初始化变换矩阵列表
T_matrices = cell(size(DH_params, 1), 1);

% 循环计算每个变换矩阵
for i = 1:size(DH_params, 1)
    alpha = DH_params(i, 1);
    a = DH_params(i, 2);
    d = DH_params(i, 3);
    theta = DH_params(i, 4);

    T_matrices{i} = trans(alpha, a, d, theta);
end

%% T01, T02, T03, T04, T05, T06, T07
% 初始化累积变换矩阵列表
T_cumulative = cell(size(DH_params, 1), 1);

% 初始变换矩阵 T01
T_cumulative{1} = T_matrices{1};

% 循环计算累积变换矩阵
for i = 2:length(T_matrices)
    T_cumulative{i} = T_cumulative{i-1} * T_matrices{i};
end

%% P01, P02 ... P07
% 初始化位移部分列表
P = cell(size(DH_params, 1), 1);

% 提取位移部分
for i = 1:length(T_cumulative)
    P{i} = T_cumulative{i}(1:3, 4);
end

%% Z1, Z2, Z7
% 初始化z轴方向列表
Z = cell(size(DH_params, 1), 1);

% 提取z轴方向
for i = 1:length(T_cumulative)
    Z{i} = T_cumulative{i}(1:3, 3);
end


j1 = [cross([0;0;1],P{7});[0;0;1]];
j2 = [cross(Z{1},P{7}-P{1});Z{2}];
j3 = [cross(Z{2},P{7}-P{2});Z{3}];
j4 = [cross(Z{3},P{7}-P{3});Z{4}];
j5 = [cross(Z{4},P{7}-P{4});Z{5}];
j6 = [cross(Z{5},P{7}-P{5});Z{6}];
j7 = [cross(Z{6},P{7}-P{6});Z{7}];
jacobian0 = [j1,j2,j3,j4,j5,j6,j7];

disp("Jacobian Matrix is: ");
disp(jacobian0);

function T = trans(alpha, a, d, theta)
    T = [cos(theta)             -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a*cos(theta);
         sin(theta)              cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)  a*sin(theta);
         0                      sin(alpha)               cos(alpha)              d;
         0                      0                        0                       1];
end
