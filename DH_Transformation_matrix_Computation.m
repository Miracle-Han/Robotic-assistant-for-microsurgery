%% 根据 DH Parameter 计算 Transformation matrix
clear all; % Clear all variables
close all; % Close all figures
clc; % Clear screen

% % Symbolic joint angle variables
% syms q1 q2 q3 q4 q5 q6 q7 real;


%% DH parameter [alpha_i; a_i; d_i; theta_i]
% alpha_i: the angle from Zi-1 to Zi measured about Xi-1
% a_i: the distance from Zi-1 to Zi measured along Xi-1
% d_i: the distance from Xi-1 to Xi measured along Zi
% theta_i: the angle from Xi-1 to Xi measured about Zi

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
    n = i-1;
    m = i-2;
%     if(m<0)
%         disp("T: " + "based-" + n +" is equal to : ")
%     else
%         disp("T: " + m + "-"+n +" is equal to : ")
%     end
%     disp(T)

    % Chain multiplication
    T_final_handbook = T_final_handbook * T;
end

disp("T_final is equal to : ")
disp(T_final_handbook)
disp("------------------------------------------------------")

%%


% 示例使用
w = 0.6; 
x = 0.0; 
y = 0.8; 
z = 0.0;
rotationMatrix = quaternionToRotationMatrix(w, x, y, z);

% 打印结果
disp('Rotation Matrix:');
disp(rotationMatrix);


function R = quaternionToRotationMatrix(w, x, y, z)
    % 初始化旋转矩阵R
    R = zeros(3,3);
    
    % 计算旋转矩阵的各个元素
    R(1,1) = 1 - 2*y^2 - 2*z^2;
    R(1,2) = 2*x*y - 2*z*w;
    R(1,3) = 2*x*z + 2*y*w;
    R(2,1) = 2*x*y + 2*z*w;
    R(2,2) = 1 - 2*x^2 - 2*z^2;
    R(2,3) = 2*y*z - 2*x*w;
    R(3,1) = 2*x*z - 2*y*w;
    R(3,2) = 2*y*z + 2*x*w;
    R(3,3) = 1 - 2*x^2 - 2*y^2;
end


% %% rotation for x axis first, then z axis
% %Transformation matrix
% %[            cos(th),           -sin(th),           0,             a]
% %[ cos(alpha)*sin(th), cos(alpha)*cos(th), -sin(alpha), -d*sin(alpha)]
% %[ sin(alpha)*sin(th), sin(alpha)*cos(th),  cos(alpha),  d*cos(alpha)]
% %[                  0,                  0,           0,             1]
% 
% T_final_lecture = eye(4);
% 
% for i = 1:size(DH_params, 1)
%     alpha = DH_params(i, 1);
%     a = DH_params(i, 2);
%     d = DH_params(i, 3);
%     theta = DH_params(i, 4);
% 
%     % 
%     T = [cos(theta),              -sin(theta),               0,               a;
%          sin(theta)*cos(alpha),   cos(theta)*cos(alpha),    -sin(alpha),      -sin(alpha)*d;
%          sin(theta)*sin(alpha),   cos(theta)*sin(alpha),     cos(alpha),       cos(alpha)*d;
%          0,                       0,                         0,                1];
% 
% 
%     n = i-1;
%     disp("From lecture: "+"T" + n +" is equal to : ")
%     disp(T)
% 
%     % Chain multiplication
%     T_final_lecture = T_final_lecture * T;
% end
% 
% disp("T_final_lecture is equal to : ")
% disp(T_final_lecture)
% 
% % % Jacobian Matrix
% % p = [T_final_handbook(1,4);T_final_handbook(2,4);T_final_handbook(3,4)];
% % Jacobian = jacobian(p, [q1 q2 q3 q4 q5 q6 q7]);
% % 
% % 
% % % Output Printing
% % disp("------------------------------------------------------")
% % disp("jacobian is equal to : ")
% % disp(Jacobian)
% % disp("done")
