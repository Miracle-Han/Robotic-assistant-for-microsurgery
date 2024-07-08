% 导入DH参数
% DH参数格式 [alpha_i, a_i, d_i, theta_i]




%% Defination

pi = 3.1415926;

d0 = 0;
d1 = -(0.1564+0.1284);
d2 = -(0.0054+0.0064);
d3 = -(0.2104+0.2104);
d4 = -(0.0064+0.0064);
d5 = -(0.2084+0.1059);
d6 = 0;
d7 = -(0.1059+0.0615);

% 定义关节角度
%                                                                                                                                                        
q1 = 1.245226;
q2 = -1.014591;
q3 = -1.626709;
q4 = 1.306197;
q5 = -0.040284;
q6 = 0.686400;
q7 = -1.036441;

DH_params = [
    pi/2  0  d1  q1;
    pi/2  0  d2  q2 + pi;
    pi/2  0  d3  q3 + pi;
    pi/2  0  d4  q4 + pi;
    pi/2  0  d5  q5 + pi;
    pi/2  0  d6  q6 + pi;
    pi    0  d7  q7 + pi
];

% 计算每个关节的变换矩阵
T = @(alpha, a, d, theta) [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                           0,           sin(alpha),            cos(alpha),            d;
                           0,           0,                     0,                     1];

% 初始化变换矩阵
T0_n = [1,  0,  0,  0;
        0, -1,  0,  0;
        0,  0, -1,  0;
        0,  0,  0,  1];

% 关节数量
num_joints = size(DH_params, 1);

% 初始化位置和Z轴向量
positions = zeros(3, num_joints + 1);
z_vectors = zeros(3, num_joints + 1);
z_vectors(:,1) = T0_n(1:3, 3);  % 基坐标系的Z轴

% 计算位置和Z轴向量
for i = 1:num_joints
    % 获取DH参数
    alpha = DH_params(i, 1);
    a = DH_params(i, 2);
    d = DH_params(i, 3);
    theta = DH_params(i, 4);
    
    % 计算变换矩阵
    T_i = T(alpha, a, d, theta);
    
    % 更新总变换矩阵
    T0_n = T0_n * T_i;
    
    % 提取位置和Z轴向量
    positions(:, i+1) = T0_n(1:3, 4);
    z_vectors(:, i+1) = T0_n(1:3, 3);
end

% 计算Jacobian矩阵
J = zeros(6, num_joints);
for i = 1:num_joints
    J(1:3, i) = cross(z_vectors(:, i), (positions(:, end) - positions(:, i)));
    J(4:6, i) = z_vectors(:, i);
end

% 显示Jacobian矩阵
disp('Jacobian Matrix:');
disp(J);
