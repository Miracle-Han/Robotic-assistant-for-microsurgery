%五次多项式插值规划
%刘躺
%2022.5.20
%matlab2019b
clear;
close all;
clear all;
clc;
clear L;

radian1=pi/180;  %弧度角度转化

%定义关节角度限制
lim1_min = -170 * radian1; lim1_max = 170 * radian1; %关节1(-170，170)
lim2_min = -132 * radian1; lim2_max =   0 * radian1; %关节2(-132，0)
lim3_min =    1 * radian1; lim3_max = 141 * radian1; %关节3(1，141)
lim4_min = -165 * radian1; lim4_max = 165 * radian1; %关节4(-165，165)
lim5_min = -105 * radian1; lim5_max = 105 * radian1; %关节5(-105，105)
lim6_min = -155 * radian1; lim6_max = 155 * radian1; %关节6(-155，155)

%定义关节旋转范围
lim1 = lim1_max - lim1_min;
lim2 = lim2_max - lim2_min;
lim3 = lim3_max - lim3_min;
lim4 = lim4_max - lim4_min;
lim5 = lim5_max - lim5_min;
lim6 = lim6_max - lim6_min;

% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%刘躺%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta1 = 0;    d1 = 169.77;     a1 = 64.2;    alpha1 = -pi/2;    offset1 = 0;
theta2 = 0;    d2 = 0;          a2 = 305;     alpha2 = 0;        offset2 = 0;
theta3 = 0;    d3 = 0;          a3 = 0;       alpha3 = pi/2;     offset3 = -pi/2;
theta4 = 0;    d4 = -222.63;    a4 = 0;       alpha4 = -pi/2;    offset4 = 0;
theta5 = 0;    d5 = 0;          a5 = 0;       alpha5 = pi/2;     offset5 = 0;
theta6 = 0;    d6 = -36.25;     a6 = 0;       alpha6 = 0;        offset6 = pi;

L(1) = Link([theta1, d1, a1, alpha1, offset1], 'standard');
L(2) = Link([theta2, d2, a2, alpha2, offset2], 'standard');
L(3) = Link([theta3, d3, a3, alpha3, offset3], 'standard');
L(4) = Link([theta4, d4, a4, alpha4, offset4], 'standard');
L(5) = Link([theta5, d5, a5, alpha5, offset5], 'standard');
L(6) = Link([theta6, d6, a6, alpha6, offset6], 'standard');

% 定义关节范围
L(1).qlim=[lim1_min,lim1_max];
L(2).qlim=[lim2_min,lim2_max];
L(3).qlim=[lim3_min,lim3_max];
L(4).qlim=[lim4_min,lim4_max];
L(5).qlim=[lim5_min,lim5_max];
L(6).qlim=[lim6_min,lim6_max];  

robot = SerialLink(L,'name','AR3');

%  五次多项式插值jtraj
T1=transl(-100,-100,300);				%齐次变换矩阵
T2=transl(200,-200,400);				%齐次变换矩阵
init_ang=robot.ikine(T1);				%运动学逆解
targ_ang=robot.ikine(T2);				%运动学逆解

%轨迹规划
f = 1;
figure(f)
step = 50;
[q ,qd, qdd]=jtraj(init_ang,targ_ang,step); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
grid on
T=robot.fkine(q);  %根据插值，得到末端执行器位姿
nT=T.T;
plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));%输出末端轨迹
title('五次多项式插值轨迹');
robot.plot(q);	    %动画演示

f = 2;
figure(f)
subplot(6, 1, 1);
plot(q(:,1)*radian1);
title('关节1角度(五次多项式插值)');
grid on;
subplot(6, 1, 2);
plot(q(:,2)*radian1);
title('关节2角度(五次多项式插值)');
grid on;
subplot(6, 1, 3);
plot(q(:,3)*radian1);
title('关节3角度(五次多项式插值)');
grid on;
subplot(6, 1, 4);
plot(q(:,4)*radian1);
title('关节4角度(五次多项式插值)');
grid on;
subplot(6, 1, 5);
plot(q(:,5)*radian1);
title('关节5角度(五次多项式插值)');
grid on;
subplot(6, 1, 6);
plot(q(:,6)*radian1);
title('关节6角度(五次多项式插值)');
grid on;
% legend ('关节1', '关节2' ,'关节3' ,'关节4' ,'关节5' ,'关节6');

f = 3;
figure(f)
subplot(6, 1, 1);
plot(qd(:,1)*radian1);
title('关节1角速度(五次多项式插值)');
grid on;
subplot(6, 1, 2);
plot(qd(:,2)*radian1);
title('关节2角速度(五次多项式插值)');
grid on;
subplot(6, 1, 3);
plot(qd(:,3)*radian1);
title('关节3角速度(五次多项式插值)');
grid on;
subplot(6, 1, 4);
plot(qd(:,4)*radian1);
title('关节4角速度(五次多项式插值)');
grid on;
subplot(6, 1, 5);
plot(qd(:,5)*radian1);
title('关节5角速度(五次多项式插值)');
grid on;
subplot(6, 1, 6);
plot(qd(:,6)*radian1);
title('关节6角速度(五次多项式插值)');
grid on;

f = 4;
figure(f)
subplot(6, 1, 1);
plot(qdd(:,1)*radian1*radian1);
title('关节1角加速度(五次多项式插值)');
grid on;
subplot(6, 1, 2);
plot(qdd(:,2)*radian1*radian1);
title('关节2角加速度(五次多项式插值)');
grid on;
subplot(6, 1, 3);
plot(qdd(:,3)*radian1*radian1);
title('关节3角加速度(五次多项式插值)');
grid on;
subplot(6, 1, 4);
plot(qdd(:,4)*radian1*radian1);
title('关节4角加速度(五次多项式插值)');
grid on;
subplot(6, 1, 5);
plot(qdd(:,5)*radian1*radian1);
title('关节5角加速度(五次多项式插值)');
grid on;
subplot(6, 1, 6);
plot(qdd(:,6)*radian1*radian1);
title('关节6角加速度(五次多项式插值)');
grid on;
