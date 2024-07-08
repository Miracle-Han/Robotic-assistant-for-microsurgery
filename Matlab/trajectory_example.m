% 时间参数
T = 10; % 时间周期
dt = 0.01; % 时间步长
time = 0:dt:T;

% 位置参数（初始位置和终止位置）
x0 = 0; y0 = 0; z0 = 0;
xf = 10; yf = 10; zf = 10;

% 姿态参数（初始姿态和终止姿态）
alpha0 = 0; beta0 = 0; gamma0 = 0;
alphaf = pi/2; betaf = pi/2; gammaf = pi/2;

% 三次样条插值计算位置轨迹
x_traj = spline([0 T], [x0 xf], time);
y_traj = spline([0 T], [y0 yf], time);
z_traj = spline([0 T], [z0 zf], time);

% 三次样条插值计算姿态轨迹
alpha_traj = spline([0 T], [alpha0 alphaf], time);
beta_traj = spline([0 T], [beta0 betaf], time);
gamma_traj = spline([0 T], [gamma0 gammaf], time);

% 计算位置轨迹的速度（对时间求导）
x_vel = diff(x_traj) / dt;
y_vel = diff(y_traj) / dt;
z_vel = diff(z_traj) / dt;

% 补齐速度数组长度
time_vel = time(1:end-1); % 时间向量对应速度长度

% 可视化位置轨迹
figure;
plot3(x_traj, y_traj, z_traj, 'b-', 'LineWidth', 2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End Effector Position Trajectory');
grid on;

% 可视化姿态轨迹
figure;
subplot(3,1,1);
plot(time, alpha_traj, 'r-', 'LineWidth', 2);
xlabel('Time'); ylabel('Alpha');
title('End Effector Alpha Trajectory');

subplot(3,1,2);
plot(time, beta_traj, 'g-', 'LineWidth', 2);
xlabel('Time'); ylabel('Beta');
title('End Effector Beta Trajectory');

subplot(3,1,3);
plot(time, gamma_traj, 'b-', 'LineWidth', 2);
xlabel('Time'); ylabel('Gamma');
title('End Effector Gamma Trajectory');

% 可视化速度轨迹
figure;
subplot(3,1,1);
plot(time_vel, x_vel, 'r-', 'LineWidth', 2);
xlabel('Time'); ylabel('X Velocity');
title('End Effector X Velocity Trajectory');

subplot(3,1,2);
plot(time_vel, y_vel, 'g-', 'LineWidth', 2);
xlabel('Time'); ylabel('Y Velocity');
title('End Effector Y Velocity Trajectory');

subplot(3,1,3);
plot(time_vel, z_vel, 'b-', 'LineWidth', 2);
xlabel('Time'); ylabel('Z Velocity');
title('End Effector Z Velocity Trajectory');
