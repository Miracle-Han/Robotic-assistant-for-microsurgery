

% 导入机器人模型
robot = importrobot('GEN3_URDF_V12.urdf');

% 定义关节角度
q1 = 1.245226;
q2 = -1.014591;
q3 = -1.626709;
q4 = 1.306197;
q5 = -0.040284;
q6 = 0.686400;
q7 = -1.036441;

% 创建关节角度配置结构
config = homeConfiguration(robot);
config(1).JointPosition = q1;
config(2).JointPosition = q2;
config(3).JointPosition = q3;
config(4).JointPosition = q4;
config(5).JointPosition = q5;
config(6).JointPosition = q6;
config(7).JointPosition = q7;

% 计算Jacobian矩阵
endEffector = 'EndEffector_Link'; % 替换为你的机械臂末端执行器的名称
J = geometricJacobian(robot, config, endEffector);

% 交换前三行和后三行
J = [J(4:6, :); J(1:3, :)];

% 显示Jacobian矩阵
disp('Jacobian Matrix:');
disp(J);

show(robot, config);
