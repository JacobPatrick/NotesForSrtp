时间：2024-04-08
# 一、实验简述及运行说明
## 1.1 实验简述

本实验首先通过（逆运动学方法），根据给定的末端执行机构位姿计算7轴机械臂各个关节的角度，从而确定机械臂初始姿态；在此基础上使用CTC控制机械臂进行轨迹跟踪，试验了该方法在不同轨迹下的跟踪效果。

## 1.2 运行说明

项目文件夹 `20240408-CTC机械臂控制-其他轨迹` 结构如下

```shell

```

运行时，先清除运行环境中其他名为 `iiwa14_multibody_dynamics_CTC_modified` 的模型，打开 `初始构型指定` 文件夹，运行 `ctcControlFile.mlx` 文件。得到的轨迹图存在 `初始构型指定/fig` 文件夹下。

# 二、开发与运行环境

操作系统						Microsoft Windows 11 专业教育版 Version 10.0 (Build 22631)
Java							Java 1.8.0_202-b08 with Oracle Corporation Java HotSpot(TM) 64-Bit Server VM mixed mode
MATLAB                                                版本 9.14             (R2023a)
Simulink                                              版本 10.7             (R2023a)
Communications Toolbox                                版本 8.0              (R2023a)
Control System Toolbox                                版本 10.13            (R2023a)
DSP System Toolbox                                    版本 9.16             (R2023a)
Deep Learning Toolbox                                 版本 14.6             (R2023a)
Parallel Computing Toolbox                            版本 7.8              (R2023a)
Reinforcement Learning Toolbox                        版本 2.4              (R2023a)
Robotics System Toolbox                               版本 4.2              (R2023a)
Signal Processing Toolbox                             版本 9.2              (R2023a)
Simscape                                              版本 5.5              (R2023a)
Simscape Multibody                                    版本 7.7              (R2023a)

# 三、算法原理

## 3.1 逆运动学求解机械臂初始关节角度

略

## 3.2 CTC控制

略

# 四、算法实现

## 4.1 求解初始关节角度

```matlab
%% 导入机械臂模型，并设置求解参数
robotRBT = DOF7_iiwa14;
eeName = 'Body10';    % 末端执行器名称
eePose = [1,0,0,0,0,-pi];    % 末端执行器初始位姿向量
TForm = eepose2tform(eePose);    % 将位姿向量转化为齐次变换矩阵
weights = [1,1,1,1,1,1];    % 姿态误差加权向量
initGuess = homeConfiguration(robotRBT);    % 关节角度迭代初值

%% 逆运动学求解初始关节角度
% ik = inverseKinematics('RigidBodyTree',robotRBT);    % 创建用于逆运动学求解的求解器，默认使用BFGS算法求解
ik = inverseKinematics('RigidBodyTree',robotRBT,'SolverAlgorithm','LevenbergMarquardt');    % 使用LM算法求解
[config,info] = ik(eeName,TForm,weights,initGuess);    %逆运动学求解
jointAngles = [config.JointPosition];
disp(jointAngles);

%% 末端执行器位姿向量与齐次变换矩阵转换函数
function TForm = eepose2tform(eePose)
	TrVec = eePose(1:3);
	EulZYX = eePose(4:6);
	TrMat = trvec2tform(TrVec);
	RotMat = eul2tform(EulZYX, 'ZYX');
	TForm = TrMat * RotMat;
end
```

## 4.2 CTC轨迹跟踪

```matlab
clc;
clear;
close all;

%% Trajectory generate
Ts=0.001;

DOF7_iiwa14=importrobot('iiwa14_multibody_waypoints');

% generate a n-side regular polygon trajectory
n = 50;
nlist = 1:n+1;
nlist = nlist * pi / 25;

EulZYXpoints2 = repmat([0;0;-pi],[1,n+1]);

waypoints_x = arrayfun(@cos,nlist)/5 + 0.3;
waypoints_y = arrayfun(@sin,nlist)/5 + 0.3;
waypoints_z = ones(1,n+1) * 0.6500;

waypoints2 = [waypoints_x;waypoints_y;waypoints_z];

waypointsTime = repmat([8/n;8/n;8/n],[1,n]);

%% CTC-PID Control
mdl = 'iiwa14_multibody_dynamics_CTC_modified';
load_system(mdl);

% assign the pid params
kp = 10000;
kd = 150;
pid_params = sprintf('[%d %d]',kp,kd);
path = [mdl,'/pid_params'];
set_param(path,'Value',pid_params);

% assign the initial guess of pose
eeName = 'Body10';
eeTrVec = waypoints2(:,0);    % 列向量
eeEulZYX = EulZYXpoints2(:,0);    % 列向量
eePose = [eeTrVec',eeEulZYX'];    % 行向量
TForm = eepose2tform(eePose);
weights = [1,1,1,1,1,1];
initGuess = homeConfiguration(DOF7_iiwa14);

ik = inverseKinematics('RigidBodyTree',DOF7_iiwa14,'SolverAlgorithm','LevenbergMarquardt');
[config,info] = ik(eeName,TForm,weights,initGuess);
jointAngles = [config.JointPosition];
str_jointAngles = mat2str(jointAngles);
path = [mdl,'/Constant'];
set_param(path,'Value',str_jointAngles);

save_system(mdl);
sim(mdl, [0 8]);

%% Compare Reference Trajectory with Real Trajectory
dataRef = load('recordingRef1.mat');
dataReal = load('recordingReal1.mat');

dataLength = length(dataRef.data{1}.Values.Data);

% cumulative error
cumErr = 0;

for i = 1:1:dataLength
    XRef(i) = dataRef.data{1}.Values.Data(1,1,i);
    YRef(i) = dataRef.data{2}.Values.Data(1,1,i);
    XReal(i) = dataReal.data{1}.Values.Data(1,1,i);
    YReal(i) = dataReal.data{2}.Values.Data(1,1,i);

    err = norm([XReal(i) YReal(i)] - [XRef(i) YRef(i)]);
    cumErr = cumErr + err;
end

% draw reference trajectory and real trajectory
plot(XRef,YRef);
hold on
plot(XReal,YReal);

TextBox = annotation('textbox',[0.15,0.2,0.1,0.1]);
TextBox.String = {['kp:',num2str(kp)], ['kd:',num2str(kd)], ['cumErr:',num2str(cumErr)]};   
TextBox.BackgroundColor = [1 1 1 0];
TextBox.FontSize = 12;

%print and save the figure
filename = sprintf('traj_kp_%d_kd_%d.png',kp,kd);
fullpath = strcat('fig/',filename);
saveas(gcf,fullpath);

function TForm = eepose2tform(eePose)
	TrVec = eePose(1:3);
	EulZYX = eePose(4:6);
	TrMat = trvec2tform(TrVec);
	RotMat = eul2tform(EulZYX, 'ZYX');
	TForm = TrMat * RotMat;
end
```
# 五、实验结果与分析

## 5.1 初始关节角度计算结果

略，其实这一步或许不必要，因为在CTC模型的逆运动学部分已经具备求初始时刻以及过程中关节角度的能力

## 5.2 CTC控制机械臂跟踪不同轨迹的效果

### 5.2.1 正n边形轨迹

最初是想做个圆，但是注意到模型 `mdl` 中的轨迹生成模块 `Trapezoidal Velocity Profile Trajectory` 使用顶点坐标矩阵生成轨迹。以正50边形为例，轨迹通过如下方式生成

```matlab
% generate a n-side regular polygon trajectory
n = 50;
nlist = 1:n+1;

EulZYXpoints2 = repmat([0;0;-pi],[1,n+1]);

waypoints_x = arrayfun(@cos,nlist);
waypoints_y = arrayfun(@sin,nlist);
waypoints_z = ones(1,n+1) * 0.6500;

waypoints2 = [waypoints_x;waypoints_y;waypoints_z];

waypointsTime = repmat([1;1;1],[1,n]);
```

最初设定的轨迹是一个内接于单位圆的正50边形，但在模拟过程中出现==奇异点==，此后尝试了另一些不同的正50边形轨迹，很多同样存在此问题。猜测是由于==轨迹上的某些位置该机械臂难以到达==，或者由于==CTC-PID控制的方案本身存在局限，使得对关节角度规划不合理，导致末端运动受限==。
# 六、下一步实验计划

1. 仍然尝试用智能体调节pid参数的方式，将DDPG智能体换成TD3，SAC等，比较控制效果，希望在月底期中考试后一周左右的时间可以完成；
2. 在复杂一些的轨迹跟踪任务中应用DDPG，比如考虑需要更多自由度的路径，或者其他约束条件；
3. 或许可以试试去掉pid控制器，目前还没考虑好怎么做；
4. 如果可以去掉pid的话，可以尝试从机械臂轨迹跟踪中的避奇异、关节力矩优化等角度对DDPG控制方法进行评价。

# Notes

1. [Modeling Inverse Kinematics in a Robotic Arm - MATLAB & Simulink - MathWorks 中国](https://ww2.mathworks.cn/help/fuzzy/modeling-inverse-kinematics-in-a-robotic-arm.html)
	1. 展示了一种利用ANFIS网络构建的模糊系统，并以此通过末端执行机构位姿预测机械臂的关节角度（没什么用）
2. [Solve closed-form inverse kinematics - MATLAB - MathWorks 中国](https://ww2.mathworks.cn/help/robotics/ref/analyticalinversekinematics.html)
	1. 提供了求刚体树形机器人闭式解的工具（算不了）
3. [创建逆运动学求解器 - MATLAB - MathWorks 中国 --- Create inverse kinematic solver - MATLAB - MathWorks 中国](https://ww2.mathworks.cn/help/robotics/ref/inversekinematics-system-object.html)
	1. 使用数值法求解机械臂逆运动学问题
4. [Create tree-structured robot - MATLAB - MathWorks 中国](https://ww2.mathworks.cn/help/robotics/ref/rigidbodytree.html)
	1. rigidBodyTree数据类型简介