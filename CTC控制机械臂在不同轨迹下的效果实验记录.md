# 一、实验简述及运行说明
## 1.1 实验简述

本实验首先通过（逆运动学方法），根据给定的末端执行机构位姿计算7轴机械臂各个关节的角度，从而确定机械臂初始姿态；在此基础上使用CTC控制机械臂进行轨迹跟踪，试验了该方法在不同轨迹下的跟踪效果。

## 1.2 运行说明

项目文件夹 `20240408-CTC机械臂控制-其他轨迹` 结构如下

```shell

```

运行时，先清除运行环境中其他名为 `iiwa14_multibody_dynamics_CTC_modified` 的模型，打开 `初始构型指定` 文件夹，运行 `ctcControlFile.mlx` 文件。得到的轨迹图存在 `初始构型指定/fig` 文件夹下。

# 二、开发与运行环境

# 三、算法原理

## 3.1 逆运动学求解机械臂初始关节角度

## 3.2 CTC控制

略

# 四、算法实现

# 五、实验结果与分析

## 5.1 机械臂初始关节角度计算结果

## 5.2 CTC控制机械臂跟踪不同轨迹的效果

# 六、下一步实验计划

# 参考材料

1. [Modeling Inverse Kinematics in a Robotic Arm - MATLAB & Simulink - MathWorks 中国](https://ww2.mathworks.cn/help/fuzzy/modeling-inverse-kinematics-in-a-robotic-arm.html)
	1. 展示了一种利用ANFIS网络构建的模糊系统，并以此通过末端执行机构位姿预测机械臂的关节角度（没什么用）
2. [Solve closed-form inverse kinematics - MATLAB - MathWorks 中国](https://ww2.mathworks.cn/help/robotics/ref/analyticalinversekinematics.html)
	1. 提供了求刚体树形机器人闭式解的工具（算不了）
3. [创建逆运动学求解器 - MATLAB - MathWorks 中国 --- Create inverse kinematic solver - MATLAB - MathWorks 中国](https://ww2.mathworks.cn/help/robotics/ref/inversekinematics-system-object.html)
	1. 使用数值法求解机械臂逆运动学问题

## Robotics Model Toolbox

### analyticalInverseKinematics求解闭式逆运动学

```matlab
% 创建用于求机械臂逆运动学解析解的求解器
aik = analyticalInverseKinematics(robotRBT);    % robotRBT是机械臂模型的rigidBodyTree对象
```

### inverseKinematics求解逆运动学

```matlab
robotRBT = DOF7_iiwa14;    % 机械臂rigidBodyTree
eeName = 'iiwa_link_7';    % 末端执行器名称
eePose = [1,0,0,0,0,-pi];    % 末端执行器初始位姿向量
TForm = eepose2tform(eePose);    % 将位姿向量转化为齐次变换矩阵
weights = [1,1,1,1,1,1];    % 姿态误差加权向量
initGuess = [0,0,0,0,0,0,0];    % 关节角度迭代初值


ik = inverseKinematics('RigidBodyTree',robotRBT);    % 创建用于逆运动学求解的求解器，默认使用BFGS算法求解
%ik = inverseKinematics('RigidBodyTree',robotRBT,'SolverAlgorithm','LevenbergMarquardt');    % 使用LM算法求解
[config,info] = ik(eeName,TForm,weights,initGuess);    %逆运动学求解
disp(config);


function TForm = eepose2tform(eePose)
	TrVec = eePose(1:3);
	EulZYX = eePose(4:6);
	TrMat = trvec2tform(TrVec);
	RotMat = eul2tform(EulZYX, 'ZYX');
	TForm = TrMat * RotMat;
end
```