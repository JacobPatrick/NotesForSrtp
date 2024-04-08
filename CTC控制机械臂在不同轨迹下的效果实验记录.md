# 一、实验简述及运行说明
## 1.1 实验简述

本实验首先通过（逆运动学方法），根据给定的末端执行机构位姿计算7轴机械臂各个关节的角度，从而确定机械臂初始姿态；在此基础上使用CTC控制机械臂进行轨迹跟踪，试验了该方法在不同轨迹下的跟踪效果。

## 1.2 运行说明

项目文件夹 `20240408-CTC机械臂控制-其他轨迹` 结构如下

```

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
	1. 展示了一种利用ANFIS网络构建的模糊系统，并以此通过末端执行机构位姿预测机械臂的关节角度
2. [Solve closed-form inverse kinematics - MATLAB - MathWorks 中国](https://ww2.mathworks.cn/help/robotics/ref/analyticalinversekinematics.html)
	1. 提供了求刚体树形机器人闭式解的工具