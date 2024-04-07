## 一、实验简述及运行说明

### 1.1 实验简述

本实验通过Matlab和Simulink实现使用计算力矩控制（CTC）和PID控制结合的7轴机械臂轨迹跟踪控制，针对正方形目标轨迹，进行PID参数整定，得到了一组较好的参数，并尝试在其他形状的路径上进行了测试（暂时没做好）。

### 1.2 运行说明

项目文件夹 `20240402-CTC机械臂控制` 结构如下所示

```bash
E:.
└─初始构型指定
    ├─fig
    ├─iiwa_description
    │  ├─meshes
    │  │  ├─iiwa14
    │  │  │  ├─collision
    │  │  │  └─visual
    │  │  └─iiwa7
    │  │      ├─collision
    │  │      └─visual
    │  └─urdf
    └─slprj
        ├─sim
        │  └─varcache
        │      └─iiwa14_multibody_dynamics_CTC_modified
        │          └─tmwinternal
        ├─_jitprj
        └─_sfprj
            ├─EMLReport
            │  └─sdYXwxUFb6nCvw05C3Ip8RB
            ├─iiwa14_multibody_dynamics_CTC_modified
            │  └─_self
            │      └─sfun
            │          └─info
            └─precompile
```

运行时，先清除运行环境中其他名为 `iiwa14_multibody_dynamics_CTC_modified` 的模型，打开 `初始构型指定` 文件夹，运行 `ctcControlFile.mlx` 文件。得到的轨迹图存在 `初始构型指定/fig` 文件夹下。

## 二、开发与运行环境

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

## 三、算法原理

### 3.1 计算力矩法控制

略

### 3.2 PID参数整定

本实验中使用PD控制器，优化控制器参数 $k_p,k_d$ 的目标是控制7轴机械臂尽可能实现对已知轨迹的高精度跟踪。将==优化目标==量化为实际轨迹 $\boldsymbol{q}(t)$ 与目标轨迹 $\boldsymbol{q}_d(t)$ $(0 \leq t \leq T)$ 在所有时刻的距离之和
$$
\text{cumErr} = \sum_{i=0}^T ||\boldsymbol{q}(i) - \boldsymbol{q}_d(i)||_2 = f(k_p,k_d)
$$
假设 $k_p,k_d \in [0,10000]$

#### 3.2.1 手动调参

最初是计划用类似牛顿法的方法来寻优的，但是发现参数一直在增大，就先手动调了一下；根据cumErr决定怎么调

#### 3.2.3 使用Ziegler-Nichols法调参

1. 令 $k_p = k_d = 0$，并从零开始缓慢增大 $k_p$
2. 寻找系统的临界稳定点以及临界稳定增益 $k_c = k_{p0}$ 和临界周期$p_c$
3. 根据 $k_c,p_c$ 以及经验公式确定合适的 $k_p,k_d$，即 $k_p = 0.8k_c, k_d = \dfrac{1}{8}k_cp_c$

## 四、算法实现

```matlab
clc;
clear;
close all;


Ts=0.001;

DOF7_iiwa14=importrobot('iiwa14_multibody_waypoints');

EulZYXpoints2 = [...
                0  0  -pi;...
                0  0  -pi;...
                0  0  -pi;...
                0  0  -pi;...
                0  0  -pi]';

waypoints2 =  [... 
            -0.1250    0.5000      0.6500;...
            -0.1250    0.3000      0.6500;...
             0.0750    0.3000      0.6500;...
             0.0750    0.5000      0.6500;...
             -0.1250    0.5000      0.6500]';

waypointsTime = [ 2 2 2 2;
                  2 2 2 2;
                  2 2 2 2];


mdl = 'iiwa14_multibody_dynamics_CTC_modified';
load_system(mdl);

% assign the pid params
kp = 10000;
kd = 160;
pid_params = sprintf('[%d %d]',kp,kd);
path = [mdl,'/pid_params'];
set_param(path,'Value',pid_params);
save_system(mdl);

sim(mdl, [0 8]);


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

% draw the reference trajectory and real trajectory
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
```

## 五、实验结果与分析

### 5.1 PID调参结果

#### 5.1.1 手动调参

因为不确定函数的凹凸性，先简单尝试几组参数

|         | kp=500                                                                                                                                                           | kp=1000                                                                                                                                                            | kp=5000                                                                                                                                                            |
| ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| kd=0    | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_500_kd_0.png" alt="traj_kp_500_kd_0" style="zoom:6%;" />cumErr = 11.5622      | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_1000_kd_0.png" alt="traj_kp_1000_kd_0" style="zoom:6%;" />cumErr=144.4247       | 轨迹超出设定区域范围                                                                                                                                                         |
| kd=100  | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_100_kd_100.png" alt="traj_kp_100_kd_100" style="zoom:6%;" />cumErr = 29.4297  | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_1000_kd_100.png" alt="traj_kp_1000_kd_100" style="zoom:6%;" />cumErr = 4.9197   | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_5000_kd_100.png" alt="traj_kp_5000_kd_100" style="zoom:6%;" />cumErr = 1.0542   |
| kd=200  | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_500_kd_200.png" alt="traj_kp_500_kd_200" style="zoom:6%;" />cumErr = 8.6582   | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_1000_kd_200.png" alt="traj_kp_1000_kd_200" style="zoom:6%;" />cumErr = 4.8266   | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_5000_kd_200.png" alt="traj_kp_5000_kd_200" style="zoom:6%;" />cumErr = 1.0547   |
| kd=1000 | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_500_kd_1000.png" alt="traj_kp_500_kd_1000" style="zoom:6%;" />cumErr = 5.5551 | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_1000_kd_1000.png" alt="traj_kp_1000_kd_1000" style="zoom:6%;" />cumErr = 4.0159 | <img src="E:\J.Patrick\courses\srtp\2023-2024SS\ctc_pid_control\初始构型指定\fig\traj_kp_5000_kd_1000.png" alt="traj_kp_5000_kd_1000" style="zoom:6%;" />cumErr = 1.0727 |

尝试了一些参数之后发现这种方法似乎并不可行，因为当 $k_p$ 不断增大，而 $k_d$ 保持在一个相对较小的值时，控制的效果会逐渐变好。==具体实验结果==已经记录在`fig`文件夹中。

经过手动调参（先确定 $k_p = 10000$，再一维搜索得到最优的 $k_d$ 值，因为可以发现当 $k_p$ 固定时，函数关于 $k_d$ 似乎是凸函数）得到的==一组较好的pid参数==为 $k_p = 10000, k_d = 150$，对应的目标函数值为 $\text{cumErr} = 0.546376$ （事实上如果继续增大参数 $k_p$ 的值，得到的结果可能更好），同时从图像上可以发现控制效果较好，跟踪存在偏差的位置主要出现在直线路径中部。

<img src="E:\J.Patrick\courses\srtp\2023-2024SS\20240402-CTC机械臂控制\初始构型指定\fig\traj_kp_10000_kd_150.png" alt="traj_kp_10000_kd_150" style="zoom:48%;" />

**讨论**

1. 当参数不断增大时跟踪的结果在不断变好，虽然没有进行更多的实验，但是也许可以猜想这种==跟踪结果随参数增大而变好的情况会继续下去，因此更多的实验或许是没有意义的==。而目前的参数已经取得了较好的结果，如果这种程度的误差是可接受的，则可以选择一组不那么大的参数作为最终结果。
2. 在实际场景中，通过模拟优化的参数很可能无法生效，目前想到了以下两点原因：
   1. 噪声和干扰的存在，很大的 $k_d$ 值可能完全破坏控制的效果；
   2. 实际机械臂模型并非完全已知，CTC的建模可能存在偏差，进而造成跟踪控制误差。

其中2.1是可能通过进一步调节pid参数来实现的，但需要更复杂的仿真模型或实物模型；2.2也许无法通过CTC-PID控制的方案完全解决。

#### 5.1.2 使用Ziegler-Nichols法调参

![image-20240402001414910](C:\Users\Jacob Patrick\AppData\Roaming\Typora\typora-user-images\image-20240402001414910.png)

![image-20240401235018434](C:\Users\Jacob Patrick\AppData\Roaming\Typora\typora-user-images\image-20240401235018434.png)

![image-20240402000756211](C:\Users\Jacob Patrick\AppData\Roaming\Typora\typora-user-images\image-20240402000756211.png)

**讨论**

实际操作也有困难，因为==很难判定系统何时处于“临界稳定”==，比如上面这组图展示了跟踪误差 $||\boldsymbol{q}(t) - \boldsymbol{q}_d(t)||_2$ 随 $t$ 的变化，可见当 $k_p$ 较大时，每次经过转角跟踪误差都会显著增大。即使不考虑这一点，由于 $k_p \geq 1000$ 时系统呈现出非常明显的失控特征，$k_c$ 一定小于1000，而手动调参已经证明这时结果并非最好，因此该方法在此或许并不适用。

### 5.2 在其他形状路径上测试的结果

#### 5.2.1 圆形路径

因为直接用圆形不太方便，这里写成一个正n边形的形式，效果应该接近

```matlab
% generate a n-side regular polygon trajectory
n = 50;
nlist = 1:n;

EulZYXpoints2 = repmat([0 0 -pi],[n,1]);

waypoints_x = arrayfun(@cos,nlist)';
waypoints_y = arrayfun(@sin,nlist)';
waypoints_z = ones(1,n) * 0.6500;
waypoints_z = waypoints_z';

waypoints2 = [waypoints_x,waypoints_y,waypoints_z];

waypointsTime = repmat([2 2 2]',[1,n]);
```

改圆形路径的时候出现报错，后来发现这里==没有改机械臂初始姿态==；目前这部分尚未完成

![image-20240402085037236](C:\Users\Jacob Patrick\AppData\Roaming\Typora\typora-user-images\image-20240402085037236.png)

这里还有个==小问题==是没找到 `EulZYXpoints2` `waypoints2` `waypointsTime` 是模型 `mdl` 里哪个模块的输入

#### 5.2.2 不规则路径

不规则路径如下

```matlab
% generate a random polyline with n nodes
n = 8;
waypoints_x = rand(1,n) * 6 - 3;
waypoints_y = rand(1,n) * 6 - 3;
waypoints_z = ones(1,n) * 0.6500;

waypoints_x = waypoints_x';
waypoints_y = waypoints_y';
waypoints_z = waypoints_z';

waypoints2 = [waypoints_x,waypoints_y,waypoints_z];
```

同样出现如上报错

## 六、下一步实验计划

1. 仍然尝试用智能体调节pid参数的方式，将DDPG智能体换成TD3，SAC等，比较控制效果；
2. 或许可以试试去掉pid控制器，目前还没考虑好怎么做。
