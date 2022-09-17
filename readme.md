# Paper Learning

[TOC]

## 1、概述

本git仓库用于记录本人的论文学习，主要包含的是论文中的一些数学推导和本人的一些见解。

## 2、具体文件目录

1.[惯导算法介绍](./ins)

主要参考武汉大学本科生课程《惯性导航》课件，包含惯导算法中的一些重要公式，如速度、姿态、位置更新公式及其误差微分方程。

2.[李代数理论在姿态优化中的应用](./lie-group)

主要参考论文：《A micro Lie theory for state estimation in robotics》。其介绍了一些关于李代数的简单但有用的知识。最主要的是在位姿优化中，如何对姿态进行求导，涉及到了左扰动和右扰动。其结论可以作为日常使用的公式。

3.[李师兄的毕业论文](./lishengyu)

其推导和实现了IMU和GNSS和视觉的多元融合，基于扩展卡尔曼滤波，其通过INS进行状态的预测，并在之后使用视觉测量和GNSS测量对状态进行更新。另外，在后续的研究中，也将激光雷达纳入到该系统中，基于面特征来进行雷达观测值的处理。

4.[MSCKF_VIO](./Robust-Stereo-Visual-Inertial-Odometry-for-Fast-Autonomous-Flight)

视觉+IMU进行的多源融合算法。其通过IMU来进行状态预测，以更好的跟踪前后相机帧上的特征点。其在解算时，将量测方程中的特征点状态投影到零空间（将其边缘化），以加速求解。

5.[Targetless_Calibration_of_LiDAR-IMU_System_Based_on_Continuous-time_Batch_Estimation](./Targetless_Calibration_of_LiDAR-IMU_System_Based_on_Continuous-time_Batch_Estimation)

本 次 论 文 为： 《Targetless Calibration of LiDAR-IMU System Based on Continuous-time Batch Estimation》，其是一篇关于传感器标定的文章。其使用 B 样条曲线，在连续时间上对传感器的轨迹进行建模 (主要是 IMU 和激光雷达)。其不需要靶标，适应的场景较多。

6.[The_Normal_Distributions_Transform_A_New_Approach_to_Laser_Scan_Matching](./The_Normal_Distributions_Transform_A_New_Approach_to_Laser_Scan_Matching)

对于激光里程计而言，帧间匹配对于位姿估计是比较重要的。一般的帧间匹配有点到点的、点到线的、点到面的(三维空间)。其基本的步骤都是需要先进行数据关联，而后构建损失函数进行优化。而本次论文里的NDT方法，不需要进行数据关联，其通过将点云格网化，并建立对应的正太分布模型，在进行帧间位姿估计。