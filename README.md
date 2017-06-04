# vehicle_controller
Vehicle controller package

## 目次  
- 1.&nbsp;[Overview](#1-overview)  
- 2.&nbsp;[Nodes](#2-nodes)  
  - 2.1.&nbsp;[vehicle_controller](#21-vehicle_controller)  
    - 2.1.1.&nbsp;[Subscribed Topics](#211-subscribed-topics)  
    - 2.1.2.&nbsp;[Published Topics](#212-published-topics)  
    - 2.1.3.&nbsp;[Parameters](#213-parameters)  
  - 2.2.&nbsp;[teleope_velocity.py](#22-teleope_velocitypy)  
- 3.&nbsp;[Launch files](#3-launch-files)  
        
## 1. Overview
本パッケージは対向２輪型の車体動作を制御する「vehicle_controller」ノードが含まれています。
対向２輪型とは２個の車輪を左右対称にロボットに取り付け、それぞれを独立に駆動するようにしたものです。

|図1|図2|
|:--:|:--:|
|![parallel_two_wheel_vehicle_01](https://farm5.staticflickr.com/4275/34410303220_19fae79e7f_b.jpg)|![parallel_two_wheel_vehicle_02](https://farm5.staticflickr.com/4166/33986938193_790790a94e_b.jpg)|

本パッケージは以下で制作された対向２輪型移動ロボットを対象としています。  
[Assembly_Manual_JP.md](https://github.com/takuyani/SLAM-Robot_Docs/blob/master/Hardware/Assembly_Manual/Assembly_Manual_JP.md)

## 2. Nodes
### 2.1. vehicle_controller
Topicのcmd_velによって与えられた並進速度[mm/s]と回転速度[deg/s]に基づいて、対向２輪型移動ロボットが動作します。  
例：  
- 並進速度 = 10, 回転速度 = 0 ⇒ 10[mm/s]で直進
- 並進速度 = 0, 回転速度 = 10 ⇒ 10[deg/s]で右回転
- 並進速度 = 10, 回転速度 = 10 ⇒ 並進速度10[mm/s], 回転速度10[deg/s]で右旋回

#### 2.1.1. Subscribed Topics
- cmd_vel([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  - linear.x: 並進速度[mm/s]（+：前進, -：後退）
  - linear.y: 未使用
  - linear.z: 未使用
  - angular.x: 未使用
  - angular.y: 未使用
  - angular.z: 回転速度[deg/s]（+：左, -：右）

#### 2.1.2. Published Topics
なし
#### 2.1.3. Parameters
- __wheel_radius__(double, default: 10)  
   車輪半径[mm]

- __tread_width__(double, default: 10)  
   車輪幅[mm]

### 2.2. teleope_velocity.py
## 3. Launch files
