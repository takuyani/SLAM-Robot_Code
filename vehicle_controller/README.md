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
    - 2.2.1.&nbsp;[Subscribed Topics](#221-subscribed-topics)  
    - 2.2.2.&nbsp;[Published Topics](#222-published-topics)  
    - 2.2.3.&nbsp;[Parameters](#223-parameters)  

        
## 1. Overview
本パッケージは対向２輪型の車体動作を制御する「vehicle_controller」ノードが含まれています。
対向２輪型とは２個の車輪を左右対称にロボットに取り付け、それぞれを独立に駆動するようにしたものです。

|図1|図2|
|:--:|:--:|
|![parallel_two_wheel_vehicle_01](https://farm5.staticflickr.com/4275/34410303220_19fae79e7f_b.jpg)|![parallel_two_wheel_vehicle_02](https://farm5.staticflickr.com/4166/33986938193_790790a94e_b.jpg)|

## 2. Nodes
### 2.1. vehicle_controller
車体の運動制御を行うノードです。入力信号は並進速度[mm/s]と回転速度[deg/s]であり、Topicのcmd_velによって与えられます。  
ここで、両輪の回転速度の平均を並進速度、両輪の回転速度の差分を回転速度と呼ぶことにします。並進速度は前進で正の値、後退で負の値とする。
回転速度は、反時計回りのときに正の値、時計回りの時に負の値とする。
並進、回転の速度が決まると、モータの回転速が算出される。

__式：__  
ωr = V/R + W\*T/(2\*R)  
ωl = V/R - W\*T/(2\*R)  

ωr : 右車輪角速度(angular velocity of right wheel)[deg/s]  
ωl : 左車輪角速度(angular velocity of left wheel)[deg/s]  
V  : 並進速度[mm/s]  
W  : 回転速度[deg/s]  
R  : 車輪半径(wheel radius)[mm]  
T  : 車輪幅(tread width)[mm]  

以下は車体の動作例となります。  

__例：__  
- 並進速度 =  10, 回転速度 =   0 ⇒  10[mm/s]で直進  
- 並進速度 = -10, 回転速度 =   0 ⇒ -10[mm/s]で後退  
- 並進速度 =   0, 回転速度 =  10 ⇒  10[deg/s]で右回転  
- 並進速度 =   0, 回転速度 = -10 ⇒  10[deg/s]で左回転  
- 並進速度 =  10, 回転速度 =  10 ⇒ 並進速度10[mm/s], 回転速度10[deg/s]で右旋回  
- 並進速度 =  10, 回転速度 = -10 ⇒ 並進速度10[mm/s], 回転速度10[deg/s]で左旋回  

#### 2.1.1. Subscribed Topics
- __cmd_vel__([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  - linear.x: 並進速度[mm/s]（+：前進, -：後退）
  - linear.y: 未使用
  - linear.z: 未使用
  - angular.x: 未使用
  - angular.y: 未使用
  - angular.z: 回転速度[deg/s]（+：左, -：右）
  
#### 2.1.2. Published Topics
  なし
#### 2.1.3. Parameters
- __~wheel_radius__(double, default: 10)  
   車輪半径[mm]

- __~tread_width__(double, default: 10)  
   車輪幅[mm]

- __~debug/enable__(bool, default: false)  
   デバッグモード有効フラグ。trueの場合、デバッグモードとなり、本ノードのデバッグを行うことが可能。  
   開発用のParameterであり、普段は使用しない。

### 2.2. teleope_velocity.py
[teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)のPublished Topicsを並進速度[mm/s]、回転速度[deg/s]に変換するためのインターフェース用ノード。以下の式に基づいて変換される。

__式：__  
並進速度[mm/s] = 基準並進速度[mm/s] \* 並進速度ゲイン  
回転速度[deg/s] = 基準回転速度[deg/s] \* 回転速度ゲイン  

#### 2.2.1. Subscribed Topics
- __src/cmd_vel__([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  - linear.x: 並進速度ゲイン
  - linear.y: 未使用
  - linear.z: 未使用
  - angular.x: 未使用
  - angular.y: 未使用
  - angular.z: 回転速度ゲイン

#### 2.2.2. Published Topics
- __dst/cmd_vel__([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  - linear.x: 並進速度[mm/s]
  - linear.y: 未使用
  - linear.z: 未使用
  - angular.x: 未使用
  - angular.y: 未使用
  - angular.z: 回転速度[deg/s]

#### 2.2.3. Parameters
- __~linear__(double, default: 10)  
   基準並進速度[mm/s]

- __~angular__(double, default: 10)  
   基準回転速度[deg/s]

## <参考>
- [並進、回転の制御について](https://hyakuren-soft.fogbugz.com/?W46)  
- [車輪移動ロボット](http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html)
