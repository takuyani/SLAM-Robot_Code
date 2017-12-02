# vehicle_controller
Vehicle controller package

## 目次  
- 1.&nbsp;[Overview](#1-overview)  
- 2.&nbsp;[Nodes](#2-nodes)  
  - 2.1.&nbsp;[vehicle_controller_node](#21-vehicle_controller_node)  
    - 2.1.1.&nbsp;[Subscribed Topics](#211-subscribed-topics)  
    - 2.1.2.&nbsp;[Published Topics](#212-published-topics)  
    - 2.1.3.&nbsp;[Parameters](#213-parameters)  
        
## 1. Overview
本パッケージは対向２輪型の車体動作を制御する「vehicle_controller」ノードが含まれています。
対向２輪型とは２個の車輪を左右対称にロボットに取り付け、それぞれを独立に駆動するようにしたものです。

|図1|図2|
|:--:|:--:|
|![parallel_two_wheel_vehicle_01](https://farm5.staticflickr.com/4275/34410303220_19fae79e7f_b.jpg)|![parallel_two_wheel_vehicle_02](https://farm5.staticflickr.com/4166/33986938193_790790a94e_b.jpg)|

## 2. Nodes
### 2.1. vehicle_controller_node
車体の運動制御を行うノードです。入力信号は並進速度[m/s]と回転速度[rad/s]であり、Topicのcmd_velによって与えられます。  
ここで、両輪の回転速度の平均を並進速度、両輪の回転速度の差分を回転速度と呼ぶことにします。  
並進速度は前進で正の値、後退で負の値とする。  
回転速度は、反時計回りのときに正の値、時計回りの時に負の値とする。  
並進、回転の速度が決まると、モータの回転速が算出される。  

**式：**  
- ωr = V/R + W\*T/(2\*R)  
- ωl = V/R - W\*T/(2\*R)  

ωr : 右車輪角速度(angular velocity of right wheel)[rad/s]  
ωl : 左車輪角速度(angular velocity of left wheel)[rad/s]  
V  : 並進速度[m/s]  
W  : 回転速度[rad/s]  
R  : 車輪半径(wheel radius)[m]  
T  : 車輪幅(tread width)[m]  

以下は車体の動作例となります。  

**例：**  
- 並進速度 =  10, 回転速度 =   0 ⇒  10[m/s]で直進  
- 並進速度 = -10, 回転速度 =   0 ⇒ -10[m/s]で後退  
- 並進速度 =   0, 回転速度 =  10 ⇒  10[rad/s]で右回転  
- 並進速度 =   0, 回転速度 = -10 ⇒  10[rad/s]で左回転  
- 並進速度 =  10, 回転速度 =  10 ⇒ 並進速度10[m/s], 回転速度10[rad/s]で右旋回  
- 並進速度 =  10, 回転速度 = -10 ⇒ 並進速度10[m/s], 回転速度10[rad/s]で左旋回  

<br>

**■フェールセーフ**  
フェールセーフ機能として、本ノードとトピック送信元のノードとの間に通信途絶が発生した場合、車体を緊急停止させます。
通信途絶はSubscribed Topicsの**cmd_vel**受信間隔で判定される。

#### 2.1.1. Subscribed Topics
- **cmd_vel**([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  - linear.x: 並進速度[m/s]（+：前進, -：後退）
  - linear.y: 未使用
  - linear.z: 未使用
  - angular.x: 未使用
  - angular.y: 未使用
  - angular.z: 回転速度[rad/s]（+：左, -：右）

#### 2.1.2. Published Topics
- **alive_resp**([geometry_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))  
  Subscribed Topic "**cmd_vel**"の受信応答。車体の状態を送信する。  
  ・ true: 車体制御可能状態  
  ・ false: 車体制御不可能状態  

#### 2.1.3. Parameters
- **~cmd_vel_timeout**(double, default: 1.0)  
  Subscribed Topicsの**cmd_vel**正常受信タイムアウト時間[s]。  
  **cmd_vel**を正常に受信してから、タイムアウト時間内に**cmd_vel**を正常に受信できなかった場合、車体に停止信号を送信する。

- **~polling_rate**(double, default: 10.0)  
  車体の状態をチェックするポーリングレート[Hz]。  
  車体に異常等が発生した場合のリカバリーモードはこの周期で実行される。

- **~wheel_radius**(double, default: 0.01)  
  車輪半径[m]

- **~tread_width**(double, default: 0.01)  
  車輪幅[m]

- **~debug/enable**(bool, default: false)  
  デバッグモード有効フラグ。trueの場合、デバッグモードとなり、本ノードのデバッグを行うことが可能。  
  開発用のParameterであり、普段はfalseに設定しておくこと。

## <参考>
- [並進、回転の制御について](https://hyakuren-soft.fogbugz.com/?W46)  
- [車輪移動ロボット](http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html)
