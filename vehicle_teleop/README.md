# vehicle_teleop
Vehicle Teleoperation package

## 目次  
- 1.&nbsp;[Overview](#1-overview)  
- 2.&nbsp;[Nodes](#2-nodes)  
  - 2.1.&nbsp;[slambot_teleop_keyboard.py](#21-slambot_teleop_keyboardpy)  
    - 2.1.1.&nbsp;[Subscribed Topics](#211-subscribed-topics)  
    - 2.1.2.&nbsp;[Published Topics](#212-published-topics)  
    - 2.1.3.&nbsp;[Parameters](#213-parameters)  
  - 2.2.&nbsp;[slambot_teleop_ps4joy_node](#22-slambot_teleop_ps4joy_node)  
    - 2.2.1.&nbsp;[Subscribed Topics](#221-subscribed-topics)  
    - 2.2.2.&nbsp;[Published Topics](#222-published-topics)  
    - 2.2.3.&nbsp;[Parameters](#223-parameters)  
    
## 1. Overview
本パッケージは対向２輪型の車体に制御命令を与えるノードです。制御命令は並進速度[m/s]と回転速度[rad/s]です。
キーボードまたは、ジョイスティックにて車体制御が可能となっています。
また、フェールセーフ機能として、本ノードとトピック送信先のノードとの間に通信途絶が発生した場合、車体は緊急停止を行います。
[vehicle_controller](https://github.com/takuyani/SLAM-Robot_Code/tree/develop/vehicle_controller)との通信を想定したパッケージとなります。

## 2. Nodes
### 2.1. slambot_teleop_keyboard.py
[teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)を改良し、[vehicle_controller](https://github.com/takuyani/SLAM-Robot_Code/tree/develop/vehicle_controller)との通信を想定したパッケージとなります。
キーボード操作により、車体制御が可能。送信トピックの単位が並進速度[m/s]、回転速度[rad/s]となっています。

#### 2.1.1. Subscribed Topics
　なし

#### 2.1.2. Published Topics
- __cmd_vel__([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  - linear.x: 並進速度[m/s]（+：前進, -：後退）
  - linear.y: 未使用
  - linear.z: 未使用
  - angular.x: 未使用
  - angular.y: 未使用
  - angular.z: 回転速度[rad/s]（+：左, -：右）

#### 2.1.3. Parameters
- __~speed__(double, default: 0.1)  
  並進速度初期値[m/s]

- __~turn__(double, default: 10.0*π/180.0)  
  回転速度初期値[rad/s]

<br>  

### 2.2. slambot_teleop_ps4joy_node
ジョイスティック入力値([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))をcmd_vel([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))に変換するノードです。  
ジョイスティック操作により、車体制御が可能となっています。

#### 2.2.1. Subscribed Topics
- __joy__([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))  
  ジョイスティック入力値

#### 2.2.2. Published Topics
- __cmd_vel__([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
  - linear.x: 並進速度[m/s]（+：前進, -：後退）
  - linear.y: 未使用
  - linear.z: 未使用
  - angular.x: 未使用
  - angular.y: 未使用
  - angular.z: 回転速度[rad/s]（+：左, -：右）

#### 2.2.3. Parameters
　なし
 
 ## <参考>
- [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)  
