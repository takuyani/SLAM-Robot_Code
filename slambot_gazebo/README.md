# slambot_gazebo
Slambot gazebo package

## 目次  
- 1.&nbsp;[Overview](#1-overview)  
- 2.&nbsp;[Launches](#2-launches)  
  - 2.1.&nbsp;[vehicle_controller_sim.launch](#21-vehicle_controller_simlaunch)  
  - 2.2.&nbsp;[display.launch](#22-displaylaunch)  
        
## 1. Overview
本パッケージは[slambot_description](https://github.com/takuyani/SLAM-Robot_Code/tree/develop/slambot_description)にて作成したロボットモデルの動力学シミュレータが含まれます。  
Gazeboにてシミュレートすることができます。  
また、[slambot_teleop](https://github.com/takuyani/SLAM-Robot_Code/tree/develop/slambot_teleop)を使用することでロボットモデルへ制御指令（並進速度、回転速度）を送ることができます。

<シミュレーション実行例>（画像をクリックするとYouTubeへジャンプします。）  
[![youtube](http://img.youtube.com/vi/UocKXpD_KqU/0.jpg)](http://www.youtube.com/watch?v=UocKXpD_KqU)

<ROS ノードグラフ>  
![rviz](https://c1.staticflickr.com/5/4782//39963598914_466689a694_o.png)

__【依存パッケージ】__
 * [slambot_description](https://github.com/takuyani/SLAM-Robot_Code/tree/develop/slambot_description)
 * [slambot_teleop](https://github.com/takuyani/SLAM-Robot_Code/tree/develop/slambot_teleop)
<br>

## 2. Launches
### 2.1. vehicle_controller_sim.launch
Gazeboシミュレーターが立ち上がります。

### 2.2. display.launch
Rvizにてモデルを可視化。(確認用)

## <参考>
- [Gazebo + ROS で自分だけのロボットをつくる](https://qiita.com/RyodoTanaka/items/c3014fd6d0f06d12814f)
- [ROSで始めるロボティクス（4） ー シミュレータ上でロボットを動かしてみる](http://bril-tech.blogspot.jp/2016/10/ros4.html)
