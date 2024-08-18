@[TOC](文章目录)

---

# 简介

本项目基于ros和gazebo仿真，通过在rviz中3D Nav Goal设定目标点，目标点个数为3时，通过minimum snap算法规划一次轨迹，px4Ctrl订阅并接收轨迹消息，从而控制四旋翼飞行器完成一次飞行，到达目的地后，可再次设置目标点。

---


# 一、环境配置

基本环境：Ubuntu20.04 + ROS noetic + gazebo11 + mavros + PX4
部分依赖库：Eigen 3.3.7

# 二、使用步骤
## 1.编译代码

```c
cd catkin_ws
catkin_make
```

## 2.启动gazebo和PX4仿真

新开一个终端，运行PX4飞控的launch文件。
```c
roslaunch px4 mavros_posix_sitl.launch
```
## 3.启动px4Ctrl和rqt_reconfigure

由于仿真没有遥控设备，所以使用rqt_reconfigure工具来切换px4Ctrl的工作模式。更多关于px4Ctrl的具体信息请查看[https://gitee.com/jerry-ironman/px4ctrl](https://gitee.com/jerry-ironman/px4ctrl)。
```c
cd catkin_ws
source devel/setup.bash
sh px4ctrl.sh
```
开启QGroundControl

## 4
source一下新终端
roslaunch a_star astar.launch

## 5
source devel/setup.bash
rosrun a_star keyboard_input_node
有范围限制

########################################################3


## 4.切换px4Ctrl到命令模式

开启QGroundControl，确保mavros与QGroundControl通信正常。点击左上角的"Ready to fly"，点击"Arm"解锁飞行器，切换到rqt_reconfigure，点击mode_bool切换到自动悬停模式，再点击cmd_bool切换到命令模式，在启动px4Ctrl的终端可以看到模式切换的提示。

rosrun rqt_reconfigure rqt_reconfigure

## 5.启动轨迹生成节点和rviz

```c
cd catkin_ws
source devel/setup.bash
sh minisnap.sh
```

## 6.发布目标点

在rviz中，选择3D Nav Goal或按下"G"键，即可在图中设置目标点的位置。在左键按下的同时按下右键，并拖动鼠标，即可设置目标点的高度。

---

# 三、效果演示

