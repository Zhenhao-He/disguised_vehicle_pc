# 1、环境设置（伪装车的机器环境已经装好）
## 1.1、创建虚拟环境

ubutnu22.04对应ROS2的Humble版本，Humble版本对应的python版本为3.10.12

```cpp
$ conda create -n ros2_parking python=3.10.12
```

## 1.2、安装ROS2系统

利用mamba进行安装，后面加载虚拟环境后，不用source也可以发现ROS2系统自带的功能包，更加方便

```cpp
$ conda install mamba -c conda-forge

$ conda config --env --add channels conda-forge
$ conda config --env --add channels robostack-staging
$ conda config --env --remove channels defaults

$ mamba install ros-humble-desktop
$ mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

## 1.3、安装pytorch

python版本为3.10.12最低支持torch==1.11.0，如果有高版本的需要，查看对应关系

https://pytorch.org/get-started/previous-versions/

```cpp
$ pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 torchaudio==0.11.0 --extra-index-url https://download.pytorch.org/whl/cu113
```


## 1.4、安装其他依赖包

```cpp
$ mamba install  proxsuite
$ pip install pandas
$ pip install  jinja2 park
$ mamba install  typeguard
$ mamba install geographiclib-cpp
$ pip install  IPython
$ pip install  tqdm
$ pip install  seaborn

$ conda install conda-forge::geographiclib-cpp


#如果出现ModuleNotFoundError: No module named 'distutils'
$ pip install --upgrade setuptools
```


# 2、新建编译工作空间
## 2.1 clone代码-新建总工作空间
```cpp
$ git clone git@github.com:Zhenhao-He/disguised_vehicle_ipc.git
```
包含2个两个子工作空间


## 2.2、新建编译rviz工作空间
注意：必须新建终端，此工作空间基于/opt/ros/humble/setup.bash
```cpp
$ cd rviz 
$ conda deactivate
$ conda deactivate
$ source /opt/ros/humble/setup.bash
$ colcon build  --cmake-args -DCMAKE_BUILD_TYPE=Release
$ [可选]colcon build --packages-select <package_name1> <package_name2> ...
```
## 2.3 新建编译wuling_autoware工作空间
注意：必须新建终端，此工作空间基于conda虚拟环境ros2_parking
```cpp
$ cd wuling_autoware  
$ 从百度云盘下载权重文件 链接: https://pan.baidu.com/s/1W3nmdGf-RyMbtOaaQkbNpw 提取码: hkx5 放在文件夹“src/sperception/parkingslot_detection/parkingslot_detection/ckpt”里面
$ sh copy_zlgcan.sh//安装周立功到虚拟环境
$ cd ..//回到wuling_autoware文件夹

$ colcon build  
$ [可选]colcon build --packages-select <package_name1> <package_name2> ...

```

# 3、运行
## 3.1 进入rviz工作空间，进行可视化
```cpp
$ cd rviz  
$ sudo su  
$ source install/setup.bash
$ ros2 launch rviz_launch launch.py #启动可视化
```

## 3.1 进入wuling_autoware工作空间，启动各个算法模块
```cpp
$ cd wuling_autoware  
$ sudo su  
$ source install/setup.bash
$ ros2 launch autoware_launch simulator_launch.py #选择1:虚拟动力仿真
$ ros2 launch autoware_launch real_launch.py #选择2:实车调试
```
## 3.1 进入wuling_autoware工作空间，使能自动驾驶
```cpp
$ cd wuling_autoware  
$ sudo su  
$ source install/setup.bash
$ ros2 launch system_launch launch.py
```