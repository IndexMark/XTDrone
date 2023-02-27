
# XTDrone
**[项目源地址](https://github.com/robin-shaun/XTDrone)
[gitee](https://gitee.com/robin_shaun/XTDrone)
[XTDrone使用文档](https://www.yuque.com/xtdrone/manual_cn)**


# USTB ME 814修改版本
**环境**：
* Ubuntu 18.04 LTS
* ROS Melodic

修改了**旋翼键盘控制程序**描述问题，以及**固定翼**在未给航点情况下弹出警告的问题  ~~小问题可以不改~~

该版本在XTDrone基础上,添加了
  * 给*Gazebo*添加了更改后的`UWB`插件([原版本](https://github.com/valentinbarral/gazebosensorplugins))
  * 带`单目摄像头`和`UWB`的**旋翼**与**固定翼**模型

话不多说，这就来装
## 安装教程
主要参考 **[XTDrone使用文档](https://www.yuque.com/xtdrone/manual_cn)**，出问题了可以回到原文档看看`PX4 1.13版本
安装教程`

注：安装默认在**根目录`~/`**，若安装在**其他地方**后续指令要作相应的更改

报错请**百度**或参考**原文档**

### 一、依赖安装
打开终端(Ctrl + T)，在终端中粘贴(Ctrl + Shift + V)下列指令，回车进行安装

```
sudo apt install ninja-build exiftool ninja-build protobuf-compiler libeigen3-dev genromfs xmlstarlet libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python-pip python3-pip gawk
```
```
pip2 install pandas jinja2 pyserial cerberus pyulog==0.7.0 numpy toml pyquaternion empy pyyaml 
pip3 install packaging numpy empy toml pyyaml jinja2 pyargparse kconfiglib jsonschema future
```
### 二、ROS安装
安装**Ubuntu**对应**ROS**版本即可

网上的方法很多，推荐先参照[ROS官方](http://wiki.ros.org/melodic/Installation/Ubuntu)(Melodic)

出现问题利用好**搜索工具**，ROS安装问题以及解决方法网上有很多 ~~开始受苦~~

>* 当你的**小海龟**动起来后，恭喜你，**ROS**安装成功

让我们来创建一个**ROS**工作空间**xtdrone_ws**，并初始化，为后面做准备

打开一个新终端
```
mkdir -p ~/xtdrone_ws/src
cd ~/xtdrone_ws/src
catkin_init_workspace
```
### 三、Gazebo安装
Gazebo包括Gazebo本身和ROS的插件，需要分别安装。首先卸载之前的Gazebo
```
sudo apt-get remove gazebo* 
sudo apt-get remove libgazebo*
sudo apt-get remove ros-melodic-gazebo* 
```
XTDrone对Gazebo的ROS插件做了修改，因此需要源码编译。

#### 首先装依赖
```
sudo apt-get install ros-melodic-moveit-msgs ros-melodic-object-recognition-msgs ros-melodic-octomap-msgs ros-melodic-camera-info-manager  ros-melodic-control-toolbox ros-melodic-polled-camera ros-melodic-controller-manager ros-melodic-transmission-interface ros-melodic-joint-limits-interface
```
**然后安装[*Gazebo*](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)**(Alternative installation: step-by-step的安装方式)

> **注意**：
>1. 如果安装有依赖问题，可以使用`sudo aptitude install gazebo11`，选择合理的依赖解决办法(别删ROS)
>2. 按步骤装完Gazebo后，升级所有的包 sudo apt upgrade，这样能保证gazebo所有依赖版本一致

___
#### **Gazebo安装**(以下内容同与上面的链接相同，以应对Gazebo官网打不开的情况)

##### 1. Setup your computer to accept software from packages.osrfoundation.org.
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```
You can check to see if the file was written correctly. For example, in Ubuntu Bionic (18.04), you can type:
```
cat /etc/apt/sources.list.d/gazebo-stable.list
```
And if everything is correct, you should see:
```
deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main
```
##### 2. Setup keys
```
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
##### 3. Install Gazebo.
First update the debian database:
```
sudo apt-get update
```
Hint: make sure the apt-get update process ends without any errors, the console output ends in `Done` similar to below:
```
$ sudo apt-get update
...
Hit http://ppa.launchpad.net bionic/main Translation-en
Ign http://us.archive.ubuntu.com bionic/main Translation-en_US
Ign http://us.archive.ubuntu.com bionic/multiverse Translation-en_US
Ign http://us.archive.ubuntu.com bionic/restricted Translation-en_US
Ign http://us.archive.ubuntu.com bionic/universe Translation-en_US
Reading package lists... Done
```
Next install gazebo-11 by:
```
sudo apt-get install gazebo11
# For developers that work on top of Gazebo, one extra package
sudo apt-get install libgazebo11-dev
```
##### 4. Check your installation
```
gazebo
```
___

#### 安装ROS插件
**XTDrone**对Gazebo的ROS插件做了修改，因此需要源码编译。

首先装依赖
```
sudo apt-get install ros-melodic-moveit-msgs ros-melodic-object-recognition-msgs ros-melodic-octomap-msgs ros-melodic-camera-info-manager  ros-melodic-control-toolbox ros-melodic-polled-camera ros-melodic-controller-manager ros-melodic-transmission-interface ros-melodic-joint-limits-interface
```
然后克隆并编译Gazebo的插件gazebo_ros_pkgs，放在`~/xtdrone_ws/src`下（如果编译时还缺其他的依赖，同上方法安装）
```
cd ~/xtdrone_ws/src
git clone -b melodic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd ..
catkin_make
```
编译成功后在两个终端分别执行如下两条指令，判断gazebo_ros是否安装成功
```
roscore 
```
```
source ~/xtdrone_ws/devel/setup.bash
rosrun gazebo_ros gazebo
```
Gazebo有很多开源的模型文件，一些需要的模型文件上传到了附件中，供下载：

--> 请点击[📎models.zip](https://www.yuque.com/attachments/yuque/0/2022/zip/985678/1670494695523-2a18624d-545e-4552-a3b1-714ac5d649c2.zip)

将该附件解压缩后放在`~/.gazebo`中，此时在`~/.gazebo/models/`路径下可以看到很多模型。如果不做这一步，之后运行Gazebo仿真，可能会缺模型，这时会自动下载，Gazebo模型服务器在国外，自动下载会比较久。
### 四、MAVROS安装
注意，mavros-extras一定别忘记装，否则视觉定位将无法完成
```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras 
wget https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh

sudo chmod a+x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh #这步需要装一段时间
```
### 五、PX4配置
#### 1.下载&编译
___
以下给出推荐配置,更多PX4仿真配置,见[PX4仿真文档](https://dev.px4.io/master/en/simulation/)

可以直接下载XTDrone的压缩包[📎PX4_Firmware.zip](https://www.yuque.com/attachments/yuque/0/2022/zip/985678/1672148620266-ad2f680e-e9ec-4be0-9746-914552829832.zip)，解压后记得删除原有的`build`文件再进行编译

或者下我上传的这个已经删掉`build`文件的版本[百度网盘]()
___
下载好后，解压放在` ～/`目录下(打开终端，输入`pwd`，显示的就是` ～/`目录)

打开终端，进入文件夹进行编译
```
cd PX4_Firmware
make px4_sitl_default gazebo
```
编译完成后，会弹出Gazebo界面，将其关闭即可
#### 2.添加环境变量
修改 ~/.bashrc
```
gedit .bashrc
```
在**文档最后**加入以下代码，注意路径匹配，前两个source顺序不能颠倒，一个是**工作空间**，一个是**PX4**
```
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
```
再运行：
```
source ~/.bashrc
```
然后运行如下命令，此时会启动Gazebo，会有相应界面显示

并在新终端执行
```
rostopic echo /mavros/state
```
若connected: True,则说明MAVROS与SITL通信成功

如果是false，一般是因为.bashrc里的路径写的不对，请仔细检查
```
---
header:
seq: 11
stamp:
secs: 1827
nsecs: 173000000
frame_id: ''
connected: True
armed: False
guided: False
manual_input: True
mode: "MANUAL"
system_status: 3
---
```

**(实际这里暂不影响目前的功能使用，可先跳过，后续走不通再回来检查)**
#### 3.安装地面站QGroundControl
点此[安装链接](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)
___
#### QGroundControl安装(同上面的链接，以应对打不开的情况)
**Ubuntu Linux**

_QGroundControl_ can be installed/run on Ubuntu LTS 20.04 (and later).

Ubuntu comes with a serial modem manager that interferes with any robotics related use of a serial port (or USB serial). Before installing _QGroundControl_ you should remove the modem manager and grant yourself permissions to access the serial port. You also need to install _GStreamer_ in order to support video streaming.

Before installing _QGroundControl_ for the first time:

1.  On the command prompt enter:
    
    ```
    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libqt5gui5 -y
    sudo apt install libfuse2 -y
    ```
    
2.  Logout and login again to enable the change to user permissions.

To install _QGroundControl_:

1.  Download [QGroundControl.AppImage](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage).(备用:[百度网盘]())
2.  Install (and run) using the terminal commands:
    
    ```
    chmod +x ./QGroundControl.AppImage
    ./QGroundControl.AppImage  (or double click)
    ```
___
### 六、XTDrone源码下载(USTB ME 814版本)
#### 1. 打开一个新终端
	```
	git clone -b ustb-demo https://github.com/IndexMark/XTDrone.git
	cd XTDrone
	git submodule update --init --recursive
	# 修改启动脚本文件
	cp sitl_config/init.d-posix/* ~/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/
	# 添加launch文件
	cp -r sitl_config/launch/* ~/PX4_Firmware/launch/
	# 添加世界文件
	cp sitl_config/worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds/
	# 修改部分模型文件
	cp -r sitl_config/models/* ~/PX4_Firmware/Tools/sitl_gazebo/models/ 
	# 替换同名文件
	cd ~/.gazebo/models/
	rm -r stereo_camera/ 3d_lidar/ 3d_gpu_lidar/ hokuyo_lidar/
	```
因为Gazebo寻找模型的顺序，是先在`~/.gazebo/models/`下寻找，然后在其他路径寻找，所以在往PX4 SITL复制models时，要注意`~/.gazebo/models/`下有没有同名文件（比如`~/.gazebo/models/`下默认有stereo_camera），有的话要么将该同名文件删去，要么替换该同名文件。

如果在仿真过程中出现了某个部件为白色方块的情况，大概率是因为`~/.gazebo/models/`下有和`PX4_Firmware/Tools/sitl_gazebo/models`重复的模型，此时删掉`~/.gazebo/models/`下的模型文件即可

* 这一步添加了带**多个单目摄像头**以及**UWB**的模型.sdf文件，以及一个带9个彩墙的世界

#### 2. 打开一个新终端，重新编译PX4固件
```
cd ~/PX4_Firmware
make px4_sitl_default gazebo
```
编译好后会出现Gazebo界面，关闭即可

### 七、UWB插件安装

