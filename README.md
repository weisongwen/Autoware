# Autoware for autonomous vehicle

Software for urban autonomous driving, initially developed by [Tier IV](http://www.tier4.jp).
This package is now customized for egg vehicle. The following functions are supported:

- 3D Localization
- 3D Mapping
- Path Following
- Motion/Steering Control
- Object Detection

## Spec Recommendation

- Number of CPU cores: 8
- RAM size: 32GB
- Storage size: 30GB

## Requirements

- ROS jade (Ubuntu 14.04)
- OpenCV 2.4.10 or higher
- Qt 5.2.1 or higher
- CUDA(Optional)
- FlyCapture2 (Optional)
- Armadillo (Optional)

### Install dependencies for Ubuntu 14.04 indigo

```
% sudo apt-get install ros-jade-desktop-full ros-jade-nmea-msgs ros-jade-nmea-navsat-driver ros-jade-sound-play ros-jade-jsk-visualization ros-jade-grid-map ros-jade-gps-common
% sudo apt-get install ros-jade-controller-manager ros-jade-ros-control ros-jade-ros-controllers ros-jade-gazebo-ros-control ros-jade-sicktoolbox ros-jade-sicktoolbox-wrapper ros-jade-joystick-drivers ros-jade-novatel-span-driver
% sudo apt-get install libnlopt-dev freeglut3-dev qtbase5-dev libqt5opengl5-dev libssh2-1-dev libarmadillo-dev libpcap-dev gksu libgl1-mesa-dev libglew-dev
```

**NOTE: Please do not install ros-indigo-velodyne-pointcloud package. Please uninstall it if you already installed.**


**NOTE: Following packages are not supported in ROS Kinetic.**
- gazebo
- orb slam
- dpm ocv

## How to Build

```
$ cd $HOME
$ git clone https://github.com/weisongwen/Autoware.git
$ cd ~/Autoware/ros/src
$ catkin_init_workspace
$ cd ../
$ ./catkin_make_release
```

## How to Start

```
$ cd $HOME/Autoware/ros
$ ./run
```

## How to use this for Egg Vehicle

please refer to the video (not available)

[![Public Road Demonstration](http://img.youtube.com/vi/5DaQBZvZwAI/mqdefault.jpg)](https://www.youtube.com/watch?v=5DaQBZvZwAI)

Steps:
- 1. Open the LiDAR driver in https://github.com/weisongwen/15_velodyne. if you have any questions about the connection between               the computer and the 3D LiDAR, you can refer to the https://github.com/weisongwen/15_velodyne.
- 2. connect the egg vehicle to the computer by run the launch file in /home/wenws/Autoware/ros/src/computing/perception/localization/packages/ivactuator/launch directory (change the directory as you may need) starup.launch, using the follwoing command.
```
$ roslaunch startup.launch
```
- 3. implemment the ndt_matching-based localization based on Autoware UI.
- 4. run the egg vehicle control node in the /home/wenws/Autoware/ros/src/computing/perception/localization/packages/ndt_localizer/nodes/ndt_matching using the following command.
```
$ python eggvehiclecontrol.py
```


## Research Papers for Reference

1. S. Kato, E. Takeuchi, Y. Ishiguro, Y. Ninomiya, K. Takeda, and T. Hamada. "An Open Approach to Autonomous Vehicles", IEEE Micro, Vol. 35, No. 6, pp. 60-69, 2015. [![Link](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)](http://online.qmags.com/MIC1115/default.aspx?sessionID=7CF18C36BF00A40746B87387B&cid=3230522&eid=19656&pg=62&mode=2#pg62&mode2)

## Demo Videos provided by the Autoware

### Public Road Demonstration
[![Public Road Demonstration](http://img.youtube.com/vi/5DaQBZvZwAI/mqdefault.jpg)](https://www.youtube.com/watch?v=5DaQBZvZwAI)

## Instruction Videos

### Quick Start
[![Quick Start](http://img.youtube.com/vi/m-4U84K7lvg/mqdefault.jpg)](https://www.youtube.com/watch?v=m-4U84K7lvg)

### Loading Map Data
[![Loading Map Data](http://img.youtube.com/vi/EJa4PHnjdRY/mqdefault.jpg)](https://www.youtube.com/watch?v=EJa4PHnjdRY)

### Localization with GNSS
[![Localization with GNSS](http://img.youtube.com/vi/5bj7gkFlul0/mqdefault.jpg)](https://www.youtube.com/watch?v=5bj7gkFlul0)

### Localization without GNSS
[![Localization without GNSS](http://img.youtube.com/vi/ODlxMzGTJzw/mqdefault.jpg)](https://www.youtube.com/watch?v=ODlxMzGTJzw)

### Mapping
[![Mapping](http://img.youtube.com/vi/HlQ0ohxvlgA/mqdefault.jpg)](https://www.youtube.com/watch?v=HlQ0ohxvlgA)

### Planning with ROSBAG
[![Planning with ROSBAG](http://img.youtube.com/vi/LZTCDbcjIdw/mqdefault.jpg)](https://www.youtube.com/watch?v=LZTCDbcjIdw)

### Planning with wf_simulator
[![Planning with wf_simulator](http://img.youtube.com/vi/HwB2NKqj2yg/mqdefault.jpg)](https://www.youtube.com/watch?v=HwB2NKqj2yg)

### Planning with Hybrid State A*
[![Planning with wf_simulator](http://img.youtube.com/vi/1WiqAHZHj8U/mqdefault.jpg)](https://www.youtube.com/watch?v=1WiqAHZHj8U)

### Data Processor for Bag File
[![Data Processor](http://img.youtube.com/vi/M38Obmy-3Ko/mqdefault.jpg)](https://youtu.be/M38Obmy-3Ko)

### Ftrace
[![Ftrace](http://img.youtube.com/vi/RoIqKgerDUw/mqdefault.jpg)](https://youtu.be/RoIqKgerDUw)


