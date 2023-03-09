# radar-inertial-odometry 
**The code for paper "Robust Radar-Inertial-Odometry in Dynamic 3D Environments" which is submitted to IROS 2023.**  
In this paper, we proposed a Radar-Inertial-Odometry (RIO) pipeline utilizing a long-range 4D millimeter-wave radar for autonomous vehicle navigation. 
The code and a test dataset are provided. A video of the demonstration of the method can be found on [Youtube](https://youtu.be/Bk2WCr_wWeI).
# System architecture
![frame](https://github.com/lincoln1587/radar-inertial-odometry/blob/master/radar_slam/doc/frame.png)

# A localization result of a sample dataset
![demo](https://github.com/lincoln1587/radar-inertial-odometry/blob/master/radar_slam/doc/lib.png)

# Dependency
The code is tested based on the following setups
ros : melodic  
gtsam : 4.0.2  
pcl : 1.8  
eigen : 3.3.4   

# Install
```
mdir -p /catkin_ws/src
cd ~/catkin_ws/src  
git clone https://github.com/lincoln1587/radar-inertial-odometry
cd ..  
catkin_make  
```
# Sample datasets
**4D millimeter wave radar data set**：[Google-drive](https://drive.google.com/file/d/13xzZ3uGyV6l2fjjCZix_S3XPCB6JCgLJ/view?usp=sharing)

# Run the package
1. Run the launch file:  
```
roslaunch radar_slam run.launch
```
2. Play existing bag files:  
```
rosbag play your-bag.bag
```

# Related Package
1. Optimization node:[liosam](https://github.com/TixiaoShan/LIO-SAM)
2. radar ego-motion estimator: [estimator](https://github.com/christopherdoer/rio)
3. Point cloud registration: [ndt](https://github.com/zju-sclab/NDT-library)  
4. catkin simple : [catkin simple](https://github.com/catkin/catkin_simple)

# TODO

