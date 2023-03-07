# radar-inertial-odometry 
The code for paper "Robust Radar-Inertial-Odometry in Dynamic 3D Environments" will be uploaded in a few days.  
Millimeter-Wave Radar is one promising sensor to achieve robust perception against challenging observing conditions. In this paper, we proposed a Radar-Inertial-Odometry (RIO) pipeline utilizing a long-range 4D millimeter-wave radar for autonomous vehicle navigation. Initially, we developed a perception front-end based on radar point cloud filtering and registration to estimate the relative transformations between frames reliably. Then an optimization-based backbone is formulated which fuses IMU data, relative poses, and point cloud velocities from radar Doppler measurements. The proposed method is extensively tested in challenging on-road environments and in-the-air environments. The results indicate that the proposed RIO can provide a reliable localization function for mobile platforms, such as automotive vehicles, Unmanned aerial vehicles (UAVs), in various operation conditions. 
A video of the demonstration of the method can be found on [YouTube].
# System architecture
![frame](https://github.com/lincoln1587/radar-inertial-odometry/blob/master/radar_slam/doc/frame.png)

# The available branches are in the master

# Run Demo
![demo](https://github.com/lincoln1587/radar-inertial-odometry/blob/master/radar_slam/doc/lib.png)

# Dependency
ros : melodic  
gtsam : 4.0.2  
pcl : 1.8  
eigen : 3.3.4   

# Install
cd ~/catkin_ws/src  
git clone https://github.com/lincoln1587/radar-inertial-odometry
cd ..  
catkin_make  

# Sample datasets
**4D millimeter wave radar data set**：[Google-drive](https://drive.google.com/file/d/13xzZ3uGyV6l2fjjCZix_S3XPCB6JCgLJ/view?usp=sharing)

# Run the package
1. Run the launch file:  
roslaunch radar_slam run.launch
2. Play existing bag files:  
rosbag play your-bag.bag -r 3


# Related Package
1. Optimization node:[liosam](https://github.com/TixiaoShan/LIO-SAM)
2. radar ego-motion estimator: [estimator](https://github.com/christopherdoer/rio)
3. Point cloud registration: [ndt](https://github.com/zju-sclab/NDT-library)
4.catkin simple : [catkin simple](https://github.com/catkin/catkin_simple)
# TODO

