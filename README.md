# radar-inertial-odometry 
The code for paper "Robust Radar-Inertial-Odometry in Dynamic 3D Environments" will be uploaded in a few days.  
Millimeter-Wave Radar is one promising sensor to achieve robust perception against challenging observing conditions. In this paper, we proposed a Radar-Inertial-Odometry (RIO) pipeline utilizing a long-range 4D millimeter-wave radar for autonomous vehicle navigation. Initially, we developed a perception front-end based on radar point cloud filtering and registration to estimate the relative transformations between frames reliably. Then an optimization-based backbone is formulated which fuses IMU data, relative poses, and point cloud velocities from radar Doppler measurements. The proposed method is extensively tested in challenging on-road environments and in-the-air environments. The results indicate that the proposed RIO can provide a reliable localization function for mobile platforms, such as automotive vehicles, Unmanned aerial vehicles (UAVs), in various operation conditions.

# Dependency
gtsam : 4.0.2  
pcl:1.8  
eigen:3.3.4   

# Install
cd ~/catkin_ws/src  
git clone https://
cd ..  
catkin_make  


# Sample datasets

# Run the package

# Related Package
