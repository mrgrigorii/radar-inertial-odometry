#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/registration/ndt.h>

#include <time.h> 
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include "utility.h"
#include <cmath>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZI PointType;
using namespace std;

struct mmWaveCloudType
{
    PCL_ADD_POINT4D;
    union
    {
        struct
        {
            float intensity;
            float velocity;
        };
        float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(mmWaveCloudType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity))

class NDT : public ParamServer
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber points_sub_;
    ros::Subscriber imu_sub_;

    ros::Subscriber sub_twist_;
    bool imu_init_flag = false;

    struct pose
    {
        double x, y, z;
        double roll, pitch, yaw;
    };
    struct pose current_pose_, current_pose_imu_;
    struct pose previous_pose_;

    pcl::PointCloud<pcl::PointXYZI> map_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

    int max_iter_;     // Maximum iterations
    double ndt_res_;   // Resolution
    double step_size_; // Step size
    double trans_eps_; // Transformation epsilon

    double voxel_leaf_size_; // Leaf size of VoxelGrid filter.

    // double scan_rate_;
    double min_scan_range_;
    double max_scan_range_;
    bool use_imu_;

    std::string robot_frame_;
    std::string map_frame_;

    ros::Publisher ndt_map_pub_, current_pose_pub_;
    nav_msgs::Odometry current_pose_msg_;

    tf::TransformBroadcaster br_;

    int initial_scan_loaded;
    double min_add_scan_shift_;

    double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
    Eigen::Matrix4f tf_btol_, tf_ltob_;

    bool _incremental_voxel_update;

    bool is_first_map_;

    nav_msgs::Path globalPath;
    ros::Publisher pubPath;
    ros::Subscriber subCloud;
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;

    Eigen::Matrix4f init_guess{Eigen::Matrix4f::Identity()};
    ros::Time last_time_;
    Eigen::Vector3f acc_bias_ = Eigen::Vector3f(0.1, -0.2, 0.3);
    Eigen::Vector3f vel_ = Eigen::Vector3f(1.0, 2.0, 3.0);

    // std::mutex init_lock;
    bool imu_received_;
    sensor_msgs::Imu imu_data_;
    std::mutex velLock;
    std::deque<geometry_msgs::TwistWithCovarianceStamped> velQue;
    geometry_msgs::TwistWithCovarianceStamped CurVel_msgs;
    double CulVel_value;

    NDT()
    {
        points_sub_ = nh_.subscribe(pointCloudTopic, 10000, &NDT::points_callback, this); /// points_raw pcl2_visualize
        imu_sub_ = nh_.subscribe(imuTopic, 10000, &NDT::imu_Callback, this);              // /imu_correct
        sub_twist_ = nh_.subscribe<geometry_msgs::TwistWithCovarianceStamped>("/reve/twist", 5, &NDT::velCallBack, this, ros::TransportHints().tcpNoDelay());

        ndt_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1);
        current_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("/current_pose", 1);
        // Default values
        nh_.param("max_iter", max_iter_, 30);   
        nh_.param("step_size", step_size_, 0.01);  
        nh_.param("ndt_res", ndt_res_, 3.0);   
        nh_.param("trans_eps", trans_eps_, 0.01); 
        nh_.param("voxel_leaf_size", voxel_leaf_size_, 4.0);
        nh_.param("min_scan_range", min_scan_range_, 2.0);
        nh_.param("max_scan_range", max_scan_range_, 200.0);
        nh_.param("use_imu", use_imu_, false);

        ros::NodeHandle pnh_("~");
        pnh_.param<std::string>("robot_frame", robot_frame_, std::string("base_link"));
        pnh_.param<std::string>("map_frame", map_frame_, std::string("map"));

        initial_scan_loaded = 0;
        min_add_scan_shift_ = 2.0; ///  原 1.0
        _tf_x = 0.0, _tf_y = 0.0, _tf_z = 0.0, _tf_roll = 0.0, _tf_pitch = 0.0, _tf_yaw = -1.57;

        Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                // tl: translation
        Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX()); // rot: rotation
        Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
        tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
        tf_ltob_ = tf_btol_.inverse();

        map_.header.frame_id = map_frame_;
        current_pose_.x = current_pose_.y = current_pose_.z = 0.0;
        current_pose_.roll = current_pose_.pitch = current_pose_.yaw = 0.0;

        previous_pose_.x = previous_pose_.y = previous_pose_.z = 0.0;
        previous_pose_.roll = previous_pose_.pitch = previous_pose_.yaw = 0.0;

        voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

        ndt.setTransformationEpsilon(trans_eps_); 
        ndt.setStepSize(step_size_);
        ndt.setResolution(ndt_res_);
        ndt.setMaximumIterations(max_iter_);
        is_first_map_ = true;
        imu_received_ = false;

        pubPath = nh_.advertise<nav_msgs::Path>("mapping/path", 1);
    }

    void velCallBack(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &velmsgs)
    {
        std::lock_guard<std::mutex> lock1(velLock);
        CulVel_value = abs(std::sqrt(velmsgs->twist.twist.linear.x * velmsgs->twist.twist.linear.x + velmsgs->twist.twist.linear.y * velmsgs->twist.twist.linear.y + velmsgs->twist.twist.linear.z * velmsgs->twist.twist.linear.z));
    }

    bool pcl2msgToPcl(const sensor_msgs::PointCloud2 &pcl_msg, pcl::PointCloud<mmWaveCloudType> &scan)
    {
        
        std::set<std::string> fields;
        std::string fields_str = "";

        for (const auto &field : pcl_msg.fields)
        {
            fields.emplace(field.name);
            fields_str += field.name + ", ";
        }
        if (fields.find("x") != fields.end() && fields.find("y") != fields.end() && fields.find("z") != fields.end() &&
            fields.find("intensity") != fields.end() && fields.find("velocity") != fields.end())
        {
            pcl::PointCloud<mmWaveCloudType> scan_mmwave;
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(pcl_msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, scan_mmwave);

            scan.clear();
            for (const auto &p : scan_mmwave)
            {
                mmWaveCloudType p_;
                p_.x = -p.y;
                p_.y = p.x;
                p_.z = p.z;
                p_.velocity = p.velocity;
                scan.push_back(p_);
            }
            return true;
        }
        else
        {
            ROS_ERROR_STREAM(
                "[pcl2msgToPcl]: Unsupported point cloud with fields: " << fields_str.substr(0, fields_str.size() - 2));
            return false;
        }
    }

    void imu_Callback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        imu_data_ = *msg;
        imu_received_ = true;
    }

    void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
        std::lock_guard<std::mutex> lock1(velLock);

        if (!imu_received_)
        {
            ROS_WARN("Waiting for IMU data...\n");
            return;
        }

        //ROS_WARN("velocity is %f", CulVel_value);
        auto mmwave_radar_scan(new pcl::PointCloud<mmWaveCloudType>);
        auto mmwave_radar_remain(new pcl::PointCloud<mmWaveCloudType>);


        pcl::PointCloud<pcl::PointXYZI> tmp, scan;
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        tf::Quaternion q;

        Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
        Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
        tf::Transform transform;

        timeLaserInfoStamp = input->header.stamp;

        pcl::fromROSMsg(*input, tmp);
        double r;
        Eigen::Vector3d point_pos;
        pcl::PointXYZI p;

        if (pcl2msgToPcl(*input, *mmwave_radar_scan))
        {
            for (unsigned int i = 0; i < mmwave_radar_scan->size(); ++i)
            {
                auto point = mmwave_radar_scan->at(i);
                auto pvel = abs(point.velocity);
                if (abs(pvel - CulVel_value) < VelThreshold)
                {
                    p.x = point.x;
                    p.y = point.y;
                    p.z = point.z;
                    p.intensity = point.velocity;
                    // minmax
                    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
                    if (min_scan_range_ < r && r < max_scan_range_ && p.z > -4)
                    {
                        scan.push_back(p);
                    }
                    //mmwave_radar_remain->push_back(point);
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

        // Add initial point cloud to velodyne_map
        if (initial_scan_loaded == 0)
        {
            pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol_);
            map_ += *transformed_scan_ptr;
            initial_scan_loaded = 1;
        }

        voxel_grid_filter_.setInputCloud(scan_ptr);
        voxel_grid_filter_.filter(*filtered_scan_ptr);
        ndt.setInputSource(filtered_scan_ptr); 

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>(map_));

        if (is_first_map_ == true)
        {
            ndt.setInputTarget(map_ptr); 
            is_first_map_ = false;
        }

        Eigen::Translation3f init_translation(current_pose_.x, current_pose_.y, current_pose_.z);
        Eigen::AngleAxisf init_rotation_x(current_pose_.roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(current_pose_.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(current_pose_.yaw, Eigen::Vector3f::UnitZ());

        // Eigen::Matrix4f init_guess =
        //     (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;
        //     (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        double roll_i, pitch_i, yaw_i;
        tf::Quaternion qua_i(imu_data_.orientation.x, imu_data_.orientation.y, imu_data_.orientation.z, imu_data_.orientation.w);
        tf::Matrix3x3(qua_i).getRPY(roll_i, pitch_i, yaw_i);
        Eigen::AngleAxisf roll_angle(roll_i, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitch_angle(pitch_i, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yaw_angle(yaw_i, Eigen::Vector3f::UnitZ());
        init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;

        // Eigen::Quaternionf imu_orientation(yaw_angle * pitch_angle * roll_angle);

        ndt.align(*output_cloud, init_guess);
        t_localizer = ndt.getFinalTransformation();
        // double transform_probability = ndt.getTransformationProbability(); 
        // ROS_WARN("transform_probability is %f", transform_probability);

        t_base_link = t_localizer * tf_ltob_;

        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

        tf::Matrix3x3 mat_b;
        mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                       static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                       static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                       static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                       static_cast<double>(t_base_link(2, 2)));

        // Update current_pose_.
        current_pose_.x = t_base_link(0, 3);
        current_pose_.y = t_base_link(1, 3);
        current_pose_.z = t_base_link(2, 3);
        mat_b.getRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw, 1); 

        transform.setOrigin(tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
        q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw); 
        transform.setRotation(q);                                          

        br_.sendTransform(tf::StampedTransform(transform, timeLaserInfoStamp, map_frame_, robot_frame_));

        double shift = sqrt(pow(current_pose_.x - previous_pose_.x, 2.0) + pow(current_pose_.y - previous_pose_.y, 2.0));
        if (shift >= min_add_scan_shift_)
        {
            map_ += *transformed_scan_ptr;
            previous_pose_.x = current_pose_.x;
            previous_pose_.y = current_pose_.y;
            previous_pose_.z = current_pose_.z;
            previous_pose_.roll = current_pose_.roll;
            previous_pose_.pitch = current_pose_.pitch;
            previous_pose_.yaw = current_pose_.yaw;
            ndt.setInputTarget(map_ptr);

            sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

            ror.setInputCloud(map_ptr);                         
            ror.setRadiusSearch(RofRadiusOutlierRemoval);           
            ror.setMinNeighborsInRadius(NumofRadiusOutlierRemoval); 
            ror.filter(*cloud_filtered); 

            pcl::toROSMsg(*cloud_filtered, *map_msg_ptr);
            ndt_map_pub_.publish(*map_msg_ptr);
        }

        current_pose_msg_.header.frame_id = "map";
        current_pose_msg_.header.stamp = timeLaserInfoStamp;
        current_pose_msg_.pose.pose.position.x = current_pose_.x;
        current_pose_msg_.pose.pose.position.y = current_pose_.y;
        current_pose_msg_.pose.pose.position.z = current_pose_.z;
        current_pose_msg_.pose.pose.orientation.x = q.x();
        current_pose_msg_.pose.pose.orientation.y = q.y();
        current_pose_msg_.pose.pose.orientation.z = q.z();
        current_pose_msg_.pose.pose.orientation.w = q.w();
        current_pose_pub_.publish(current_pose_msg_);

        static tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped tfs;
        tfs.header.frame_id = "map";
        tfs.header.stamp = timeLaserInfoStamp;
        tfs.child_frame_id = "radar"; 
        tfs.transform.translation.x = current_pose_.x;
        tfs.transform.translation.y = current_pose_.y;
        tfs.transform.translation.z = current_pose_.z;
        tfs.transform.rotation = current_pose_msg_.pose.pose.orientation;
        broadcaster.sendTransform(tfs);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = timeLaserInfoStamp;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = current_pose_.x;
        pose_stamped.pose.position.y = current_pose_.y;
        pose_stamped.pose.position.z = current_pose_.z;
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();
        pose_stamped.pose.orientation = current_pose_msg_.pose.pose.orientation;
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = "map";
        globalPath.poses.push_back(pose_stamped);
        pubPath.publish(globalPath);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_slam");
    NDT NDT;
    ROS_INFO("\033[1;32m----> NDT Started.\033[0m");
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
