#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/Int16.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;
const double f = 1 / 298.2572236;
const double e_2 = 2 * f - pow(f, 2);
const int a = 6378137;
const double pi = 3.14159265358979;
int init_att_flag = 0;

struct coordinate
{
    double cor_1;
    double cor_2;
    double cor_3;
};

class DJI
{
public:
    ros::NodeHandle ph;

    nav_msgs::Path path;
    nav_msgs::Odometry gpsodom;
    ros::Publisher path_pub;
    ros::Publisher odom_pub;
    geometry_msgs::Quaternion Q_FLU;

    string gps_topic;
    string VO_topic;
    string attitude_topic;
    string gps_health_topic;
    string imu_attitude_topic;
    double init_yaw_d;
    double init_yaw;

    int health;
    int init_att_flag;

    ros::Subscriber gpsSub;
    ros::Subscriber attitudeSub;
    ros::Subscriber imu_attitudeSub;
    ros::Subscriber healthSub;

    tf2_ros::StaticTransformBroadcaster broadcaster;

    DJI()
    {
        ph.param<string>("gps_topic", gps_topic, "/dji_osdk_ros/rtk_position");
        ph.param<string>("VO_topic", VO_topic, "/dji_osdk_ros/vo_position");
        ph.param<string>("attitude_topic", attitude_topic, "/dji_osdk_ros/attitude1");
        ph.param<string>("healthy_topic", gps_health_topic, "/dji_osdk_ros/gps_health");
        ph.param<string>("imu_attitude_topic", imu_attitude_topic, "/dji_osdk_ros/imu");
        ph.param<double>("init_yaw_d", init_yaw_d, 0);

        init_yaw = init_yaw_d * pi / 180;
        health = 5;
        init_att_flag = 0;
        path_pub = ph.advertise<nav_msgs::Path>("/trajectory_gps", 10, true);
        odom_pub = ph.advertise<nav_msgs::Odometry>("/odometry/gps", 10, true);

        gpsSub = ph.subscribe(gps_topic, 10, &DJI::gpsCallback, this);
        attitudeSub = ph.subscribe(attitude_topic, 10, &DJI::attitudeCallback, this);
        imu_attitudeSub = ph.subscribe(imu_attitude_topic, 10, &DJI::imuCallback, this);
        healthSub = ph.subscribe(gps_health_topic, 10, &DJI::healthCallback, this);
    }
    coordinate LLA2ECEF(coordinate LLA)
    {
        double lon = LLA.cor_1 * pi / 180;
        double lat = LLA.cor_2 * pi / 180;
        double h = LLA.cor_3;
        double RN = a / sqrt(1 - e_2 * pow(sin(lat), 2));
        coordinate ECEF;
        ECEF.cor_1 = (RN + h) * cos(lat) * cos(lon);
        ECEF.cor_2 = (RN + h) * cos(lat) * sin(lon);
        ECEF.cor_3 = (RN * (1 - e_2) + h) * sin(lat);
        return ECEF;
    }

    coordinate ECEF2ENU(coordinate ECEF, coordinate LLA_0, coordinate ECEF_0)
    {
        double lon_0 = LLA_0.cor_1 * pi / 180;
        double lat_0 = LLA_0.cor_2 * pi / 180;
        double h_0 = LLA_0.cor_3;
        coordinate delta;
        delta.cor_1 = ECEF.cor_1 - ECEF_0.cor_1;
        delta.cor_2 = ECEF.cor_2 - ECEF_0.cor_2;
        delta.cor_3 = ECEF.cor_3 - ECEF_0.cor_3;
        coordinate ENU;
        ENU.cor_1 = -sin(lon_0) * delta.cor_1 + cos(lon_0) * delta.cor_2;
        ENU.cor_2 = -sin(lat_0) * cos(lon_0) * delta.cor_1 + -sin(lat_0) * sin(lon_0) * delta.cor_2 + cos(lat_0) * delta.cor_3;
        ENU.cor_3 = cos(lat_0) * cos(lon_0) * delta.cor_1 + cos(lat_0) * sin(lon_0) * delta.cor_2 + sin(lat_0) * delta.cor_3;
        return ENU;
    }

    void attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
    {
        static int init_att_flag = 0;
        static Eigen::Quaterniond init_att;
        static Eigen::Matrix3d init_Matrix;
        Eigen::Quaterniond cur_att;
        if (init_att_flag < 5)
        {
            init_att.w() = msg->quaternion.w;
            init_att.x() = msg->quaternion.x;
            init_att.y() = msg->quaternion.y;
            init_att.z() = msg->quaternion.z;
            init_Matrix = init_att.toRotationMatrix();
            init_att_flag += 1;
            return;
        }
        Eigen::Quaterniond Q_FRD2FLU(0, 1, 0, 0);
        cur_att.w() = msg->quaternion.w;
        cur_att.x() = msg->quaternion.x;
        cur_att.y() = msg->quaternion.y;
        cur_att.z() = msg->quaternion.z;
        Eigen::Matrix3d FLU_Matrix = init_Matrix.inverse() * cur_att.toRotationMatrix();
        Eigen::Quaterniond tmp(FLU_Matrix);
        Q_FLU.w = tmp.w();
        Q_FLU.x = tmp.x();
        Q_FLU.y = tmp.y();
        Q_FLU.z = tmp.z();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        static Eigen::Quaterniond init_att;
        static Eigen::Matrix3d init_Matrix;
        Eigen::Quaterniond cur_att;
        if (init_att_flag < 5)
        {
            init_att.w() = msg->orientation.w;
            init_att.x() = msg->orientation.x;
            init_att.y() = msg->orientation.y;
            init_att.z() = msg->orientation.z;
            init_Matrix = init_att.toRotationMatrix();
            init_att_flag += 1;
            return;
        }
        Eigen::Quaterniond Q_FRD2FLU(0, 1, 0, 0);
        cur_att.w() = msg->orientation.w;
        cur_att.x() = msg->orientation.x;
        cur_att.y() = msg->orientation.y;
        cur_att.z() = msg->orientation.z;
        Eigen::Matrix3d FLU_Matrix = init_Matrix.inverse() * cur_att.toRotationMatrix();
        Eigen::Quaterniond tmp(FLU_Matrix);
        Q_FLU.w = tmp.w();
        Q_FLU.x = tmp.x();
        Q_FLU.y = tmp.y();
        Q_FLU.z = tmp.z();
    }

    void healthCallback(std_msgs::UInt8 msg)
    {
        health = msg.data;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {

        static int threshold;
        static coordinate LLA_0, ECEF_0;
        coordinate LLA_now, ECEF_now, ENU_now;
        if (threshold == 0)
        {
            LLA_0.cor_1 = msg->longitude;
            LLA_0.cor_2 = msg->latitude;
            LLA_0.cor_3 = msg->altitude;
            ECEF_0 = LLA2ECEF(LLA_0);
            threshold = 1;
        }
        LLA_now.cor_1 = msg->longitude;
        LLA_now.cor_2 = msg->latitude;
        LLA_now.cor_3 = msg->altitude;
        ECEF_now = LLA2ECEF(LLA_now);
        ENU_now = ECEF2ENU(ECEF_now, LLA_0, ECEF_0);

        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = ENU_now.cor_1;
        this_pose_stamped.pose.position.y = ENU_now.cor_2;
        this_pose_stamped.pose.position.z = ENU_now.cor_3;
        this_pose_stamped.pose.orientation = Q_FLU;
        this_pose_stamped.header.stamp = msg->header.stamp;
        this_pose_stamped.header.frame_id = "map_ENU";

        path.poses.push_back(this_pose_stamped);
        path.header.stamp = msg->header.stamp;
        path.header.frame_id = "map_ENU";
        path_pub.publish(path);
        gpsodom.pose.pose.position = this_pose_stamped.pose.position;
        gpsodom.pose.pose.orientation = Q_FLU;
        gpsodom.pose.covariance[0] = 4e-4 * pow(10, 5 - health);
        gpsodom.pose.covariance[7] = 4e-4 * pow(10, 5 - health);
        gpsodom.pose.covariance[14] = 9e-4 * pow(10, 5 - health);
        gpsodom.pose.covariance[21] = 2.7e-4 * pow(10, 5 - health);
        gpsodom.pose.covariance[28] = 2.7e-4 * pow(10, 5 - health);
        gpsodom.pose.covariance[35] = 2.5e-3 * pow(10, 5 - health);
        gpsodom.header.stamp = msg->header.stamp;
        gpsodom.header.frame_id = "map_ENU";
        odom_pub.publish(gpsodom);

        geometry_msgs::TransformStamped ts;
        ts.header.seq = 100;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "map";
        ts.child_frame_id = "map_ENU";
        ts.transform.translation.x = 0.0;
        ts.transform.translation.y = 0.0;
        ts.transform.translation.z = 0.0;
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, init_yaw);
        ts.transform.rotation.x = qtn.getX();
        ts.transform.rotation.y = qtn.getY();
        ts.transform.rotation.z = qtn.getZ();
        ts.transform.rotation.w = qtn.getW();
        broadcaster.sendTransform(ts);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DJI");
    DJI dji;
    ROS_INFO("\033[1;32m----> DJI Started.\033[0m");
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}