#pragma once

#include <mutex>
#include <angles/angles.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <radar_ego_velocity_estimator/data_types.h>
#include <radar_ego_velocity_estimator/ros_helper.h>
#include <radar_ego_velocity_estimator/simple_profiler.h>

#include <radar_ego_velocity_estimator/RadarEgoVelocityEstimatorConfig.h>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator.h>

namespace reve
{

class RadarEgoVelocityEstimatorRos
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  RadarEgoVelocityEstimatorRos(ros::NodeHandle nh);

  void reconfigureCallback(radar_ego_velocity_estimation::RadarEgoVelocityEstimatorConfig& config, uint32_t level)
  {
    estimator_.configure(config);
  }

  void processRadarData(const sensor_msgs::PointCloud2& radar_scan, const ros::Time& trigger_stamp);

  void callbackImu(const sensor_msgs::ImuConstPtr& imu_msg);

  void callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_scan_msg);

  void callbackRadarTrigger(const std_msgs::HeaderConstPtr& trigger_msg);

private:
  const std::string kPrefix = "[RadarEgoVelocityEstimatorRos]: ";

  dynamic_reconfigure::Server<radar_ego_velocity_estimation::RadarEgoVelocityEstimatorConfig> reconfigure_server_;

  RadarEgoVelocityEstimator estimator_;

  SimpleProfiler profiler;

  ros::Subscriber sub_radar_scan_;
  ros::Subscriber sub_radar_trigger_;

  ros::Publisher pub_twist_;
  ros::Publisher pub_twist_ground_truth_;

  bool run_without_trigger = false; //false

  std::mutex mutex_;
  ros::Time trigger_stamp = ros::TIME_MIN;
};

} 
