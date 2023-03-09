#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <radar_ego_velocity_estimator/ros_helper.h>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_ros.h>

using namespace reve;

RadarEgoVelocityEstimatorRos::RadarEgoVelocityEstimatorRos(ros::NodeHandle nh)
{
  reconfigure_server_.setCallback(boost::bind(&RadarEgoVelocityEstimatorRos::reconfigureCallback, this, _1, _2));

  run_without_trigger = false; 
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "run_without_trigger", run_without_trigger);
  if (run_without_trigger)
    ROS_WARN_STREAM(kPrefix << "Running without radar trigger");

  std::string topic_twist = "twist";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_twist", topic_twist);

  std::string topic_radar_scan = "/pcl2_visualize_2";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_scan", topic_radar_scan);

  std::string topic_radar_trigger = "/radar/trigger";
  getRosParameter(nh, kPrefix, RosParameterType::Recommended, "topic_radar_trigger", topic_radar_trigger);


  sub_radar_scan_ = nh.subscribe<sensor_msgs::PointCloud2>(topic_radar_scan, 50, &RadarEgoVelocityEstimatorRos::callbackRadarScan, this);
  sub_radar_trigger_ = nh.subscribe<std_msgs::Header>(topic_radar_trigger, 50, &RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this);
  pub_twist_              = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(topic_twist, 5);
}


void RadarEgoVelocityEstimatorRos::processRadarData(const sensor_msgs::PointCloud2& radar_scan,
                                                    const ros::Time& trigger_stamp)
{
  Vector3 v_b_r;
  Matrix3 P_v_b_r;
  profiler.start("ego_velocity_estimation");
  if (estimator_.estimate(radar_scan, v_b_r, P_v_b_r))
  {
    profiler.stop("ego_velocity_estimation");

    geometry_msgs::TwistWithCovarianceStamped msg;
    msg.header.stamp         = trigger_stamp;
    msg.header.frame_id      = (radar_scan.header.frame_id.empty())? "radar" : radar_scan.header.frame_id;
    msg.twist.twist.linear.x = v_b_r.x();
    msg.twist.twist.linear.y = v_b_r.y();
    msg.twist.twist.linear.z = v_b_r.z();

    for (uint l = 0; l < 3; ++l)
      for (uint k = 0; k < 3; ++k) msg.twist.covariance.at(l * 6 + k) = P_v_b_r(l, k);
    pub_twist_.publish(msg);
  }
  else
  {
    profiler.stop("ego_velocity_estimation");
    ROS_ERROR_STREAM(kPrefix << "Radar ego velocity estimation failed");
  }

  ROS_INFO_THROTTLE(5,
                    "%s Runtime statistics: %s",
                    kPrefix.c_str(),
                    profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_scan_msg)
{
  mutex_.lock();

  if (run_without_trigger)
  { 
    if (radar_scan_msg->header.stamp.sec == 0)
    {
      ROS_WARN_THROTTLE(1.0, "Time stamp of radar scan pcl is 0 using current ros time!");
      processRadarData(*radar_scan_msg, ros::Time::now());
    }
    else
    {
      processRadarData(*radar_scan_msg, radar_scan_msg->header.stamp);
    }
  }
  else
  {
    if (trigger_stamp > ros::TIME_MIN)
      processRadarData(*radar_scan_msg, trigger_stamp);
    else
    trigger_stamp = ros::TIME_MIN;
  }

  mutex_.unlock();
}

void RadarEgoVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::HeaderConstPtr& trigger_msg)
{
  mutex_.lock();
  trigger_stamp = trigger_msg->stamp;
  mutex_.unlock();
}
