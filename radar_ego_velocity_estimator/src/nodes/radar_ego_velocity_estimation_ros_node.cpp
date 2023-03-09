#include <ros/ros.h>

#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_ros.h>

using namespace reve;

const std::string kNodeName = "radar_ego_velocity_estimator";
const std::string kPrefix = "[" + kNodeName + "]: ";

int main(int argc, char **argv)
{
  ros::init(argc, argv, kNodeName);
  ros::NodeHandle nh("~");

  RadarEgoVelocityEstimatorRos estimator_ros(nh);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  // ros::spin();

  return 0;
}
