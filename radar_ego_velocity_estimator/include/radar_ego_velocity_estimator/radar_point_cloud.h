#pragma once

#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

namespace reve
{
struct RadarPointCloudType
{
  PCL_ADD_POINT4D;    
  float snr_db;         
  float v_doppler_mps;  
  float noise_db;       
  float range;        
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

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

bool pcl2msgToPcl(const sensor_msgs::PointCloud2& pcl_msg, pcl::PointCloud<RadarPointCloudType>& scan);

bool pclToPcl2msg(pcl::PointCloud<RadarPointCloudType> scan, sensor_msgs::PointCloud2& pcl_msg);

}  
