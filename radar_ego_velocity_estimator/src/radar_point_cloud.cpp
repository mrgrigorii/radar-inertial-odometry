#include <set>
#include <radar_ego_velocity_estimator/radar_point_cloud.h>

using namespace reve;

POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPointCloudType,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, snr_db, snr_db)
                                  (float, noise_db,   noise_db)
                                  (float, v_doppler_mps,   v_doppler_mps)
                                  )


POINT_CLOUD_REGISTER_POINT_STRUCT (mmWaveCloudType,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, velocity, velocity))

bool reve::pcl2msgToPcl(const sensor_msgs::PointCloud2& pcl_msg, pcl::PointCloud<RadarPointCloudType>& scan)
{

  std::set<std::string> fields;
  std::string fields_str = "";

  for (const auto& field : pcl_msg.fields)
  {
    fields.emplace(field.name);
    fields_str += field.name + ", ";
  }

  if (fields.find("x") != fields.end() && fields.find("y") != fields.end() && fields.find("z") != fields.end() &&
      fields.find("snr_db") != fields.end() && fields.find("noise_db") != fields.end() &&
      fields.find("v_doppler_mps") != fields.end())
  {
    ROS_INFO_ONCE("[pcl2msgToPcl]: Detected rio pcl format!");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pcl_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, scan);

    for (auto& p : scan) p.range = p.getVector3fMap().norm();

    return true;
  }
  else if (fields.find("x") != fields.end() && fields.find("y") != fields.end() && fields.find("z") != fields.end() &&
           fields.find("intensity") != fields.end() && fields.find("velocity") != fields.end())
  {
    pcl::PointCloud<mmWaveCloudType> scan_mmwave;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pcl_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, scan_mmwave);

    scan.clear();
    for (const auto& p : scan_mmwave)
    {
      RadarPointCloudType p_;
      p_.x             = -p.y;
      p_.y             = p.x;
      p_.z             = p.z;
      p_.snr_db        = p.intensity;
      p_.v_doppler_mps = p.velocity;
      p_.range         = p.getVector3fMap().norm();
      p_.noise_db      = -1.;
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

bool reve::pclToPcl2msg(pcl::PointCloud<RadarPointCloudType> scan, sensor_msgs::PointCloud2& pcl_msg)
{
  scan.height = 1;
  scan.width  = scan.size();

  pcl::PCLPointCloud2 tmp;
  pcl::toPCLPointCloud2<RadarPointCloudType>(scan, tmp);
  pcl_conversions::fromPCL(tmp, pcl_msg);

  return true;
}
