#pragma once

#include <Eigen/Dense>
#include <radar_ego_velocity_estimator/data_types.h>

namespace reve
{
namespace math_helper
{

static Matrix3 skewVec(const Vector3& v)
{
  Matrix3 S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}
}  
}  
