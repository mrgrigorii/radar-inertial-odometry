#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <chrono>
#include <numeric>
#include <yaml-cpp/yaml.h>

#include <ros/console.h>

namespace reve
{
struct RuntimeStatistics
{
  float min_ms   = 0.0f;
  float max_ms   = 0.0f;
  float mean_ms  = 0.0f;
  float total_ms = 0.0f;

  std::string toStringMs() const
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "mean_ms=" << mean_ms << ", max_ms=" << max_ms << ", min_ms=" << min_ms
       << ", total_s=" << total_ms;
    return ss.str();
  }

  std::string toStringUs() const
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "mean_us=" << mean_ms * 1.0e3f << ", max_us=" << max_ms * 1.0e3f
       << ", min_us=" << min_ms * 1.0e3f << ", total_us=" << total_ms * 1.0e3f;
    return ss.str();
  }
};

struct ProfileData
{
  int id = -1;
  RuntimeStatistics statistics;
  std::vector<float> execution_ms = {};
};


class SimpleProfiler
{
public:

  SimpleProfiler(const bool is_on = true);

  void start(const std::string& key);

  bool stop(const std::string& key);

  float stopWithRuntimeMs(const std::string& key);

  RuntimeStatistics getStatistics(const std::string& key);

  std::string toString(const uint indent = 0);

  std::string toMarkdownTable();

  float getTotalRuntime();

private:

  RuntimeStatistics calculateProfileStatistics(const std::string& key);
  const std::string kPrefix;
  bool is_on_;
  int next_id_;
  std::map<std::string, std::chrono::system_clock::time_point> start_times_;
  std::unordered_map<std::string, ProfileData> profile_data_;
};

}  
