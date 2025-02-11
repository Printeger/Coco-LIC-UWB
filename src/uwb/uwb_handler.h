#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <uwb/UWBData.h>  // Add your UWB message type header
#include <vector>
#include <string>
#include <Eigen/Core>
#include <spline/trajectory.h>
#include <utils/cloud_tool.h>
#include <utils/yaml_utils.h>
namespace cocolic {
// 😃
class UWBHandler {
 public:
  UWBHandler(const YAML::Node &node, ros::NodeHandle &nh);
  
  // Process incoming UWB measurements
  void ParseUWBData(const uwb::UWBData::ConstPtr &uwb_msg);
  
  // Publish processed UWB data
  void PublishUWBData();
  
  // Save UWB data to file
  void SaveUWBData();

  // Getters
  bool HasNewMeasurement() const { return has_new_measurement_; }
  const uwb::UWBData& GetLatestMeasurement() const { return latest_measurement_; }
  
 private:
  // ROS related
  ros::NodeHandle nh_;
  ros::Publisher pub_uwb_data_;
  ros::Subscriber sub_uwb_data_;

  // UWB data storage
  std::vector<uwb::UWBData> uwb_data_;
  uwb::UWBData latest_measurement_;
  
  // Configuration
  std::string uwb_data_path_;
  std::string uwb_data_file_;
  std::string uwb_data_topic_;
  int uwb_data_num_;
  
  // State flags
  bool is_uwb_data_saved_;
  bool has_new_measurement_;
};
}  // namespace cocolic
