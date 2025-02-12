#include "uwb/uwb_handler.h"

#include <ros/ros.h>

#include <fstream>

namespace cocolic {

UWBHandler::UWBHandler(const YAML::Node &node, ros::NodeHandle &nh)
    : nh_(nh), is_uwb_data_saved_(false), has_new_measurement_(false) {
  // Load configuration from YAML
  uwb_data_path_ = node["uwb_data_path"].as<std::string>();
  uwb_data_file_ = node["uwb_data_file"].as<std::string>();
  uwb_data_topic_ = node["uwb_data_topic"].as<std::string>();
  uwb_data_num_ = node["uwb_data_num"].as<int>();

  // Initialize ROS publisher
  pub_uwb_data_ = nh_.advertise<uwb::UWBData>(uwb_data_topic_, 1);

  // Initialize ROS subscriber
  sub_uwb_data_ =
      nh_.subscribe(uwb_data_topic_, 10, &UWBHandler::ParseUWBData, this);

  // Reserve space for data storage
  uwb_data_.reserve(uwb_data_num_);
}

void UWBHandler::ParseUWBData(const uwb::UWBData::ConstPtr &uwb_msg) {
  // Store the measurement
  latest_measurement_ = *uwb_msg;
  has_new_measurement_ = true;

  // Add to history if needed
  if (uwb_data_.size() < uwb_data_num_) {
    uwb_data_.push_back(*uwb_msg);
  }

  // Publish processed data
  PublishUWBData();
}

void UWBHandler::PublishUWBData() {
  if (has_new_measurement_) {
    pub_uwb_data_.publish(latest_measurement_);
    has_new_measurement_ = false;
  }
}

void UWBHandler::SaveUWBData() {
  if (is_uwb_data_saved_ || uwb_data_.empty()) {
    return;
  }

  try {
    std::ofstream file(uwb_data_path_ + "/" + uwb_data_file_);

    for (const auto &data : uwb_data_) {
      // Format: timestamp, distance, anchor_id, etc.
      file << data.header.stamp << "," << data.distance << "," << data.anchor_id
           << "\n";  // Adjust based on your UWBData message structure
    }

    file.close();
    is_uwb_data_saved_ = true;
    ROS_INFO("UWB data saved successfully to: %s",
             (uwb_data_path_ + "/" + uwb_data_file_).c_str());
  } catch (const std::exception &e) {
    ROS_ERROR("Failed to save UWB data: %s", e.what());
  }
}

}  // namespace cocolic