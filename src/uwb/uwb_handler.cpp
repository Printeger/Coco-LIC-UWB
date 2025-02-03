#include "uwb/uwb_handler.h"

namespace cocolic {
UWBHandler::UWBHandler(const YAML::Node &node, ros::NodeHandle &nh)
    : nh_(nh), is_uwb_data_saved_(false) {
  uwb_data_path_ = node["uwb_data_path"].as<std::string>();
  uwb_data_file_ = node["uwb_data_file"].as<std::string>();
  uwb_data_topic_ = node["uwb_data_topic"].as<std::string>();
  uwb_data_num_ = node["uwb_data_num"].as<int>();

  pub_uwb_data_ = nh_.advertise<uwb::UWBData>(uwb_data_topic_, 1);
}
}  // namespace cocolic