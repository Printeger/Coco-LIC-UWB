#include <spline/trajectory.h>
#include <utils/cloud_tool.h>
#include <utils/yaml_utils.h>

namespace cocolic {
// 😃
class UWBHandler {
 public:
  UWBHandler(const YAML::Node &node, ros::NodeHandle &nh);
  void ParseUWBData(const uwb::UWBData::ConstPtr &uwb_msg);
  void PublishUWBData();
  void SaveUWBData();

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_uwb_data_;
  std::vector<uwb::UWBData> uwb_data_;
  std::string uwb_data_path_;
  std::string uwb_data_file_;
  std::string uwb_data_topic_;
  int uwb_data_num_;
  bool is_uwb_data_saved_;
}
}  // namespace cocolic
