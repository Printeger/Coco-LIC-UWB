// test_uwb.hpp

#ifndef TEST_UWB_HPP
#define TEST_UWB_HPP

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <your_package/UWBData.h>  // Replace with your actual message type

class UWBNode {
 public:
  UWBNode();
  void uwbCallback(const your_package::UWBData::ConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber uwb_sub_;
  ros::Publisher position_pub_;

  geometry_msgs::Point calculatePosition(
      const your_package::UWBData::ConstPtr& msg);
};

#endif  // TEST_UWB_HPP