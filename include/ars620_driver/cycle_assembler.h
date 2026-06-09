#pragma once

#include <ros/duration.h>
#include <ros/time.h>

#include <ars620_driver/ars620_decoder.h>

namespace ars620_driver {

class RdiAssembler {
 public:
  explicit RdiAssembler(bool publish_partial = false, ros::Duration timeout = ros::Duration(0.1));
  bool processHeader(const RdiHeader& header, const ros::Time& stamp, RdiCycle* completed);
  bool processTargets(const std::vector<RdiTarget>& targets, RdiCycle* completed);
  bool pollTimeout(const ros::Time& now, RdiCycle* completed);

 private:
  bool complete(RdiCycle* completed, bool allow_partial);
  bool active_ = false;
  bool publish_partial_ = false;
  ros::Duration timeout_;
  ros::Time start_time_;
  RdiCycle current_;
};

class OdAssembler {
 public:
  explicit OdAssembler(bool publish_partial = false, ros::Duration timeout = ros::Duration(0.1));
  bool processHeader(const OdHeader& header, const ros::Time& stamp, OdCycle* completed);
  bool processTargets(const std::vector<OdTarget>& targets, OdCycle* completed);
  bool pollTimeout(const ros::Time& now, OdCycle* completed);

 private:
  bool complete(OdCycle* completed, bool allow_partial);
  bool active_ = false;
  bool publish_partial_ = false;
  ros::Duration timeout_;
  ros::Time start_time_;
  OdCycle current_;
};

}  // namespace ars620_driver
