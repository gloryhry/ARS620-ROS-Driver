#pragma once

#include <ros/duration.h>
#include <ros/time.h>

#include <ars620_driver/ars620_decoder.h>

namespace ars620_driver {

class RdiAssembler {
 public:
  explicit RdiAssembler(bool publish_partial = false,
                        ros::Duration timeout = ros::Duration(0.1),
                        size_t max_targets = 256);
  bool processHeader(const RdiHeader& header, const ros::Time& stamp, RdiCycle* completed);
  bool processTargets(const RdiTargetFrame& frame, RdiCycle* completed);
  bool pollTimeout(const ros::Time& now, RdiCycle* completed);
  bool hasActiveCycle() const;
  uint16_t cycleCounter() const;
  size_t expectedCount() const;
  size_t receivedCount() const;
  std::vector<uint32_t> missingFrameIds() const;
  bool hasTimedOut(const ros::Time& now) const;

 private:
  size_t effectiveTargetCount(uint16_t header_count) const;
  size_t expectedFrameCount() const;
  bool complete(RdiCycle* completed, bool allow_partial);
  void reset();
  bool active_ = false;
  bool publish_partial_ = false;
  ros::Duration timeout_;
  size_t max_targets_ = 256;
  ros::Time start_time_;
  RdiHeader header_;
  std::vector<RdiTarget> targets_;
  std::vector<bool> received_slots_;
  std::vector<bool> received_frame_ids_;
  size_t received_count_ = 0;
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
