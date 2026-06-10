#pragma once

#include <ros/time.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace ars620_driver {

struct ProcessingStageStats {
  uint64_t count = 0;
  double total_ms = 0.0;
  double max_ms = 0.0;

  double averageMs() const;
};

class ProcessingStageStatsView {
 public:
  ProcessingStageStatsView() = default;
  explicit ProcessingStageStatsView(const ProcessingStageStats* stats) : stats_(stats) {}

  bool has_value() const { return stats_ != nullptr; }
  explicit operator bool() const { return has_value(); }
  const ProcessingStageStats& value() const { return *stats_; }
  const ProcessingStageStats* operator->() const { return stats_; }

 private:
  const ProcessingStageStats* stats_ = nullptr;
};

class ProcessingTimingStats {
 public:
  void record(const std::string& stage, double elapsed_ms);
  ProcessingStageStatsView stage(const std::string& stage) const;
  std::string format(const std::vector<std::string>& stage_order) const;
  void reset();
  bool empty() const;

 private:
  std::vector<std::pair<std::string, ProcessingStageStats>> stages_;
};

class ScopedProcessingTimer {
 public:
  ScopedProcessingTimer(ProcessingTimingStats* stats, const std::string& stage, bool enabled);
  ~ScopedProcessingTimer();

  ScopedProcessingTimer(const ScopedProcessingTimer&) = delete;
  ScopedProcessingTimer& operator=(const ScopedProcessingTimer&) = delete;

 private:
  ProcessingTimingStats* stats_ = nullptr;
  std::string stage_;
  ros::WallTime start_;
  bool enabled_ = false;
};

}  // namespace ars620_driver
