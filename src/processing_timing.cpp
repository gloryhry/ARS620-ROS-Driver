#include <ars620_driver/processing_timing.h>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace ars620_driver {

double ProcessingStageStats::averageMs() const {
  return count == 0 ? 0.0 : total_ms / static_cast<double>(count);
}

void ProcessingTimingStats::record(const std::string& stage, const double elapsed_ms) {
  auto found = std::find_if(stages_.begin(), stages_.end(), [&stage](const auto& entry) {
    return entry.first == stage;
  });
  if (found == stages_.end()) {
    stages_.push_back({stage, ProcessingStageStats()});
    found = stages_.end() - 1;
  }

  auto& stats = found->second;
  ++stats.count;
  stats.total_ms += elapsed_ms;
  stats.max_ms = std::max(stats.max_ms, elapsed_ms);
}

ProcessingStageStatsView ProcessingTimingStats::stage(const std::string& stage_name) const {
  const auto found = std::find_if(stages_.begin(), stages_.end(), [&stage_name](const auto& entry) {
    return entry.first == stage_name;
  });
  if (found == stages_.end()) {
    return ProcessingStageStatsView();
  }
  return ProcessingStageStatsView(&found->second);
}

std::string ProcessingTimingStats::format(const std::vector<std::string>& stage_order) const {
  uint64_t samples = 0;
  for (const auto& stage_name : stage_order) {
    const auto stats = stage(stage_name);
    if (stats) {
      samples = std::max(samples, stats->count);
    }
  }
  if (samples == 0) {
    return std::string();
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "samples=" << samples;
  for (const auto& stage_name : stage_order) {
    const auto stats = stage(stage_name);
    if (!stats) {
      continue;
    }
    oss << ' ' << stage_name << " avg=" << stats->averageMs() << " max=" << stats->max_ms
        << " ms";
  }
  return oss.str();
}

void ProcessingTimingStats::reset() {
  stages_.clear();
}

bool ProcessingTimingStats::empty() const {
  return stages_.empty();
}

ScopedProcessingTimer::ScopedProcessingTimer(ProcessingTimingStats* stats, const std::string& stage,
                                             const bool enabled)
    : stats_(stats), stage_(stage), enabled_(enabled && stats != nullptr) {
  if (enabled_) {
    start_ = ros::WallTime::now();
  }
}

ScopedProcessingTimer::~ScopedProcessingTimer() {
  if (!enabled_) {
    return;
  }
  const ros::WallDuration elapsed = ros::WallTime::now() - start_;
  stats_->record(stage_, elapsed.toSec() * 1000.0);
}

}  // namespace ars620_driver
