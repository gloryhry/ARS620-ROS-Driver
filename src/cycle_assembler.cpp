#include <ars620_driver/cycle_assembler.h>

#include <algorithm>
#include <numeric>

namespace ars620_driver {

RdiAssembler::RdiAssembler(bool publish_partial, ros::Duration timeout, size_t max_targets)
    : publish_partial_(publish_partial), timeout_(timeout), max_targets_(max_targets) {}

bool RdiAssembler::processHeader(const RdiHeader& header, const ros::Time& stamp, RdiCycle* completed) {
  if (active_) {
    reset();
  }
  active_ = true;
  start_time_ = stamp;
  header_ = header;
  const size_t effective_count = effectiveTargetCount(header.num_clusters);
  targets_.assign(effective_count, RdiTarget{});
  received_slots_.assign(effective_count, false);
  received_frame_ids_.assign(expectedFrameCount(), false);
  received_count_ = 0;
  return effective_count == 0 && complete(completed, true);
}

bool RdiAssembler::processTargets(const RdiTargetFrame& frame, RdiCycle* completed) {
  if (!active_) {
    return false;
  }
  if (frame.frame_id < kRdiFirstDataId || frame.frame_id > kRdiLastDataId) {
    return false;
  }
  const size_t slot = static_cast<size_t>(frame.frame_id - kRdiFirstDataId);
  if (slot >= received_frame_ids_.size()) {
    return false;
  }
  received_frame_ids_[slot] = true;
  const size_t base_index = slot * 8U;
  for (size_t i = 0; i < frame.targets.size(); ++i) {
    const size_t target_index = base_index + i;
    if (target_index >= targets_.size()) {
      break;
    }
    if (!received_slots_[target_index]) {
      targets_[target_index] = frame.targets[i];
      targets_[target_index].cluster_index = static_cast<uint16_t>(target_index);
      received_slots_[target_index] = true;
      ++received_count_;
    }
  }
  return complete(completed, false);
}

bool RdiAssembler::pollTimeout(const ros::Time& now, RdiCycle* completed) {
  if (!active_ || !publish_partial_ || received_count_ == 0 || now - start_time_ < timeout_) {
    return false;
  }
  return complete(completed, true);
}

bool RdiAssembler::hasActiveCycle() const {
  return active_;
}

uint16_t RdiAssembler::cycleCounter() const {
  return header_.cycle_counter;
}

size_t RdiAssembler::expectedCount() const {
  return active_ ? targets_.size() : 0U;
}

size_t RdiAssembler::receivedCount() const {
  return active_ ? received_count_ : 0U;
}

std::vector<uint32_t> RdiAssembler::missingFrameIds() const {
  std::vector<uint32_t> missing;
  if (!active_) {
    return missing;
  }
  for (size_t i = 0; i < received_frame_ids_.size(); ++i) {
    if (!received_frame_ids_[i]) {
      missing.push_back(static_cast<uint32_t>(kRdiFirstDataId + i));
    }
  }
  return missing;
}

bool RdiAssembler::hasTimedOut(const ros::Time& now) const {
  return active_ && received_count_ > 0 && now - start_time_ >= timeout_;
}

size_t RdiAssembler::effectiveTargetCount(uint16_t header_count) const {
  if (max_targets_ == 0U) {
    return header_count;
  }
  return std::min(static_cast<size_t>(header_count), max_targets_);
}

size_t RdiAssembler::expectedFrameCount() const {
  return (targets_.size() + 7U) / 8U;
}

bool RdiAssembler::complete(RdiCycle* completed, bool allow_partial) {
  if (!active_) {
    return false;
  }
  const bool full = received_count_ >= targets_.size();
  if (!full && !allow_partial) {
    return false;
  }
  if (completed != nullptr) {
    completed->header = header_;
    completed->targets.clear();
    completed->targets.reserve(full ? targets_.size() : received_count_);
    for (size_t i = 0; i < targets_.size(); ++i) {
      if (received_slots_[i]) {
        completed->targets.push_back(targets_[i]);
      }
    }
  }
  reset();
  return true;
}

void RdiAssembler::reset() {
  active_ = false;
  header_ = RdiHeader{};
  targets_.clear();
  received_slots_.clear();
  received_frame_ids_.clear();
  received_count_ = 0;
}

OdAssembler::OdAssembler(bool publish_partial, ros::Duration timeout)
    : publish_partial_(publish_partial), timeout_(timeout) {}

bool OdAssembler::processHeader(const OdHeader& header, const ros::Time& stamp, OdCycle* completed) {
  OdCycle previous;
  const bool had_partial = active_ && publish_partial_ && !current_.targets.empty();
  if (had_partial) {
    previous = current_;
  }
  active_ = true;
  start_time_ = stamp;
  current_ = OdCycle{};
  current_.header = header;
  if (had_partial && completed != nullptr) {
    *completed = previous;
    return true;
  }
  return header.num_objects == 0 && complete(completed, true);
}

bool OdAssembler::processTargets(const std::vector<OdTarget>& targets, OdCycle* completed) {
  if (!active_) {
    return false;
  }
  const size_t needed = current_.header.num_objects;
  for (const auto& target : targets) {
    if (current_.targets.size() < needed) {
      current_.targets.push_back(target);
    }
  }
  return complete(completed, false);
}

bool OdAssembler::pollTimeout(const ros::Time& now, OdCycle* completed) {
  if (!active_ || !publish_partial_ || current_.targets.empty() || now - start_time_ < timeout_) {
    return false;
  }
  return complete(completed, true);
}

bool OdAssembler::complete(OdCycle* completed, bool allow_partial) {
  if (!active_) {
    return false;
  }
  const bool full = current_.targets.size() >= current_.header.num_objects;
  if (!full && !allow_partial) {
    return false;
  }
  if (completed != nullptr) {
    *completed = current_;
  }
  active_ = false;
  current_ = OdCycle{};
  return true;
}

}  // namespace ars620_driver
