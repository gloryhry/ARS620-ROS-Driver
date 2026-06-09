#include <ars620_driver/cycle_assembler.h>

#include <algorithm>

namespace ars620_driver {

RdiAssembler::RdiAssembler(bool publish_partial, ros::Duration timeout)
    : publish_partial_(publish_partial), timeout_(timeout) {}

bool RdiAssembler::processHeader(const RdiHeader& header, const ros::Time& stamp, RdiCycle* completed) {
  RdiCycle previous;
  const bool had_partial = active_ && publish_partial_ && !current_.targets.empty();
  if (had_partial) {
    previous = current_;
  }
  active_ = true;
  start_time_ = stamp;
  current_ = RdiCycle{};
  current_.header = header;
  if (had_partial && completed != nullptr) {
    *completed = previous;
    return true;
  }
  return header.num_clusters == 0 && complete(completed, true);
}

bool RdiAssembler::processTargets(const std::vector<RdiTarget>& targets, RdiCycle* completed) {
  if (!active_) {
    return false;
  }
  const size_t needed = current_.header.num_clusters;
  for (const auto& target : targets) {
    if (current_.targets.size() < needed) {
      current_.targets.push_back(target);
    }
  }
  return complete(completed, false);
}

bool RdiAssembler::pollTimeout(const ros::Time& now, RdiCycle* completed) {
  if (!active_ || !publish_partial_ || current_.targets.empty() || now - start_time_ < timeout_) {
    return false;
  }
  return complete(completed, true);
}

bool RdiAssembler::complete(RdiCycle* completed, bool allow_partial) {
  if (!active_) {
    return false;
  }
  const bool full = current_.targets.size() >= current_.header.num_clusters;
  if (!full && !allow_partial) {
    return false;
  }
  if (completed != nullptr) {
    *completed = current_;
  }
  active_ = false;
  current_ = RdiCycle{};
  return true;
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
