#include <ars620_driver/canfd_frame_logger.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace ars620_driver {
namespace {

std::string expandUserPath(const std::string& path) {
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char* home = std::getenv("HOME");
  if (home == nullptr || home[0] == '\0') {
    return path;
  }
  if (path.size() == 1) {
    return home;
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}

bool makeDirectories(const std::string& path, std::string* error) {
  if (path.empty()) {
    if (error != nullptr) {
      *error = "log directory is empty";
    }
    return false;
  }
  std::string current;
  for (size_t i = 0; i < path.size(); ++i) {
    current.push_back(path[i]);
    if (path[i] != '/' || current.size() == 1) {
      continue;
    }
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      if (error != nullptr) {
        *error = "failed to create directory " + current + ": " + std::strerror(errno);
      }
      return false;
    }
  }
  if (::mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
    if (error != nullptr) {
      *error = "failed to create directory " + path + ": " + std::strerror(errno);
    }
    return false;
  }
  return true;
}

std::string currentTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&now_time, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::string ascId(const RawCanFdFrame& frame) {
  std::ostringstream oss;
  oss << std::uppercase << std::hex << frame.can_id;
  if (!frame.is_extended) {
    oss << 'x';
  } else {
    oss << 'X';
  }
  return oss.str();
}

}  // namespace

CanFdFrameLogger::~CanFdFrameLogger() {
  close(false);
}

bool CanFdFrameLogger::open(const std::string& directory, std::string* error) {
  return open(directory, currentTimestamp(), error);
}

bool CanFdFrameLogger::open(const std::string& directory, const std::string& timestamp,
                            std::string* error) {
  close(false);
  const std::string expanded_dir = expandUserPath(directory);
  if (!makeDirectories(expanded_dir, error)) {
    return false;
  }

  const std::string stem = expanded_dir + "/ars620_canfd_" + timestamp;
  asc_path_ = stem + ".asc";
  raw_csv_path_ = stem + ".raw.csv";
  mf4_path_ = stem + ".mf4";

  asc_.open(asc_path_, std::ios::out | std::ios::trunc);
  if (!asc_) {
    if (error != nullptr) {
      *error = "failed to open ASC log " + asc_path_;
    }
    close(false);
    return false;
  }
  csv_.open(raw_csv_path_, std::ios::out | std::ios::trunc);
  if (!csv_) {
    if (error != nullptr) {
      *error = "failed to open raw CSV log " + raw_csv_path_;
    }
    close(false);
    return false;
  }
  opened_ = true;
  return writeHeader(error);
}

bool CanFdFrameLogger::writeHeader(std::string* error) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&now_time, &tm);

  asc_ << "date " << std::put_time(&tm, "%a %b %d %H:%M:%S %Y") << '\n';
  asc_ << "base hex  timestamps absolute" << '\n';
  asc_ << "internal events logged" << '\n';
  asc_ << "Begin Triggerblock" << '\n';
  csv_ << "timestamp_us,channel,can_id,is_extended,is_rtr,is_error,fd_flags,length";
  for (int i = 0; i < 64; ++i) {
    csv_ << ",data_" << std::setw(2) << std::setfill('0') << i;
  }
  csv_ << std::setfill(' ') << '\n';
  if (!asc_ || !csv_) {
    if (error != nullptr) {
      *error = "failed to write CAN-FD log headers";
    }
    close(false);
    return false;
  }
  return true;
}

bool CanFdFrameLogger::write(const RawCanFdFrame& frame, std::string* error) {
  if (!opened_) {
    if (error != nullptr) {
      *error = "CAN-FD frame logger is not open";
    }
    return false;
  }
  if (!have_first_timestamp_) {
    first_timestamp_us_ = frame.timestamp_us;
    have_first_timestamp_ = true;
  }
  const double relative_sec = static_cast<double>(frame.timestamp_us - first_timestamp_us_) / 1e6;
  asc_ << std::dec << std::fixed << std::setprecision(6) << relative_sec << ' ' << frame.channel
       << ' ' << ascId(frame) << " Rx FD " << std::dec << static_cast<unsigned int>(frame.len)
       << ' ' << std::uppercase << std::hex << std::setw(2)
       << std::setfill('0') << static_cast<unsigned int>(frame.flags) << std::setfill(' ');
  const uint8_t len = frame.len > frame.data.size() ? frame.data.size() : frame.len;
  for (uint8_t i = 0; i < len; ++i) {
    asc_ << ' ' << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(frame.data[i])
         << std::setfill(' ');
  }
  asc_ << std::dec << '\n';

  csv_ << std::dec << frame.timestamp_us << ',' << frame.channel << ',' << frame.can_id << ','
       << (frame.is_extended ? 1 : 0) << ',' << (frame.is_rtr ? 1 : 0) << ','
       << (frame.is_error ? 1 : 0) << ',' << static_cast<unsigned int>(frame.flags) << ','
       << static_cast<unsigned int>(frame.len);
  for (size_t i = 0; i < frame.data.size(); ++i) {
    csv_ << ',' << static_cast<unsigned int>(frame.data[i]);
  }
  csv_ << '\n';

  asc_.flush();
  csv_.flush();
  if (!asc_ || !csv_) {
    if (error != nullptr) {
      *error = "failed to write CAN-FD frame log";
    }
    close(false);
    return false;
  }
  return true;
}

void CanFdFrameLogger::close(bool convert_to_mf4) {
  const bool should_convert = opened_ && convert_to_mf4;
  if (asc_.is_open()) {
    asc_ << "End TriggerBlock" << '\n';
    asc_.flush();
    asc_.close();
  }
  if (csv_.is_open()) {
    csv_.flush();
    csv_.close();
  }
  opened_ = false;
  have_first_timestamp_ = false;
  first_timestamp_us_ = 0;
  if (should_convert) {
    convertRawCsvToMf4();
  }
}

bool CanFdFrameLogger::isOpen() const {
  return opened_;
}

bool CanFdFrameLogger::convertRawCsvToMf4() {
  const std::string package_path = ros::package::getPath("ars620_driver");
  if (package_path.empty()) {
    ROS_ERROR_STREAM("CAN-FD MF4 conversion skipped: cannot locate ars620_driver package");
    return false;
  }
  std::ostringstream command;
  command << "python3 '" << package_path << "/scripts/raw_canfd_csv_to_mf4.py' '" << raw_csv_path_
          << "' '" << mf4_path_ << "'";
  const int rc = std::system(command.str().c_str());
  if (rc != 0) {
    ROS_ERROR_STREAM("CAN-FD MF4 conversion failed. ASC and raw CSV are preserved. Install asammdf "
                     "with `python3 -m pip install asammdf` and run: "
                     << command.str());
    return false;
  }
  ROS_INFO_STREAM("Wrote CAN-FD MF4 log: " << mf4_path_);
  return true;
}

}  // namespace ars620_driver
