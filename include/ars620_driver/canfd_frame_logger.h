#pragma once

#include <ars620_driver/usbcanfd_receiver.h>

#include <fstream>
#include <string>

namespace ars620_driver {

class CanFdFrameLogger {
 public:
  CanFdFrameLogger() = default;
  ~CanFdFrameLogger();

  CanFdFrameLogger(const CanFdFrameLogger&) = delete;
  CanFdFrameLogger& operator=(const CanFdFrameLogger&) = delete;

  bool open(const std::string& directory, std::string* error);
  bool open(const std::string& directory, const std::string& timestamp, std::string* error);
  bool write(const RawCanFdFrame& frame, std::string* error);
  void close(bool convert_to_mf4 = true);

  bool isOpen() const;
  const std::string& ascPath() const { return asc_path_; }
  const std::string& rawCsvPath() const { return raw_csv_path_; }
  const std::string& mf4Path() const { return mf4_path_; }

 private:
  bool writeHeader(std::string* error);
  bool convertRawCsvToMf4();

  std::ofstream asc_;
  std::ofstream csv_;
  std::string asc_path_;
  std::string raw_csv_path_;
  std::string mf4_path_;
  uint64_t first_timestamp_us_ = 0;
  bool have_first_timestamp_ = false;
  bool opened_ = false;
};

}  // namespace ars620_driver

