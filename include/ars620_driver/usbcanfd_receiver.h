#pragma once

#include <string>
#include <vector>

#include <ars620_driver/ars620_decoder.h>

namespace ars620_driver {

struct UsbCanFdConfig {
  std::string library_path = "src/ars620_driver/lib/libcontrolcanfd/x86_64/libcontrolcanfd.so";
  uint32_t device_type = 41;
  uint32_t device_index = 0;
  uint32_t channel_index = 0;
  uint32_t abit_baud = 500000;
  uint32_t dbit_baud = 2000000;
  uint32_t canfd_standard = 0;
  bool termination_enable = false;
};

class UsbCanFdReceiver {
 public:
  UsbCanFdReceiver();
  ~UsbCanFdReceiver();

  UsbCanFdReceiver(const UsbCanFdReceiver&) = delete;
  UsbCanFdReceiver& operator=(const UsbCanFdReceiver&) = delete;

  bool open(const UsbCanFdConfig& config, std::string* error);
  void close();
  bool receive(int wait_time_ms, std::vector<CanFrame>* frames, std::string* error);
  bool isOpen() const;

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace ars620_driver
