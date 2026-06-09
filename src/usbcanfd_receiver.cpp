#include <ars620_driver/usbcanfd_receiver.h>

#include <controlcanfd.h>
#include <dlfcn.h>

#include <cstring>
#include <sstream>

namespace ars620_driver {
namespace {

using OpenDeviceFn = DEVICE_HANDLE (*)(UINT, UINT, UINT);
using CloseDeviceFn = UINT (*)(DEVICE_HANDLE);
using InitCanFn = CHANNEL_HANDLE (*)(DEVICE_HANDLE, UINT, ZCAN_CHANNEL_INIT_CONFIG*);
using StartCanFn = UINT (*)(CHANNEL_HANDLE);
using ResetCanFn = UINT (*)(CHANNEL_HANDLE);
using ClearBufferFn = UINT (*)(CHANNEL_HANDLE);
using ReceiveFdFn = UINT (*)(CHANNEL_HANDLE, ZCAN_ReceiveFD_Data*, UINT, int);
using SetBaudFn = UINT (*)(DEVICE_HANDLE, UINT, UINT);

template <typename Fn>
bool loadSymbol(void* handle, const char* name, Fn* out, std::string* error) {
  dlerror();
  *out = reinterpret_cast<Fn>(dlsym(handle, name));
  const char* err = dlerror();
  if (*out == nullptr || err != nullptr) {
    if (error != nullptr) {
      std::ostringstream oss;
      oss << "missing symbol " << name;
      if (err != nullptr) {
        oss << ": " << err;
      }
      *error = oss.str();
    }
    return false;
  }
  return true;
}

bool ok(UINT status) {
  return status == STATUS_OK;
}

}  // namespace

struct UsbCanFdReceiver::Impl {
  void* library = nullptr;
  DEVICE_HANDLE device = INVALID_DEVICE_HANDLE;
  CHANNEL_HANDLE channel = INVALID_CHANNEL_HANDLE;
  OpenDeviceFn open_device = nullptr;
  CloseDeviceFn close_device = nullptr;
  InitCanFn init_can = nullptr;
  StartCanFn start_can = nullptr;
  ResetCanFn reset_can = nullptr;
  ClearBufferFn clear_buffer = nullptr;
  ReceiveFdFn receive_fd = nullptr;
  SetBaudFn set_abit_baud = nullptr;
  SetBaudFn set_dbit_baud = nullptr;
  SetBaudFn set_canfd_standard = nullptr;
  SetBaudFn set_resistance_enable = nullptr;
};

UsbCanFdReceiver::UsbCanFdReceiver() : impl_(new Impl) {}

UsbCanFdReceiver::~UsbCanFdReceiver() {
  close();
  delete impl_;
}

bool UsbCanFdReceiver::open(const UsbCanFdConfig& config, std::string* error) {
  close();
  impl_->library = dlopen(config.library_path.c_str(), RTLD_NOW);
  if (impl_->library == nullptr) {
    if (error != nullptr) {
      *error = std::string("dlopen ") + config.library_path + " failed: " + dlerror();
    }
    return false;
  }

  if (!loadSymbol(impl_->library, "ZCAN_OpenDevice", &impl_->open_device, error) ||
      !loadSymbol(impl_->library, "ZCAN_CloseDevice", &impl_->close_device, error) ||
      !loadSymbol(impl_->library, "ZCAN_InitCAN", &impl_->init_can, error) ||
      !loadSymbol(impl_->library, "ZCAN_StartCAN", &impl_->start_can, error) ||
      !loadSymbol(impl_->library, "ZCAN_ResetCAN", &impl_->reset_can, error) ||
      !loadSymbol(impl_->library, "ZCAN_ClearBuffer", &impl_->clear_buffer, error) ||
      !loadSymbol(impl_->library, "ZCAN_ReceiveFD", &impl_->receive_fd, error) ||
      !loadSymbol(impl_->library, "ZCAN_SetAbitBaud", &impl_->set_abit_baud, error) ||
      !loadSymbol(impl_->library, "ZCAN_SetDbitBaud", &impl_->set_dbit_baud, error) ||
      !loadSymbol(impl_->library, "ZCAN_SetCANFDStandard", &impl_->set_canfd_standard, error) ||
      !loadSymbol(impl_->library, "ZCAN_SetResistanceEnable", &impl_->set_resistance_enable, error)) {
    close();
    return false;
  }

  impl_->device = impl_->open_device(config.device_type, config.device_index, 0);
  if (impl_->device == INVALID_DEVICE_HANDLE) {
    if (error != nullptr) {
      *error = "ZCAN_OpenDevice failed";
    }
    close();
    return false;
  }

  if (!ok(impl_->set_abit_baud(impl_->device, config.channel_index, config.abit_baud)) ||
      !ok(impl_->set_dbit_baud(impl_->device, config.channel_index, config.dbit_baud)) ||
      !ok(impl_->set_canfd_standard(impl_->device, config.channel_index, config.canfd_standard)) ||
      !ok(impl_->set_resistance_enable(impl_->device, config.channel_index,
                                       config.termination_enable ? 1U : 0U))) {
    if (error != nullptr) {
      *error = "failed to configure USBCAN-FD channel baud/CANFD/termination settings";
    }
    close();
    return false;
  }

  ZCAN_CHANNEL_INIT_CONFIG init{};
  init.can_type = TYPE_CANFD;
  init.canfd.acc_code = 0;
  init.canfd.acc_mask = 0xFFFFFFFF;
  init.canfd.filter = 1;
  init.canfd.mode = 0;
  init.canfd.brp = 0;
  impl_->channel = impl_->init_can(impl_->device, config.channel_index, &init);
  if (impl_->channel == INVALID_CHANNEL_HANDLE) {
    if (error != nullptr) {
      *error = "ZCAN_InitCAN failed";
    }
    close();
    return false;
  }
  if (!ok(impl_->start_can(impl_->channel)) || !ok(impl_->clear_buffer(impl_->channel))) {
    if (error != nullptr) {
      *error = "ZCAN_StartCAN or ZCAN_ClearBuffer failed";
    }
    close();
    return false;
  }

  if (error != nullptr) {
    error->clear();
  }
  return true;
}

void UsbCanFdReceiver::close() {
  if (impl_->channel != INVALID_CHANNEL_HANDLE && impl_->reset_can != nullptr) {
    impl_->reset_can(impl_->channel);
  }
  impl_->channel = INVALID_CHANNEL_HANDLE;
  if (impl_->device != INVALID_DEVICE_HANDLE && impl_->close_device != nullptr) {
    impl_->close_device(impl_->device);
  }
  impl_->device = INVALID_DEVICE_HANDLE;
  if (impl_->library != nullptr) {
    dlclose(impl_->library);
  }
  *impl_ = Impl{};
}

bool UsbCanFdReceiver::receive(int wait_time_ms, std::vector<CanFrame>* frames, std::string* error) {
  if (frames == nullptr) {
    return false;
  }
  frames->clear();
  if (!isOpen()) {
    if (error != nullptr) {
      *error = "USBCAN-FD receiver is not open";
    }
    return false;
  }
  ZCAN_ReceiveFD_Data rx[128]{};
  const UINT count = impl_->receive_fd(impl_->channel, rx, 128, wait_time_ms);
  frames->reserve(count);
  for (UINT i = 0; i < count; ++i) {
    const UINT raw_id = rx[i].frame.can_id;
    if (IS_EFF(raw_id) || IS_RTR(raw_id) || IS_ERR(raw_id)) {
      continue;
    }
    CanFrame frame;
    frame.id = GET_ID(raw_id);
    frame.len = rx[i].frame.len;
    frame.timestamp_us = rx[i].timestamp;
    const uint8_t copy_len = frame.len > frame.data.size() ? frame.data.size() : frame.len;
    std::memcpy(frame.data.data(), rx[i].frame.data, copy_len);
    frames->push_back(frame);
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool UsbCanFdReceiver::isOpen() const {
  return impl_->library != nullptr && impl_->device != INVALID_DEVICE_HANDLE &&
         impl_->channel != INVALID_CHANNEL_HANDLE;
}

}  // namespace ars620_driver
