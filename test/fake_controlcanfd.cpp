#include <cstring>

#include <controlcanfd.h>

extern "C" {

namespace {
bool g_started = false;
}

DEVICE_HANDLE ZCAN_OpenDevice(UINT, UINT, UINT) {
  g_started = false;
  return reinterpret_cast<DEVICE_HANDLE>(0x1);
}

UINT ZCAN_CloseDevice(DEVICE_HANDLE) {
  return STATUS_OK;
}

UINT ZCAN_GetDeviceInf(DEVICE_HANDLE, ZCAN_DEVICE_INFO* info) {
  if (info != nullptr) {
    std::memset(info, 0, sizeof(*info));
    info->can_Num = 2;
  }
  return STATUS_OK;
}

CHANNEL_HANDLE ZCAN_InitCAN(DEVICE_HANDLE, UINT, ZCAN_CHANNEL_INIT_CONFIG*) {
  return reinterpret_cast<CHANNEL_HANDLE>(0x2);
}

UINT ZCAN_StartCAN(CHANNEL_HANDLE) {
  g_started = true;
  return STATUS_OK;
}

UINT ZCAN_ResetCAN(CHANNEL_HANDLE) {
  return STATUS_OK;
}

UINT ZCAN_ClearBuffer(CHANNEL_HANDLE) {
  if (!g_started) {
    return STATUS_ERR;
  }
  return STATUS_OK;
}

UINT ZCAN_ReceiveFD(CHANNEL_HANDLE, ZCAN_ReceiveFD_Data* frames, UINT len, int) {
  if (len == 0 || frames == nullptr) {
    return 0;
  }
  std::memset(frames, 0, sizeof(ZCAN_ReceiveFD_Data));
  frames[0].frame.can_id = MAKE_CAN_ID(0x100, 0, 0, 0);
  frames[0].frame.len = 32;
  frames[0].timestamp = 123456;
  return 1;
}

UINT ZCAN_SetAbitBaud(DEVICE_HANDLE, UINT, UINT) {
  return STATUS_OK;
}

UINT ZCAN_SetDbitBaud(DEVICE_HANDLE, UINT, UINT) {
  return STATUS_OK;
}

UINT ZCAN_SetCANFDStandard(DEVICE_HANDLE, UINT, UINT) {
  return STATUS_OK;
}

UINT ZCAN_SetResistanceEnable(DEVICE_HANDLE, UINT, UINT) {
  return STATUS_OK;
}

}
