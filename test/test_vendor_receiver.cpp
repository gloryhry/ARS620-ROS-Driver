#include <gtest/gtest.h>

#include <ars620_driver/usbcanfd_receiver.h>

TEST(UsbCanFdReceiver, LoadsFakeLibraryAndReceivesStandardCanFdFrame) {
  ars620_driver::UsbCanFdConfig config;
  config.library_path = FAKE_CONTROLCANFD_PATH;
  config.device_type = 41;
  config.channel_index = 0;

  ars620_driver::UsbCanFdReceiver receiver;
  std::string error;
  ASSERT_TRUE(receiver.open(config, &error)) << error;
  ASSERT_TRUE(receiver.isOpen());

  std::vector<ars620_driver::CanFrame> frames;
  ASSERT_TRUE(receiver.receive(0, &frames, &error)) << error;
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].id, 0x100U);
  EXPECT_EQ(frames[0].len, 32U);
  EXPECT_EQ(frames[0].timestamp_us, 123456ULL);
}

TEST(UsbCanFdReceiver, ReportsMissingLibraryClearly) {
  ars620_driver::UsbCanFdConfig config;
  config.library_path = "/tmp/does-not-exist-libcontrolcanfd.so";

  ars620_driver::UsbCanFdReceiver receiver;
  std::string error;
  EXPECT_FALSE(receiver.open(config, &error));
  EXPECT_NE(error.find("dlopen"), std::string::npos);
}
