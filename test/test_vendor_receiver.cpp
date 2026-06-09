#include <gtest/gtest.h>

#include <ars620_driver/usbcanfd_receiver.h>

TEST(UsbCanFdReceiver, LoadsFakeLibraryAndReceivesAllRawFrames) {
  ars620_driver::UsbCanFdConfig config;
  config.library_path = FAKE_CONTROLCANFD_PATH;
  config.device_type = 41;
  config.channel_index = 0;

  ars620_driver::UsbCanFdReceiver receiver;
  std::string error;
  ASSERT_TRUE(receiver.open(config, &error)) << error;
  ASSERT_TRUE(receiver.isOpen());

  std::vector<ars620_driver::CanFrame> frames;
  std::vector<ars620_driver::RawCanFdFrame> raw_frames;
  ASSERT_TRUE(receiver.receive(0, &frames, &raw_frames, &error)) << error;
  ASSERT_EQ(raw_frames.size(), 4U);

  EXPECT_EQ(raw_frames[0].can_id, 0x100U);
  EXPECT_FALSE(raw_frames[0].is_extended);
  EXPECT_FALSE(raw_frames[0].is_rtr);
  EXPECT_FALSE(raw_frames[0].is_error);
  EXPECT_EQ(raw_frames[0].flags, 0x01U);
  EXPECT_EQ(raw_frames[0].len, 32U);
  EXPECT_EQ(raw_frames[0].data[0], 0x11U);
  EXPECT_EQ(raw_frames[0].timestamp_us, 123456ULL);

  EXPECT_EQ(raw_frames[1].can_id, 0x1ABCDEU);
  EXPECT_TRUE(raw_frames[1].is_extended);
  EXPECT_FALSE(raw_frames[1].is_rtr);
  EXPECT_FALSE(raw_frames[1].is_error);
  EXPECT_EQ(raw_frames[1].flags, 0x02U);
  EXPECT_EQ(raw_frames[1].timestamp_us, 123556ULL);

  EXPECT_EQ(raw_frames[2].can_id, 0x123U);
  EXPECT_TRUE(raw_frames[2].is_rtr);

  EXPECT_EQ(raw_frames[3].can_id, 0x321U);
  EXPECT_TRUE(raw_frames[3].is_error);

  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].id, 0x100U);
  EXPECT_EQ(frames[0].len, 32U);
  EXPECT_EQ(frames[0].timestamp_us, 123456ULL);
}

TEST(UsbCanFdReceiver, ExistingReceiveApiKeepsDecoderFacingFilter) {
  ars620_driver::UsbCanFdConfig config;
  config.library_path = FAKE_CONTROLCANFD_PATH;

  ars620_driver::UsbCanFdReceiver receiver;
  std::string error;
  ASSERT_TRUE(receiver.open(config, &error)) << error;

  std::vector<ars620_driver::CanFrame> frames;
  ASSERT_TRUE(receiver.receive(0, &frames, &error)) << error;
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].id, 0x100U);
}

TEST(UsbCanFdReceiver, ReportsMissingLibraryClearly) {
  ars620_driver::UsbCanFdConfig config;
  config.library_path = "/tmp/does-not-exist-libcontrolcanfd.so";

  ars620_driver::UsbCanFdReceiver receiver;
  std::string error;
  EXPECT_FALSE(receiver.open(config, &error));
  EXPECT_NE(error.find("dlopen"), std::string::npos);
}
