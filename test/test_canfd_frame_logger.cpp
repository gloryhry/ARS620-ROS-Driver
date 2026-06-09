#include <gtest/gtest.h>

#include <ars620_driver/canfd_frame_logger.h>

#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readFile(const std::string& path) {
  std::ifstream input(path);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

}  // namespace

TEST(CanFdFrameLogger, WritesAscAndRawCsvWithPayloadAndFlags) {
  ars620_driver::CanFdFrameLogger logger;
  const std::string directory = std::string(::testing::TempDir()) + "/ars620_canfd_logger_test";
  std::string error;
  ASSERT_TRUE(logger.open(directory, "20260609_102030", &error)) << error;

  ars620_driver::RawCanFdFrame frame;
  frame.can_id = 0x123;
  frame.is_extended = false;
  frame.is_rtr = false;
  frame.is_error = false;
  frame.flags = 0x05;
  frame.len = 3;
  frame.data[0] = 0x01;
  frame.data[1] = 0xAB;
  frame.data[2] = 0xFE;
  frame.timestamp_us = 900000ULL;

  ASSERT_TRUE(logger.write(frame, &error)) << error;
  frame.timestamp_us = 1000000ULL;
  ASSERT_TRUE(logger.write(frame, &error)) << error;
  logger.close(false);

  const std::string asc = readFile(directory + "/ars620_canfd_20260609_102030.asc");
  EXPECT_NE(asc.find("base hex  timestamps absolute"), std::string::npos);
  EXPECT_NE(asc.find("0.100000 1 123x Rx FD 3 05 01 AB FE"), std::string::npos);

  const std::string csv = readFile(directory + "/ars620_canfd_20260609_102030.raw.csv");
  EXPECT_NE(csv.find("timestamp_us,channel,can_id,is_extended,is_rtr,is_error,fd_flags,length"),
            std::string::npos);
  EXPECT_NE(csv.find("1000000,1,291,0,0,0,5,3,1,171,254"), std::string::npos);
}
