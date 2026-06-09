#include <gtest/gtest.h>

#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <ars620_driver/ars620_decoder.h>
#include <ars620_driver/cycle_assembler.h>
#include <ars620_driver/pointcloud.h>

namespace {

using ars620_driver::CanFrame;
using ars620_driver::SignalSpec;

void setMotorola(uint8_t* data, const uint16_t start_bit, const uint8_t length, uint64_t value) {
  int dbc_bit = static_cast<int>(start_bit);
  for (uint8_t i = 0; i < length; ++i) {
    const int signal_bit = length - 1 - i;
    const bool set = ((value >> signal_bit) & 0x1U) != 0;
    const int byte = dbc_bit / 8;
    const int bit_in_byte = dbc_bit % 8;
    const uint8_t mask = static_cast<uint8_t>(1U << bit_in_byte);
    if (set) {
      data[byte] |= mask;
    } else {
      data[byte] &= static_cast<uint8_t>(~mask);
    }
    dbc_bit = (dbc_bit % 8 == 0) ? dbc_bit + 15 : dbc_bit - 1;
  }
}

void setPhys(CanFrame* frame, const SignalSpec& spec, const double physical) {
  const double raw_d = (physical - spec.offset) / spec.scale;
  int64_t raw_signed = static_cast<int64_t>(std::llround(raw_d));
  uint64_t raw = 0;
  if (raw_signed < 0) {
    raw = (1ULL << spec.length) + raw_signed;
  } else {
    raw = static_cast<uint64_t>(raw_signed);
  }
  setMotorola(frame->data.data(), spec.start_bit, spec.length, raw);
}

CanFrame frame(uint32_t id, uint8_t len) {
  CanFrame out;
  out.id = id;
  out.len = len;
  return out;
}

}  // namespace

TEST(Ars620Decoder, ExtractsMotorolaBigEndianSignals) {
  CanFrame f = frame(0x100, 32);
  setMotorola(f.data.data(), 31, 32, 0x12345678);
  setMotorola(f.data.data(), 159, 10, 511);

  EXPECT_EQ(ars620_driver::extractMotorola(f.data.data(), f.len, 31, 32), 0x12345678U);
  EXPECT_EQ(ars620_driver::extractMotorola(f.data.data(), f.len, 159, 10), 511U);
  EXPECT_DOUBLE_EQ(ars620_driver::decodeSignal(f.data.data(), f.len, {159, 10, 1.0, 0.0, false}), 511.0);
}

TEST(Ars620Decoder, DecodesConfigAndSystemStatusFrames) {
  CanFrame cfg = frame(0x50, 32);
  setMotorola(cfg.data.data(), 31, 8, 3);
  setMotorola(cfg.data.data(), 32, 1, 1);
  setPhys(&cfg, {47, 12, 0.01, 0.0, false}, 16.23);
  setPhys(&cfg, {151, 11, 0.001, -1.023, false}, 0.125);
  setMotorola(cfg.data.data(), 255, 8, 9);
  ars620_driver::ConfigState decoded_cfg;
  ASSERT_TRUE(ars620_driver::decodeConfigState(cfg, &decoded_cfg));
  EXPECT_EQ(decoded_cfg.mode, 3);
  EXPECT_TRUE(decoded_cfg.time_sync_enable);
  EXPECT_NEAR(decoded_cfg.steering_ratio, 16.23, 1e-6);
  EXPECT_NEAR(decoded_cfg.lateral_position_m, 0.125, 1e-6);
  EXPECT_EQ(decoded_cfg.msg_group_counter, 9);

  CanFrame status = frame(0x52, 20);
  setMotorola(status.data.data(), 31, 4, 7);
  setMotorola(status.data.data(), 27, 3, 5);
  setPhys(&status, {39, 8, 0.0015, -0.1905, false}, 0.0);
  setMotorola(status.data.data(), 55, 9, 123);
  setMotorola(status.data.data(), 71, 8, 1);
  setMotorola(status.data.data(), 79, 8, 7);
  setMotorola(status.data.data(), 87, 8, 17);
  setMotorola(status.data.data(), 88, 1, 1);
  setMotorola(status.data.data(), 143, 1, 1);
  ars620_driver::SystemStatus decoded_status;
  ASSERT_TRUE(ars620_driver::decodeSystemStatus(status, &decoded_status));
  EXPECT_EQ(decoded_status.operation_mode, 7);
  EXPECT_EQ(decoded_status.calibration_state, 5);
  EXPECT_NEAR(decoded_status.azimuth_correction_rad, 0.0, 1e-9);
  EXPECT_EQ(decoded_status.visibility_range_m, 123);
  EXPECT_EQ(decoded_status.software_version_major, 1);
  EXPECT_EQ(decoded_status.software_version_minor, 7);
  EXPECT_EQ(decoded_status.software_version_patch, 17);
  EXPECT_TRUE(decoded_status.dtc_sensor_full_blockage);
  EXPECT_TRUE(decoded_status.dtc_gearpos_invalid);
}

TEST(Ars620Decoder, DecodesRdiHeaderAndEightClusterPayload) {
  CanFrame header = frame(0x100, 32);
  setMotorola(header.data.data(), 31, 32, 10);
  setMotorola(header.data.data(), 63, 32, 20);
  setMotorola(header.data.data(), 95, 32, 30);
  setMotorola(header.data.data(), 127, 16, 40);
  setMotorola(header.data.data(), 143, 16, 55);
  setMotorola(header.data.data(), 159, 10, 8);
  setPhys(&header, {183, 11, 0.125, -128.0, false}, 12.5);
  ars620_driver::RdiHeader decoded_header;
  ASSERT_TRUE(ars620_driver::decodeRdiHeader(header, &decoded_header));
  EXPECT_EQ(decoded_header.global_timestamp_sec, 10U);
  EXPECT_EQ(decoded_header.global_timestamp_nsec, 20U);
  EXPECT_EQ(decoded_header.local_timestamp_us, 30U);
  EXPECT_EQ(decoded_header.meas_counter, 40);
  EXPECT_EQ(decoded_header.cycle_counter, 55);
  EXPECT_EQ(decoded_header.num_clusters, 8);
  EXPECT_NEAR(decoded_header.ego_velocity_mps, 12.5, 1e-9);

  CanFrame payload = frame(0x101, 64);
  setPhys(&payload, {31, 12, 0.07, 0.0, false}, 14.0);
  setPhys(&payload, {35, 11, 0.1, -120.0, false}, -3.2);
  setPhys(&payload, {40, 9, 0.2, -51.2, false}, 6.0);
  setPhys(&payload, {63, 10, 0.0025, -1.2775, false}, 0.25);
  setPhys(&payload, {69, 7, 0.005, -0.315, false}, 0.10);
  setPhys(&payload, {78, 7, 0.5, 0.0, false}, 15.5);
  setMotorola(payload.data.data(), 87, 2, 2);
  setMotorola(payload.data.data(), 85, 2, 3);
  std::vector<ars620_driver::RdiTarget> targets;
  ASSERT_TRUE(ars620_driver::decodeRdiTargets(payload, &targets));
  ASSERT_EQ(targets.size(), 8U);
  EXPECT_EQ(targets[0].cluster_index, 0);
  EXPECT_NEAR(targets[0].range, 14.0, 1e-9);
  EXPECT_NEAR(targets[0].azimuth, 0.25, 1e-9);
  EXPECT_NEAR(targets[0].elevation, 0.10, 1e-9);
  EXPECT_NEAR(targets[0].vrad_rel, -3.2, 1e-9);
  EXPECT_NEAR(targets[0].rcs, 6.0, 1e-9);
  EXPECT_NEAR(targets[0].snr, 15.5, 1e-9);
  EXPECT_EQ(targets[0].dyn_prop, 2);
  EXPECT_EQ(targets[0].quality, 3);
  EXPECT_NEAR(targets[0].x, 14.0 * std::cos(0.10) * std::cos(0.25), 1e-6);
  EXPECT_NEAR(targets[0].y, 14.0 * std::cos(0.10) * std::sin(0.25), 1e-6);
  EXPECT_NEAR(targets[0].z, 14.0 * std::sin(0.10), 1e-6);
}

TEST(Ars620Decoder, DecodesOdHeaderAndTwoObjectPayload) {
  CanFrame header = frame(0x200, 32);
  setMotorola(header.data.data(), 31, 32, 100);
  setMotorola(header.data.data(), 159, 6, 2);
  setPhys(&header, {167, 11, 0.1, 30.0, false}, 45.0);
  ars620_driver::OdHeader decoded_header;
  ASSERT_TRUE(ars620_driver::decodeOdHeader(header, &decoded_header));
  EXPECT_EQ(decoded_header.global_timestamp_sec, 100U);
  EXPECT_EQ(decoded_header.num_objects, 2);
  EXPECT_NEAR(decoded_header.latency_ms, 45.0, 1e-9);

  CanFrame payload = frame(0x201, 64);
  setPhys(&payload, {31, 12, 0.07, 0.0, false}, 21.0);
  setPhys(&payload, {63, 12, 0.08, -163.76, false}, -3.2);
  setPhys(&payload, {35, 11, 0.1, -120.0, false}, 8.4);
  setPhys(&payload, {67, 11, 0.1, -120.0, false}, -1.5);
  setPhys(&payload, {40, 9, 0.125, -31.875, false}, 0.5);
  setPhys(&payload, {72, 9, 0.125, -31.875, false}, -0.25);
  setMotorola(payload.data.data(), 122, 3, 4);
  setMotorola(payload.data.data(), 136, 7, 80);
  setMotorola(payload.data.data(), 154, 3, 5);
  setPhys(&payload, {160, 7, 0.2, 0.0, false}, 4.0);
  setMotorola(payload.data.data(), 169, 2, 2);
  setPhys(&payload, {183, 7, 0.2, 0.0, false}, 2.0);
  setPhys(&payload, {176, 9, 0.2, -51.2, false}, 12.0);
  setPhys(&payload, {199, 10, 0.01, -5.11, false}, 0.30);
  setPhys(&payload, {205, 10, 0.001, -0.511, false}, 0.02);
  setMotorola(payload.data.data(), 487, 8, 42);
  std::vector<ars620_driver::OdTarget> targets;
  ASSERT_TRUE(ars620_driver::decodeOdTargets(payload, &targets));
  ASSERT_EQ(targets.size(), 2U);
  EXPECT_EQ(targets[0].object_id, 42);
  EXPECT_NEAR(targets[0].x, 21.0, 1e-9);
  EXPECT_NEAR(targets[0].y, -3.2, 1e-9);
  EXPECT_NEAR(targets[0].vx, 8.4, 1e-9);
  EXPECT_NEAR(targets[0].vy, -1.5, 1e-9);
  EXPECT_NEAR(targets[0].ax, 0.5, 1e-9);
  EXPECT_NEAR(targets[0].ay, -0.25, 1e-9);
  EXPECT_EQ(targets[0].classification, 4);
  EXPECT_EQ(targets[0].dyn_prop, 5);
  EXPECT_EQ(targets[0].prob_of_exist, 80);
  EXPECT_NEAR(targets[0].length, 4.0, 1e-9);
  EXPECT_NEAR(targets[0].width, 2.0, 1e-9);
  EXPECT_NEAR(targets[0].rcs, 12.0, 1e-9);
  EXPECT_NEAR(targets[0].orientation, 0.30, 1e-9);
  EXPECT_NEAR(targets[0].yaw_rate, 0.02, 1e-9);
  EXPECT_EQ(targets[0].maintenance_state, 2);
}

TEST(Ars620Decoder, BuildsPointCloud2FieldsAndValues) {
  std_msgs::Header header;
  header.frame_id = "ars620";
  ars620_driver::RdiTarget rdi;
  rdi.x = 1.0;
  rdi.y = 2.0;
  rdi.z = 3.0;
  rdi.range = 4.0;
  rdi.cluster_index = 9;
  const auto cloud = ars620_driver::makeRdiPointCloud(header, {rdi});
  ASSERT_EQ(cloud.width, 1U);
  ASSERT_EQ(cloud.height, 1U);
  ASSERT_EQ(cloud.fields.size(), 12U);
  EXPECT_EQ(cloud.fields[0].name, "x");
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> range(cloud, "range");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> index(cloud, "cluster_index");
  EXPECT_FLOAT_EQ(*x, 1.0F);
  EXPECT_FLOAT_EQ(*range, 4.0F);
  EXPECT_EQ(*index, 9U);
}

TEST(Ars620Decoder, AssemblersPublishCompleteAndTimeoutPartialCycles) {
  ars620_driver::RdiAssembler complete_only(false, ros::Duration(0.1));
  ars620_driver::RdiHeader header;
  header.num_clusters = 2;
  ars620_driver::RdiCycle completed;
  EXPECT_FALSE(complete_only.processHeader(header, ros::Time(1.0), &completed));
  EXPECT_FALSE(complete_only.processTargets({ars620_driver::RdiTarget{}}, &completed));
  EXPECT_FALSE(complete_only.pollTimeout(ros::Time(2.0), &completed));
  std::vector<ars620_driver::RdiTarget> remaining(1);
  EXPECT_TRUE(complete_only.processTargets(remaining, &completed));
  EXPECT_EQ(completed.targets.size(), 2U);

  ars620_driver::OdAssembler partial(true, ros::Duration(0.1));
  ars620_driver::OdHeader od_header;
  od_header.num_objects = 2;
  ars620_driver::OdCycle od_completed;
  EXPECT_FALSE(partial.processHeader(od_header, ros::Time(1.0), &od_completed));
  EXPECT_FALSE(partial.processTargets({ars620_driver::OdTarget{}}, &od_completed));
  EXPECT_TRUE(partial.pollTimeout(ros::Time(1.2), &od_completed));
  EXPECT_EQ(od_completed.targets.size(), 1U);
}
