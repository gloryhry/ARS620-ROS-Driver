#include <gtest/gtest.h>

#include <ars620_driver/processing_timing.h>

#include <string>

TEST(ProcessingTimingStats, AccumulatesCountAverageAndMaximum) {
  ars620_driver::ProcessingTimingStats stats;
  stats.record("decode", 1.0);
  stats.record("decode", 3.0);
  stats.record("publish", 2.5);

  const auto decode = stats.stage("decode");
  ASSERT_TRUE(decode.has_value());
  EXPECT_EQ(decode->count, 2U);
  EXPECT_DOUBLE_EQ(decode->total_ms, 4.0);
  EXPECT_DOUBLE_EQ(decode->max_ms, 3.0);
  EXPECT_DOUBLE_EQ(decode->averageMs(), 2.0);

  const auto publish = stats.stage("publish");
  ASSERT_TRUE(publish.has_value());
  EXPECT_EQ(publish->count, 1U);
  EXPECT_DOUBLE_EQ(publish->averageMs(), 2.5);
}

TEST(ProcessingTimingStats, FormatsOnlyRecordedStagesWithMilliseconds) {
  ars620_driver::ProcessingTimingStats stats;
  stats.record("receive", 2.345);
  stats.record("receive", 4.0);
  stats.record("decode", 0.125);

  const std::string formatted = stats.format({"receive", "decode", "publish"});
  EXPECT_NE(formatted.find("samples=2"), std::string::npos);
  EXPECT_NE(formatted.find("receive avg=3.17 max=4.00 ms"), std::string::npos);
  EXPECT_NE(formatted.find("decode avg=0.12 max=0.12 ms"), std::string::npos);
  EXPECT_EQ(formatted.find("publish"), std::string::npos);
}

TEST(ProcessingTimingStats, ResetClearsRecordedSamples) {
  ars620_driver::ProcessingTimingStats stats;
  stats.record("loop", 5.0);
  ASSERT_TRUE(stats.stage("loop").has_value());

  stats.reset();

  EXPECT_FALSE(stats.stage("loop").has_value());
  EXPECT_TRUE(stats.empty());
  EXPECT_TRUE(stats.format({"loop"}).empty());
}

TEST(ScopedProcessingTimer, RecordsElapsedWallDurationOnDestruction) {
  ars620_driver::ProcessingTimingStats stats;
  {
    ars620_driver::ScopedProcessingTimer timer(&stats, "message", true);
    ros::WallDuration(0.001).sleep();
  }

  const auto message = stats.stage("message");
  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->count, 1U);
  EXPECT_GT(message->total_ms, 0.0);
}

TEST(ScopedProcessingTimer, DoesNothingWhenDisabled) {
  ars620_driver::ProcessingTimingStats stats;
  {
    ars620_driver::ScopedProcessingTimer timer(&stats, "message", false);
    ros::WallDuration(0.001).sleep();
  }

  EXPECT_TRUE(stats.empty());
}
