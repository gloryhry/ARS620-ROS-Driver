#include <ars620_driver/ars620_decoder.h>
#include <ars620_driver/canfd_frame_logger.h>
#include <ars620_driver/cycle_assembler.h>
#include <ars620_driver/pointcloud.h>
#include <ars620_driver/processing_timing.h>
#include <ars620_driver/usbcanfd_receiver.h>

#include <ars620_driver/Ars620ConfigState.h>
#include <ars620_driver/Ars620OdTargetArray.h>
#include <ars620_driver/Ars620RdiTargetArray.h>
#include <ars620_driver/Ars620SystemStatus.h>

#include <ros/ros.h>

#include <map>
#include <sstream>
#include <vector>

namespace ars620_driver {
namespace {

enum class DecodedFrameType {
  kNone,
  kConfigState,
  kSystemStatus,
  kRdiHeader,
  kRdiTargets,
  kOdHeader,
  kOdTargets,
};

const std::vector<std::string>& timingStageOrder() {
  static const std::vector<std::string> stages = {"receive", "raw_log", "raw_summary", "decode",
                                                  "assemble", "message", "pointcloud", "publish",
                                                  "loop"};
  return stages;
}

void maybeLogTimingStats(ProcessingTimingStats* timing_stats, const bool debug_timing,
                         const double timing_log_period, ros::WallTime* next_log_time) {
  if (!debug_timing || timing_stats->empty()) {
    return;
  }
  const ros::WallTime now = ros::WallTime::now();
  if (now < *next_log_time) {
    return;
  }
  const std::string formatted = timing_stats->format(timingStageOrder());
  if (!formatted.empty()) {
    ROS_INFO_STREAM_THROTTLE(timing_log_period, "ARS620 timing: " << formatted);
  }
  timing_stats->reset();
  *next_log_time = now + ros::WallDuration(timing_log_period);
}

ros::Time stampForFrame(const CanFrame& frame, const std::string& policy) {
  if (policy == "vendor_timestamp" && frame.timestamp_us > 0) {
    return ros::Time().fromNSec(frame.timestamp_us * 1000ULL);
  }
  return ros::Time::now();
}

std::string formatFrameIds(const std::vector<uint32_t>& frame_ids) {
  std::ostringstream oss;
  for (const auto frame_id : frame_ids) {
    if (oss.tellp() > 0) {
      oss << ' ';
    }
    oss << "0x" << std::hex << frame_id << std::dec;
  }
  return oss.str();
}

Ars620ConfigState toMsg(const ConfigState& in, const std_msgs::Header& header) {
  Ars620ConfigState msg;
  msg.header = header;
  msg.crc16_checksum = in.crc16_checksum;
  msg.msg_counter = in.msg_counter;
  msg.mode = in.mode;
  msg.time_sync_enable = in.time_sync_enable;
  msg.steering_ratio = in.steering_ratio;
  msg.wheel_base = in.wheel_base_m;
  msg.track_width_front = in.track_width_front_m;
  msg.track_width_rear = in.track_width_rear_m;
  msg.vehicle_weight = in.vehicle_weight_kg;
  msg.center_of_gravity_height = in.center_of_gravity_height_m;
  msg.axis_load_distribution = in.axis_load_distribution_pct;
  msg.sensor_lat_pos = in.lateral_position_m;
  msg.sensor_long_pos = in.longitudinal_position_m;
  msg.sensor_vert_pos = in.vertical_position_m;
  msg.sensor_long_pos_to_cog = in.longitudinal_position_to_cog_m;
  msg.sensor_yaw_angle = in.yaw_angle_rad;
  msg.sensor_orientation = in.sensor_orientation ? 1 : 0;
  msg.cover_damping = in.cover_damping_db;
  msg.steering_ratio_valid = in.str_ratio_configured;
  msg.wheel_base_valid = in.wheel_base_configured;
  msg.track_width_front_valid = in.track_width_front_configured;
  msg.track_width_rear_valid = in.track_width_rear_configured;
  msg.vehicle_weight_valid = in.vehicle_weight_configured;
  msg.center_of_gravity_height_valid = in.center_of_gravity_height_configured;
  msg.axis_load_distribution_valid = in.axis_load_distribution_configured;
  msg.sensor_lat_pos_valid = in.lateral_position_configured;
  msg.sensor_long_pos_valid = in.longitudinal_position_configured;
  msg.sensor_vert_pos_valid = in.vertical_position_configured;
  msg.sensor_long_pos_to_cog_valid = in.longitudinal_position_to_cog_configured;
  msg.sensor_yaw_angle_valid = in.yaw_angle_configured;
  msg.sensor_orientation_valid = in.sensor_orientation_configured;
  msg.cover_damping_valid = in.cover_damping_configured;
  msg.msg_group_counter = in.msg_group_counter;
  return msg;
}

Ars620SystemStatus toMsg(const SystemStatus& in, const std_msgs::Header& header) {
  Ars620SystemStatus msg;
  msg.header = header;
  msg.crc16_checksum = in.crc16_checksum;
  msg.msg_counter = in.msg_counter;
  msg.operation_mode = in.operation_mode;
  msg.calibration_state = in.calibration_state;
  msg.tunnel_flag = in.tunnel;
  msg.azimuth_correction = in.azimuth_correction_rad;
  msg.elevation_correction = in.elevation_correction_rad;
  msg.visibility_range = in.visibility_range_m;
  msg.visibility_state = in.visibility_state;
  msg.sw_version_major = in.software_version_major;
  msg.sw_version_minor = in.software_version_minor;
  msg.sw_version_patch = in.software_version_patch;
  msg.dtc_sensor_full_blockage = in.dtc_sensor_full_blockage;
  msg.dtc_sensor_partial_blockage = in.dtc_sensor_partial_blockage;
  msg.dtc_online_calib_failed = in.dtc_online_calib_failed;
  msg.dtc_alignment_fail = in.dtc_alignment_fail;
  msg.dtc_alignment_never_done = in.dtc_alignment_never_done;
  msg.dtc_vbat_high = in.dtc_vbat_high;
  msg.dtc_vbat_low = in.dtc_vbat_low;
  msg.dtc_private_can_busoff = in.dtc_private_can_busoff;
  msg.dtc_vehicle_config_param_invalid = in.dtc_vehicle_config_invalid;
  msg.dtc_vdy_parameter_exception = in.dtc_vdy_parameter_exception;
  msg.dtc_temperature_too_high = in.dtc_temperature_too_high;
  msg.dtc_temperature_too_low = in.dtc_temperature_too_low;
  msg.dtc_nvm_access_error = in.dtc_nvm_access_error;
  msg.dtc_sw_failure = in.dtc_sw_failure;
  msg.dtc_hw_plausibility_failure = in.dtc_hw_plausibility_failure;
  msg.dtc_hw_failure = in.dtc_hw_failure;
  msg.dtc_idcm_timeout = in.dtc_idcm_timeout;
  msg.dtc_tbox_timeout = in.dtc_tbox_timeout;
  msg.dtc_tm_timeout = in.dtc_tm_timeout;
  msg.dtc_sas_timeout = in.dtc_sas_timeout;
  msg.dtc_esc_timeout = in.dtc_esc_timeout;
  msg.dtc_acu_timeout = in.dtc_acu_timeout;
  msg.dtc_time_sync_fail = in.dtc_time_sync_fail;
  msg.dtc_vdy_signal_exception = in.dtc_vdy_signal_exception;
  msg.dtc_tm_rollingcounter_error = in.dtc_tm_rollingcounter_error;
  msg.dtc_sas_rollingcounter_error = in.dtc_sas_rollingcounter_error;
  msg.dtc_esc_rollingcounter_error = in.dtc_esc_rollingcounter_error;
  msg.dtc_acu_rollingcounter_error = in.dtc_acu_rollingcounter_error;
  msg.dtc_tm_checksum_error = in.dtc_tm_checksum_error;
  msg.dtc_sas_checksum_error = in.dtc_sas_checksum_error;
  msg.dtc_esc_checksum_error = in.dtc_esc_checksum_error;
  msg.dtc_acu_checksum_error = in.dtc_acu_checksum_error;
  msg.dtc_wheel_direction_fl_invalid = in.dtc_whldir_fl_invalid;
  msg.dtc_wheel_speed_rr_invalid = in.dtc_whlspd_rr_invalid;
  msg.dtc_wheel_speed_rl_invalid = in.dtc_whlspd_rl_invalid;
  msg.dtc_wheel_speed_fr_invalid = in.dtc_whlspd_fr_invalid;
  msg.dtc_wheel_speed_fl_invalid = in.dtc_whlspd_fl_invalid;
  msg.dtc_yawrate_invalid = in.dtc_yawrate_invalid;
  msg.dtc_lateralaccel_invalid = in.dtc_lateralaccel_invalid;
  msg.dtc_long_accel_invalid = in.dtc_long_accel_invalid;
  msg.dtc_steeringwheel_angle_invalid = in.dtc_strwhl_ang_invalid;
  msg.dtc_epb_invalid = in.dtc_epb_invalid;
  msg.dtc_tcs_invalid = in.dtc_tcs_invalid;
  msg.dtc_abs_invalid = in.dtc_abs_invalid;
  msg.dtc_vehicle_speed_invalid = in.dtc_vehspeed_invalid;
  msg.dtc_wheel_direction_rr_invalid = in.dtc_whldir_rr_invalid;
  msg.dtc_wheel_direction_rl_invalid = in.dtc_whldir_rl_invalid;
  msg.dtc_wheel_direction_fr_invalid = in.dtc_whldir_fr_invalid;
  msg.dtc_gearpos_invalid = in.dtc_gearpos_invalid;
  msg.msg_group_counter = in.msg_group_counter;
  return msg;
}

Ars620RdiTargetArray toMsg(const RdiCycle& cycle, const std_msgs::Header& header) {
  Ars620RdiTargetArray msg;
  msg.header = header;
  msg.radar_timestamp_sec = cycle.header.global_timestamp_sec;
  msg.radar_timestamp_nsec = cycle.header.global_timestamp_nsec;
  msg.radar_timestamp_local_us = cycle.header.local_timestamp_us;
  msg.cycle_counter = cycle.header.cycle_counter;
  msg.meas_counter = cycle.header.meas_counter;
  msg.num_clusters = static_cast<uint16_t>(cycle.targets.size());
  if (cycle.header.num_clusters > cycle.targets.size()) {
    ROS_WARN_STREAM_THROTTLE(1.0, "RDI target count limited from header value "
                                      << cycle.header.num_clusters << " to published count "
                                      << cycle.targets.size());
  }
  msg.max_detection_range = cycle.header.max_detection_range_m;
  msg.ego_velocity = cycle.header.ego_velocity_mps;
  msg.ego_yaw_rate = cycle.header.ego_yaw_rate_radps;
  msg.latency_ms = cycle.header.latency_ms;
  msg.ego_velocity_std = cycle.header.ego_velocity_std_mps;
  msg.ego_acceleration = cycle.header.ego_acceleration_mps2;
  msg.ego_curvature = cycle.header.ego_curvature_inv_m;
  msg.ambig_free_doppler_range = cycle.header.ambig_free_doppler_range_mps;
  msg.task_valid = cycle.header.task_valid;
  msg.extended_cycle = cycle.header.extended_cycle;
  msg.msg_group_counter = cycle.header.msg_group_counter;
  msg.targets.reserve(cycle.targets.size());
  for (const auto& in : cycle.targets) {
    Ars620RdiTarget target;
    target.cluster_index = in.cluster_index;
    target.x = in.x;
    target.y = in.y;
    target.z = in.z;
    target.range = in.range;
    target.azimuth = in.azimuth;
    target.elevation = in.elevation;
    target.vrad_rel = in.vrad_rel;
    target.rcs = in.rcs;
    target.snr = in.snr;
    target.dyn_prop = in.dyn_prop;
    target.quality = in.quality;
    msg.targets.push_back(target);
  }
  return msg;
}

Ars620OdTargetArray toMsg(const OdCycle& cycle, const std_msgs::Header& header) {
  Ars620OdTargetArray msg;
  msg.header = header;
  msg.radar_timestamp_sec = cycle.header.global_timestamp_sec;
  msg.radar_timestamp_nsec = cycle.header.global_timestamp_nsec;
  msg.radar_timestamp_local_us = cycle.header.local_timestamp_us;
  msg.cycle_counter = cycle.header.cycle_counter;
  msg.meas_counter = cycle.header.meas_counter;
  msg.num_objects = cycle.header.num_objects;
  msg.latency_ms = cycle.header.latency_ms;
  msg.ego_velocity = cycle.header.ego_velocity_mps;
  msg.ego_yaw_rate = cycle.header.ego_yaw_rate_radps;
  msg.ego_velocity_std = cycle.header.ego_velocity_std_mps;
  msg.ego_acceleration = cycle.header.ego_acceleration_mps2;
  msg.ego_curvature = cycle.header.ego_curvature_inv_m;
  msg.task_valid = cycle.header.task_valid;
  msg.extended_cycle = cycle.header.extended_cycle;
  msg.msg_group_counter = cycle.header.msg_group_counter;
  msg.targets.reserve(cycle.targets.size());
  for (const auto& in : cycle.targets) {
    Ars620OdTarget target;
    target.object_id = in.object_id;
    target.x = in.x;
    target.y = in.y;
    target.z = in.z;
    target.vx = in.vx;
    target.vy = in.vy;
    target.ax = in.ax;
    target.ay = in.ay;
    target.rcs = in.rcs;
    target.length = in.length;
    target.width = in.width;
    target.orientation = in.orientation;
    target.yaw_rate = in.yaw_rate;
    target.classification = in.classification;
    target.dyn_prop = in.dyn_prop;
    target.prob_of_exist = in.prob_of_exist;
    target.maintenance_state = in.maintenance_state;
    target.class_confidence = in.class_confidence;
    target.mirror_probability = in.mirror_probability;
    target.obstruction_type = in.obstruction_type;
    target.ref_point = in.ref_point;
    target.life_cycle = in.life_cycle;
    target.x_std = in.x_std;
    target.y_std = in.y_std;
    target.vx_std = in.vx_std;
    target.vy_std = in.vy_std;
    target.ax_std = in.ax_std;
    target.ay_std = in.ay_std;
    target.orientation_std = in.orientation_std;
    target.yaw_rate_std = in.yaw_rate_std;
    target.obstacle_probability = in.obstacle_probability;
    msg.targets.push_back(target);
  }
  return msg;
}

}  // namespace
}  // namespace ars620_driver

int main(int argc, char** argv) {
  ros::init(argc, argv, "ars620_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ars620_driver::UsbCanFdConfig config;
  pnh.param<std::string>("library_path", config.library_path, config.library_path);
  int device_type = config.device_type;
  int device_index = config.device_index;
  int channel_index = config.channel_index;
  int abit_baud = config.abit_baud;
  int dbit_baud = config.dbit_baud;
  int canfd_standard = config.canfd_standard;
  pnh.param("device_type", device_type, device_type);
  pnh.param("device_index", device_index, device_index);
  pnh.param("channel_index", channel_index, channel_index);
  pnh.param("abit_baud", abit_baud, abit_baud);
  pnh.param("dbit_baud", dbit_baud, dbit_baud);
  pnh.param("canfd_standard", canfd_standard, canfd_standard);
  pnh.param("termination_enable", config.termination_enable, config.termination_enable);
  config.device_type = static_cast<uint32_t>(device_type);
  config.device_index = static_cast<uint32_t>(device_index);
  config.channel_index = static_cast<uint32_t>(channel_index);
  config.abit_baud = static_cast<uint32_t>(abit_baud);
  config.dbit_baud = static_cast<uint32_t>(dbit_baud);
  config.canfd_standard = static_cast<uint32_t>(canfd_standard);

  std::string frame_id = "ars620";
  std::string stamp_policy = "vendor_timestamp";
  bool publish_partial = false;
  bool debug_raw_frames = false;
  bool debug_timing = false;
  bool save_all_canfd_frames = false;
  std::string save_all_canfd_path = "~/.ros/ars620_canfd_logs";
  double partial_timeout = 0.1;
  double timing_log_period = 1.0;
  int receive_wait_ms = 20;
  int rdi_max_targets = 256;
  pnh.param("frame_id", frame_id, frame_id);
  pnh.param("stamp_policy", stamp_policy, stamp_policy);
  pnh.param("publish_partial", publish_partial, publish_partial);
  pnh.param("debug_raw_frames", debug_raw_frames, debug_raw_frames);
  pnh.param("debug_timing", debug_timing, debug_timing);
  pnh.param("save_all_canfd_frames", save_all_canfd_frames, save_all_canfd_frames);
  pnh.param("save_all_canfd_path", save_all_canfd_path, save_all_canfd_path);
  pnh.param("partial_timeout", partial_timeout, partial_timeout);
  pnh.param("timing_log_period", timing_log_period, timing_log_period);
  pnh.param("receive_wait_ms", receive_wait_ms, receive_wait_ms);
  pnh.param("rdi_max_targets", rdi_max_targets, rdi_max_targets);
  if (timing_log_period <= 0.0) {
    ROS_WARN_STREAM("timing_log_period must be positive; using 1.0 seconds");
    timing_log_period = 1.0;
  }
  if (rdi_max_targets < 0) {
    ROS_WARN_STREAM("rdi_max_targets must be >= 0; using 256");
    rdi_max_targets = 256;
  }

  ros::Publisher rdi_points_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/ars620/rdi_points", 10);
  ros::Publisher od_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/ars620/od_points", 10);
  ros::Publisher config_pub =
      nh.advertise<ars620_driver::Ars620ConfigState>("/ars620/config_state", 10);
  ros::Publisher status_pub =
      nh.advertise<ars620_driver::Ars620SystemStatus>("/ars620/system_status", 10);
  ros::Publisher rdi_targets_pub =
      nh.advertise<ars620_driver::Ars620RdiTargetArray>("/ars620/rdi_targets", 10);
  ros::Publisher od_targets_pub =
      nh.advertise<ars620_driver::Ars620OdTargetArray>("/ars620/od_targets", 10);

  ars620_driver::UsbCanFdReceiver receiver;
  std::string error;
  if (!receiver.open(config, &error)) {
    ROS_FATAL_STREAM("Failed to open USBCAN-FD receiver: " << error);
    return 1;
  }

  ars620_driver::CanFdFrameLogger canfd_logger;
  if (save_all_canfd_frames) {
    if (!canfd_logger.open(save_all_canfd_path, &error)) {
      ROS_ERROR_STREAM("Failed to enable CAN-FD full-frame logging: " << error);
    } else {
      ROS_INFO_STREAM("Saving all CAN-FD frames to " << canfd_logger.ascPath() << " and "
                                            << canfd_logger.rawCsvPath());
    }
  }

  ars620_driver::RdiAssembler rdi_assembler(
      publish_partial, ros::Duration(partial_timeout), static_cast<size_t>(rdi_max_targets));
  ars620_driver::OdAssembler od_assembler(publish_partial, ros::Duration(partial_timeout));
  std::vector<ars620_driver::CanFrame> frames;
  std::vector<ars620_driver::RawCanFdFrame> raw_frames;
  ars620_driver::ProcessingTimingStats timing_stats;
  ros::WallTime next_timing_log = ros::WallTime::now() + ros::WallDuration(timing_log_period);
  ros::Rate idle_rate(200.0);
  while (ros::ok()) {
    {
      ars620_driver::ScopedProcessingTimer loop_timer(&timing_stats, "loop", debug_timing);
      bool received = false;
      {
        ars620_driver::ScopedProcessingTimer timer(&timing_stats, "receive", debug_timing);
        received = receiver.receive(receive_wait_ms, &frames,
                                    save_all_canfd_frames ? &raw_frames : nullptr, &error);
      }
      if (!received) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "USBCAN-FD receive failed: " << error);
      } else {
        if (canfd_logger.isOpen()) {
          ars620_driver::ScopedProcessingTimer timer(&timing_stats, "raw_log", debug_timing);
          for (const auto& raw_frame : raw_frames) {
            if (!canfd_logger.write(raw_frame, &error)) {
              ROS_ERROR_STREAM("CAN-FD full-frame logging disabled: " << error);
              break;
            }
          }
        }
        if (debug_raw_frames && !frames.empty()) {
          ars620_driver::ScopedProcessingTimer timer(&timing_stats, "raw_summary", debug_timing);
          std::map<uint32_t, size_t> id_counts;
          for (const auto& frame : frames) {
            ++id_counts[frame.id];
          }
          std::ostringstream oss;
          oss << "received " << frames.size() << " raw CAN-FD frames:";
          for (const auto& entry : id_counts) {
            oss << " 0x" << std::hex << entry.first << std::dec << "=" << entry.second;
          }
          ROS_INFO_STREAM_THROTTLE(1.0, oss.str());
        }
        for (const auto& frame : frames) {
          std_msgs::Header header;
          header.stamp = ars620_driver::stampForFrame(frame, stamp_policy);
          header.frame_id = frame_id;

          ars620_driver::ConfigState config_state;
          ars620_driver::SystemStatus status;
          ars620_driver::RdiHeader rdi_header;
          ars620_driver::OdHeader od_header;
          ars620_driver::RdiTargetFrame rdi_target_frame;
          std::vector<ars620_driver::OdTarget> od_targets;
          ars620_driver::RdiCycle rdi_cycle;
          ars620_driver::OdCycle od_cycle;
          ars620_driver::DecodedFrameType decoded_type = ars620_driver::DecodedFrameType::kNone;

          {
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "decode", debug_timing);
            if (ars620_driver::decodeConfigState(frame, &config_state)) {
              decoded_type = ars620_driver::DecodedFrameType::kConfigState;
            } else if (ars620_driver::decodeSystemStatus(frame, &status)) {
              decoded_type = ars620_driver::DecodedFrameType::kSystemStatus;
            } else if (ars620_driver::decodeRdiHeader(frame, &rdi_header)) {
              decoded_type = ars620_driver::DecodedFrameType::kRdiHeader;
            } else if (ars620_driver::decodeRdiTargetFrame(frame, &rdi_target_frame)) {
              decoded_type = ars620_driver::DecodedFrameType::kRdiTargets;
            } else if (ars620_driver::decodeOdHeader(frame, &od_header)) {
              decoded_type = ars620_driver::DecodedFrameType::kOdHeader;
            } else if (ars620_driver::decodeOdTargets(frame, &od_targets)) {
              decoded_type = ars620_driver::DecodedFrameType::kOdTargets;
            }
          }

          bool publish_config = false;
          bool publish_status = false;
          bool publish_rdi = false;
          bool publish_od = false;
          {
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "assemble", debug_timing);
            if (decoded_type == ars620_driver::DecodedFrameType::kConfigState) {
              publish_config = true;
            } else if (decoded_type == ars620_driver::DecodedFrameType::kSystemStatus) {
              publish_status = true;
            } else if (decoded_type == ars620_driver::DecodedFrameType::kRdiHeader) {
              if (rdi_assembler.hasActiveCycle() &&
                  rdi_assembler.receivedCount() < rdi_assembler.expectedCount()) {
                ROS_WARN_STREAM("dropping incomplete RDI cycle " << rdi_assembler.cycleCounter()
                                << " before new header: expected " << rdi_assembler.expectedCount()
                                << " targets, received " << rdi_assembler.receivedCount()
                                << ", missing RDI frames: "
                                << ars620_driver::formatFrameIds(rdi_assembler.missingFrameIds()));
              }
              publish_rdi = rdi_assembler.processHeader(rdi_header, ros::Time::now(), &rdi_cycle);
            } else if (decoded_type == ars620_driver::DecodedFrameType::kRdiTargets) {
              publish_rdi = rdi_assembler.processTargets(rdi_target_frame, &rdi_cycle);
            } else if (decoded_type == ars620_driver::DecodedFrameType::kOdHeader) {
              publish_od = od_assembler.processHeader(od_header, header.stamp, &od_cycle);
            } else if (decoded_type == ars620_driver::DecodedFrameType::kOdTargets) {
              publish_od = od_assembler.processTargets(od_targets, &od_cycle);
            }
          }

          if (publish_config) {
            const auto msg = [&]() {
              ars620_driver::ScopedProcessingTimer timer(&timing_stats, "message", debug_timing);
              return ars620_driver::toMsg(config_state, header);
            }();
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "publish", debug_timing);
            config_pub.publish(msg);
          } else if (publish_status) {
            const auto msg = [&]() {
              ars620_driver::ScopedProcessingTimer timer(&timing_stats, "message", debug_timing);
              return ars620_driver::toMsg(status, header);
            }();
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "publish", debug_timing);
            status_pub.publish(msg);
          } else if (publish_rdi) {
            const auto target_msg = [&]() {
              ars620_driver::ScopedProcessingTimer timer(&timing_stats, "message", debug_timing);
              return ars620_driver::toMsg(rdi_cycle, header);
            }();
            const auto pointcloud = [&]() {
              ars620_driver::ScopedProcessingTimer timer(&timing_stats, "pointcloud", debug_timing);
              return ars620_driver::makeRdiPointCloud(header, rdi_cycle.targets);
            }();
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "publish", debug_timing);
            rdi_targets_pub.publish(target_msg);
            rdi_points_pub.publish(pointcloud);
          } else if (publish_od) {
            const auto target_msg = [&]() {
              ars620_driver::ScopedProcessingTimer timer(&timing_stats, "message", debug_timing);
              return ars620_driver::toMsg(od_cycle, header);
            }();
            const auto pointcloud = [&]() {
              ars620_driver::ScopedProcessingTimer timer(&timing_stats, "pointcloud", debug_timing);
              return ars620_driver::makeOdPointCloud(header, od_cycle.targets);
            }();
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "publish", debug_timing);
            od_targets_pub.publish(target_msg);
            od_points_pub.publish(pointcloud);
          }
        }

        ars620_driver::RdiCycle rdi_cycle;
        ars620_driver::OdCycle od_cycle;
        std_msgs::Header timeout_header;
        timeout_header.stamp = ros::Time::now();
        timeout_header.frame_id = frame_id;
        bool publish_rdi_timeout = false;
        bool publish_od_timeout = false;
        {
          ars620_driver::ScopedProcessingTimer timer(&timing_stats, "assemble", debug_timing);
          if (rdi_assembler.pollTimeout(timeout_header.stamp, &rdi_cycle)) {
            publish_rdi_timeout = true;
          } else if (rdi_assembler.hasTimedOut(timeout_header.stamp) &&
                     rdi_assembler.receivedCount() < rdi_assembler.expectedCount()) {
            ROS_WARN_STREAM_THROTTLE(1.0, "waiting for complete RDI cycle "
                                              << rdi_assembler.cycleCounter() << ": expected "
                                              << rdi_assembler.expectedCount()
                                              << " targets, received " << rdi_assembler.receivedCount()
                                              << ", missing RDI frames: "
                                              << ars620_driver::formatFrameIds(
                                                     rdi_assembler.missingFrameIds()));
          }
          publish_od_timeout = od_assembler.pollTimeout(timeout_header.stamp, &od_cycle);
        }
        if (publish_rdi_timeout) {
          const auto target_msg = [&]() {
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "message", debug_timing);
            return ars620_driver::toMsg(rdi_cycle, timeout_header);
          }();
          const auto pointcloud = [&]() {
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "pointcloud", debug_timing);
            return ars620_driver::makeRdiPointCloud(timeout_header, rdi_cycle.targets);
          }();
          ars620_driver::ScopedProcessingTimer timer(&timing_stats, "publish", debug_timing);
          rdi_targets_pub.publish(target_msg);
          rdi_points_pub.publish(pointcloud);
        }
        if (publish_od_timeout) {
          const auto target_msg = [&]() {
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "message", debug_timing);
            return ars620_driver::toMsg(od_cycle, timeout_header);
          }();
          const auto pointcloud = [&]() {
            ars620_driver::ScopedProcessingTimer timer(&timing_stats, "pointcloud", debug_timing);
            return ars620_driver::makeOdPointCloud(timeout_header, od_cycle.targets);
          }();
          ars620_driver::ScopedProcessingTimer timer(&timing_stats, "publish", debug_timing);
          od_targets_pub.publish(target_msg);
          od_points_pub.publish(pointcloud);
        }
      }

      ros::spinOnce();
    }
    ars620_driver::maybeLogTimingStats(&timing_stats, debug_timing, timing_log_period,
                                       &next_timing_log);
    idle_rate.sleep();
  }
  canfd_logger.close(true);
  return 0;
}
