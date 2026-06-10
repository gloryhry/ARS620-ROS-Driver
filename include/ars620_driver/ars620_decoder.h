#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace ars620_driver {

constexpr uint32_t kArsConfigStateId = 0x50;
constexpr uint32_t kArsSystemStatusId = 0x52;
constexpr uint32_t kRdiHeaderId = 0x100;
constexpr uint32_t kRdiFirstDataId = 0x101;
constexpr uint32_t kRdiLastDataId = 0x140;
constexpr uint32_t kOdHeaderId = 0x200;
constexpr uint32_t kOdFirstDataId = 0x201;
constexpr uint32_t kOdLastDataId = 0x219;

struct SignalSpec {
  uint16_t start_bit;
  uint8_t length;
  double scale;
  double offset;
  bool is_signed;
};

uint64_t extractMotorola(const uint8_t* data, size_t len, uint16_t start_bit, uint8_t bit_length);
int64_t signExtend(uint64_t raw, uint8_t bit_length);
double decodeSignal(const uint8_t* data, size_t len, const SignalSpec& spec);

struct CanFrame {
  uint32_t id = 0;
  std::array<uint8_t, 64> data{};
  uint8_t len = 0;
  uint64_t timestamp_us = 0;
};

struct ConfigState {
  uint16_t crc16_checksum = 0;
  uint8_t msg_counter = 0;
  uint8_t mode = 0;
  bool time_sync_enable = false;
  bool str_ratio_configured = false;
  bool wheel_base_configured = false;
  bool track_width_front_configured = false;
  bool track_width_rear_configured = false;
  bool vehicle_weight_configured = false;
  bool center_of_gravity_height_configured = false;
  bool axis_load_distribution_configured = false;
  double steering_ratio = 0.0;
  double wheel_base_m = 0.0;
  double track_width_front_m = 0.0;
  double track_width_rear_m = 0.0;
  uint32_t vehicle_weight_kg = 0;
  double center_of_gravity_height_m = 0.0;
  double axis_load_distribution_pct = 0.0;
  double lateral_position_m = 0.0;
  double longitudinal_position_m = 0.0;
  double vertical_position_m = 0.0;
  double longitudinal_position_to_cog_m = 0.0;
  double yaw_angle_rad = 0.0;
  double cover_damping_db = 0.0;
  bool lateral_position_configured = false;
  bool longitudinal_position_configured = false;
  bool vertical_position_configured = false;
  bool longitudinal_position_to_cog_configured = false;
  bool yaw_angle_configured = false;
  bool cover_damping_configured = false;
  bool sensor_orientation_configured = false;
  bool sensor_orientation = false;
  uint8_t msg_group_counter = 0;
};

struct SystemStatus {
  uint16_t crc16_checksum = 0;
  uint8_t msg_counter = 0;
  uint8_t operation_mode = 0;
  uint8_t calibration_state = 0;
  bool tunnel = false;
  double azimuth_correction_rad = 0.0;
  double elevation_correction_rad = 0.0;
  uint16_t visibility_range_m = 0;
  uint8_t visibility_state = 0;
  uint8_t software_version_major = 0;
  uint8_t software_version_minor = 0;
  uint8_t software_version_patch = 0;
  bool dtc_sensor_full_blockage = false;
  bool dtc_sensor_partial_blockage = false;
  bool dtc_online_calib_failed = false;
  bool dtc_alignment_fail = false;
  bool dtc_alignment_never_done = false;
  bool dtc_vbat_high = false;
  bool dtc_vbat_low = false;
  bool dtc_private_can_busoff = false;
  bool dtc_vehicle_config_invalid = false;
  bool dtc_vdy_parameter_exception = false;
  bool dtc_temperature_too_high = false;
  bool dtc_temperature_too_low = false;
  bool dtc_nvm_access_error = false;
  bool dtc_sw_failure = false;
  bool dtc_hw_plausibility_failure = false;
  bool dtc_hw_failure = false;
  bool dtc_idcm_timeout = false;
  bool dtc_tbox_timeout = false;
  bool dtc_tm_timeout = false;
  bool dtc_sas_timeout = false;
  bool dtc_esc_timeout = false;
  bool dtc_acu_timeout = false;
  bool dtc_time_sync_fail = false;
  bool dtc_vdy_signal_exception = false;
  bool dtc_tm_rollingcounter_error = false;
  bool dtc_sas_rollingcounter_error = false;
  bool dtc_esc_rollingcounter_error = false;
  bool dtc_acu_rollingcounter_error = false;
  bool dtc_tm_checksum_error = false;
  bool dtc_sas_checksum_error = false;
  bool dtc_esc_checksum_error = false;
  bool dtc_acu_checksum_error = false;
  bool dtc_whldir_fl_invalid = false;
  bool dtc_whlspd_rr_invalid = false;
  bool dtc_whlspd_rl_invalid = false;
  bool dtc_whlspd_fr_invalid = false;
  bool dtc_whlspd_fl_invalid = false;
  bool dtc_yawrate_invalid = false;
  bool dtc_lateralaccel_invalid = false;
  bool dtc_long_accel_invalid = false;
  bool dtc_strwhl_ang_invalid = false;
  bool dtc_epb_invalid = false;
  bool dtc_tcs_invalid = false;
  bool dtc_abs_invalid = false;
  bool dtc_vehspeed_invalid = false;
  bool dtc_whldir_rr_invalid = false;
  bool dtc_whldir_rl_invalid = false;
  bool dtc_whldir_fr_invalid = false;
  bool dtc_gearpos_invalid = false;
  uint8_t msg_group_counter = 0;
};

struct RdiHeader {
  uint32_t global_timestamp_sec = 0;
  uint32_t global_timestamp_nsec = 0;
  uint32_t local_timestamp_us = 0;
  uint16_t meas_counter = 0;
  uint16_t cycle_counter = 0;
  uint16_t num_clusters = 0;
  uint8_t max_detection_range_m = 0;
  double ego_velocity_mps = 0.0;
  double ego_yaw_rate_radps = 0.0;
  double latency_ms = 0.0;
  double ego_velocity_std_mps = 0.0;
  double ego_acceleration_mps2 = 0.0;
  double ego_curvature_inv_m = 0.0;
  double ambig_free_doppler_range_mps = 0.0;
  bool task_valid = false;
  bool extended_cycle = false;
  uint8_t msg_group_counter = 0;
};

struct RdiTarget {
  uint16_t cluster_index = 0;
  double range = 0.0;
  double azimuth = 0.0;
  double elevation = 0.0;
  double vrad_rel = 0.0;
  double rcs = 0.0;
  double snr = 0.0;
  uint8_t dyn_prop = 0;
  uint8_t quality = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct RdiTargetFrame {
  uint32_t frame_id = 0;
  std::vector<RdiTarget> targets;
};

struct RdiCycle {
  RdiHeader header;
  std::vector<RdiTarget> targets;
};

struct OdHeader {
  uint32_t global_timestamp_sec = 0;
  uint32_t global_timestamp_nsec = 0;
  uint32_t local_timestamp_us = 0;
  uint16_t meas_counter = 0;
  uint16_t cycle_counter = 0;
  uint8_t num_objects = 0;
  double latency_ms = 0.0;
  double ego_velocity_mps = 0.0;
  double ego_yaw_rate_radps = 0.0;
  double ego_velocity_std_mps = 0.0;
  double ego_acceleration_mps2 = 0.0;
  double ego_curvature_inv_m = 0.0;
  bool task_valid = false;
  bool extended_cycle = false;
  uint8_t msg_group_counter = 0;
};

struct OdTarget {
  uint8_t object_id = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double rcs = 0.0;
  double length = 0.0;
  double width = 0.0;
  double orientation = 0.0;
  double yaw_rate = 0.0;
  uint8_t classification = 0;
  uint8_t dyn_prop = 0;
  uint8_t prob_of_exist = 0;
  uint8_t maintenance_state = 0;
  uint8_t class_confidence = 0;
  uint8_t mirror_probability = 0;
  uint8_t obstruction_type = 0;
  uint8_t ref_point = 0;
  uint16_t life_cycle = 0;
  double x_std = 0.0;
  double y_std = 0.0;
  double vx_std = 0.0;
  double vy_std = 0.0;
  double ax_std = 0.0;
  double ay_std = 0.0;
  double orientation_std = 0.0;
  double yaw_rate_std = 0.0;
  uint8_t obstacle_probability = 0;
};

struct OdCycle {
  OdHeader header;
  std::vector<OdTarget> targets;
};

bool decodeConfigState(const CanFrame& frame, ConfigState* out);
bool decodeSystemStatus(const CanFrame& frame, SystemStatus* out);
bool decodeRdiHeader(const CanFrame& frame, RdiHeader* out);
bool decodeRdiTargetFrame(const CanFrame& frame, RdiTargetFrame* out);
bool decodeRdiTargets(const CanFrame& frame, std::vector<RdiTarget>* out);
bool decodeOdHeader(const CanFrame& frame, OdHeader* out);
bool decodeOdTargets(const CanFrame& frame, std::vector<OdTarget>* out);
std::string frameRejectReason(const CanFrame& frame);

}  // namespace ars620_driver
