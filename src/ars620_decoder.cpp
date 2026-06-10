#include <ars620_driver/ars620_decoder.h>

#include <cmath>
#include <sstream>

namespace ars620_driver {
namespace {

double u(const CanFrame& f, uint16_t start, uint8_t len, double scale = 1.0, double offset = 0.0) {
  return decodeSignal(f.data.data(), f.len, {start, len, scale, offset, false});
}

uint64_t uraw(const CanFrame& f, uint16_t start, uint8_t len) {
  return extractMotorola(f.data.data(), f.len, start, len);
}

bool bit(const CanFrame& f, uint16_t start) {
  return uraw(f, start, 1) != 0;
}

bool validFrame(const CanFrame& frame, uint32_t id, uint8_t len) {
  return frame.id == id && frame.len == len;
}

void fillRdiCartesian(RdiTarget* target) {
  const double ce = std::cos(target->elevation);
  target->x = target->range * ce * std::cos(target->azimuth);
  target->y = target->range * ce * std::sin(target->azimuth);
  target->z = target->range * std::sin(target->elevation);
}

RdiTarget decodeRdiTargetAt(const CanFrame& f, int local_index, uint16_t cluster_index) {
  const int pair = local_index / 2;
  const int offset = pair * 120;
  const bool even = (local_index % 2) == 0;
  RdiTarget out;
  out.cluster_index = cluster_index;
  if (even) {
    out.range = u(f, 31 + offset, 12, 0.07, 0.0);
    out.vrad_rel = u(f, 35 + offset, 11, 0.1, -120.0);
    out.rcs = u(f, 40 + offset, 9, 0.2, -51.2);
    out.azimuth = u(f, 63 + offset, 10, 0.0025, -1.2775);
    out.elevation = u(f, 69 + offset, 7, 0.005, -0.315);
    out.snr = u(f, 78 + offset, 7, 0.5, 0.0);
    out.dyn_prop = static_cast<uint8_t>(uraw(f, 87 + offset, 2));
    out.quality = static_cast<uint8_t>(uraw(f, 85 + offset, 2));
  } else {
    out.range = u(f, 95 + offset, 12, 0.07, 0.0);
    out.vrad_rel = u(f, 99 + offset, 11, 0.1, -120.0);
    out.rcs = u(f, 104 + offset, 9, 0.2, -51.2);
    out.azimuth = u(f, 127 + offset, 10, 0.0025, -1.2775);
    out.elevation = u(f, 133 + offset, 7, 0.005, -0.315);
    out.snr = u(f, 142 + offset, 7, 0.5, 0.0);
    out.dyn_prop = static_cast<uint8_t>(uraw(f, 83 + offset, 2));
    out.quality = static_cast<uint8_t>(uraw(f, 81 + offset, 2));
  }
  fillRdiCartesian(&out);
  return out;
}

OdTarget decodeOdTargetAt(const CanFrame& f, int local_index, uint8_t object_index) {
  OdTarget out;
  const bool first = local_index == 0;
  out.object_id = static_cast<uint8_t>(uraw(f, first ? 487 : 495, 8));
  out.x = u(f, first ? 31 : 247, 12, 0.07, 0.0);
  out.y = u(f, first ? 63 : 279, 12, 0.08, -163.76);
  out.z = 0.0;
  out.vx = u(f, first ? 35 : 251, 11, 0.1, -120.0);
  out.vy = u(f, first ? 67 : 283, 11, 0.1, -120.0);
  out.ax = u(f, first ? 40 : 256, 9, 0.125, -31.875);
  out.ay = u(f, first ? 72 : 288, 9, 0.125, -31.875);
  out.x_std = u(f, first ? 95 : 311, 8, 0.05, 0.0);
  out.y_std = u(f, first ? 103 : 319, 8, 0.1, 0.0);
  out.vx_std = u(f, first ? 111 : 327, 7, 0.1, 0.0);
  out.vy_std = u(f, first ? 104 : 320, 7, 0.2, 0.0);
  out.ax_std = u(f, first ? 113 : 329, 7, 0.25, 0.0);
  out.ay_std = u(f, first ? 135 : 351, 8, 0.25, 0.0);
  out.classification = static_cast<uint8_t>(uraw(f, first ? 122 : 338, 3));
  out.class_confidence = static_cast<uint8_t>(uraw(f, first ? 143 : 359, 7));
  out.prob_of_exist = static_cast<uint8_t>(uraw(f, first ? 136 : 352, 7));
  out.dyn_prop = static_cast<uint8_t>(uraw(f, first ? 154 : 370, 3));
  out.mirror_probability = static_cast<uint8_t>(uraw(f, first ? 167 : 383, 7));
  out.length = u(f, first ? 160 : 376, 7, 0.2, 0.0);
  out.maintenance_state = static_cast<uint8_t>(uraw(f, first ? 169 : 385, 2));
  out.width = u(f, first ? 183 : 399, 7, 0.2, 0.0);
  out.rcs = u(f, first ? 176 : 392, 9, 0.2, -51.2);
  out.orientation = u(f, first ? 199 : 415, 10, 0.01, -5.11);
  out.yaw_rate = u(f, first ? 205 : 421, 10, 0.001, -0.511);
  out.yaw_rate_std = u(f, first ? 211 : 427, 10, 0.0001, 0.0);
  out.ref_point = static_cast<uint8_t>(uraw(f, first ? 217 : 433, 2));
  out.life_cycle = static_cast<uint16_t>(uraw(f, first ? 231 : 447, 16));
  out.orientation_std = u(f, first ? 463 : 459, 4, 0.002, 0.0);
  out.obstacle_probability = static_cast<uint8_t>(uraw(f, first ? 471 : 479, 7));
  out.obstruction_type = static_cast<uint8_t>(uraw(f, first ? 497 : 499, 2));
  (void)object_index;
  return out;
}

}  // namespace

uint64_t extractMotorola(const uint8_t* data, size_t len, uint16_t start_bit, uint8_t bit_length) {
  if (data == nullptr || bit_length == 0 || bit_length > 64) {
    return 0;
  }
  uint64_t value = 0;
  int dbc_bit = static_cast<int>(start_bit);
  for (uint8_t i = 0; i < bit_length; ++i) {
    if (dbc_bit < 0) {
      return 0;
    }
    const size_t byte = static_cast<size_t>(dbc_bit / 8);
    const uint8_t bit_in_byte = static_cast<uint8_t>(dbc_bit % 8);
    if (byte >= len) {
      return 0;
    }
    value <<= 1U;
    value |= (data[byte] >> bit_in_byte) & 0x1U;
    dbc_bit = (dbc_bit % 8 == 0) ? dbc_bit + 15 : dbc_bit - 1;
  }
  return value;
}

int64_t signExtend(uint64_t raw, uint8_t bit_length) {
  if (bit_length == 0 || bit_length >= 64) {
    return static_cast<int64_t>(raw);
  }
  const uint64_t sign_bit = 1ULL << (bit_length - 1U);
  if ((raw & sign_bit) == 0) {
    return static_cast<int64_t>(raw);
  }
  const uint64_t mask = ~((1ULL << bit_length) - 1ULL);
  return static_cast<int64_t>(raw | mask);
}

double decodeSignal(const uint8_t* data, size_t len, const SignalSpec& spec) {
  const uint64_t raw = extractMotorola(data, len, spec.start_bit, spec.length);
  const double numeric = spec.is_signed ? static_cast<double>(signExtend(raw, spec.length))
                                        : static_cast<double>(raw);
  return numeric * spec.scale + spec.offset;
}

bool decodeConfigState(const CanFrame& f, ConfigState* out) {
  if (out == nullptr || !validFrame(f, kArsConfigStateId, 32)) {
    return false;
  }
  *out = ConfigState{};
  out->crc16_checksum = static_cast<uint16_t>(uraw(f, 7, 16));
  out->msg_counter = static_cast<uint8_t>(uraw(f, 23, 8));
  out->mode = static_cast<uint8_t>(uraw(f, 31, 8));
  out->time_sync_enable = bit(f, 32);
  out->str_ratio_configured = bit(f, 33);
  out->wheel_base_configured = bit(f, 34);
  out->track_width_front_configured = bit(f, 35);
  out->track_width_rear_configured = bit(f, 36);
  out->vehicle_weight_configured = bit(f, 37);
  out->center_of_gravity_height_configured = bit(f, 38);
  out->axis_load_distribution_configured = bit(f, 39);
  out->steering_ratio = u(f, 47, 12, 0.01, 0.0);
  out->wheel_base_m = u(f, 49, 10, 0.01, 0.0);
  out->track_width_front_m = u(f, 71, 12, 0.001, 0.0);
  out->track_width_rear_m = u(f, 87, 12, 0.001, 0.0);
  out->vehicle_weight_kg = static_cast<uint32_t>(uraw(f, 103, 16));
  out->center_of_gravity_height_m = u(f, 119, 12, 0.001, 0.0);
  out->axis_load_distribution_pct = u(f, 135, 10, 0.1, 0.0);
  out->lateral_position_m = u(f, 151, 11, 0.001, -1.023);
  out->longitudinal_position_m = u(f, 167, 13, 0.001, -4.095);
  out->vertical_position_m = u(f, 183, 10, 0.001, 0.0);
  out->lateral_position_configured = bit(f, 189);
  out->longitudinal_position_to_cog_m = u(f, 199, 14, 0.001, -4.095);
  out->yaw_angle_rad = u(f, 215, 13, 0.001, -4.095);
  out->longitudinal_position_configured = bit(f, 218);
  out->vertical_position_configured = bit(f, 217);
  out->longitudinal_position_to_cog_configured = bit(f, 216);
  out->sensor_orientation = bit(f, 231);
  out->sensor_orientation_configured = bit(f, 230);
  out->yaw_angle_configured = bit(f, 229);
  out->cover_damping_configured = bit(f, 228);
  out->cover_damping_db = u(f, 227, 12, 0.001, 0.0);
  out->msg_group_counter = static_cast<uint8_t>(uraw(f, 255, 8));
  return true;
}

bool decodeSystemStatus(const CanFrame& f, SystemStatus* out) {
  if (out == nullptr || !validFrame(f, kArsSystemStatusId, 20)) {
    return false;
  }
  *out = SystemStatus{};
  out->crc16_checksum = static_cast<uint16_t>(uraw(f, 7, 16));
  out->msg_counter = static_cast<uint8_t>(uraw(f, 23, 8));
  out->operation_mode = static_cast<uint8_t>(uraw(f, 31, 4));
  out->calibration_state = static_cast<uint8_t>(uraw(f, 27, 3));
  out->tunnel = bit(f, 24);
  out->azimuth_correction_rad = u(f, 39, 8, 0.0015, -0.1905);
  out->elevation_correction_rad = u(f, 47, 8, 0.0015, -0.1905);
  out->visibility_range_m = static_cast<uint16_t>(uraw(f, 55, 9));
  out->visibility_state = static_cast<uint8_t>(uraw(f, 62, 2));
  out->software_version_major = static_cast<uint8_t>(uraw(f, 71, 8));
  out->software_version_minor = static_cast<uint8_t>(uraw(f, 79, 8));
  out->software_version_patch = static_cast<uint8_t>(uraw(f, 87, 8));
  out->dtc_sensor_full_blockage = bit(f, 88);
  out->dtc_sensor_partial_blockage = bit(f, 89);
  out->dtc_online_calib_failed = bit(f, 90);
  out->dtc_alignment_fail = bit(f, 91);
  out->dtc_alignment_never_done = bit(f, 92);
  out->dtc_vbat_high = bit(f, 93);
  out->dtc_vbat_low = bit(f, 94);
  out->dtc_private_can_busoff = bit(f, 95);
  out->dtc_vehicle_config_invalid = bit(f, 96);
  out->dtc_vdy_parameter_exception = bit(f, 97);
  out->dtc_temperature_too_high = bit(f, 98);
  out->dtc_temperature_too_low = bit(f, 99);
  out->dtc_nvm_access_error = bit(f, 100);
  out->dtc_sw_failure = bit(f, 101);
  out->dtc_hw_plausibility_failure = bit(f, 102);
  out->dtc_hw_failure = bit(f, 103);
  out->dtc_idcm_timeout = bit(f, 104);
  out->dtc_tbox_timeout = bit(f, 105);
  out->dtc_tm_timeout = bit(f, 106);
  out->dtc_sas_timeout = bit(f, 107);
  out->dtc_esc_timeout = bit(f, 108);
  out->dtc_acu_timeout = bit(f, 109);
  out->dtc_time_sync_fail = bit(f, 110);
  out->dtc_vdy_signal_exception = bit(f, 111);
  out->dtc_tm_rollingcounter_error = bit(f, 112);
  out->dtc_sas_rollingcounter_error = bit(f, 113);
  out->dtc_esc_rollingcounter_error = bit(f, 114);
  out->dtc_acu_rollingcounter_error = bit(f, 115);
  out->dtc_tm_checksum_error = bit(f, 116);
  out->dtc_sas_checksum_error = bit(f, 117);
  out->dtc_esc_checksum_error = bit(f, 118);
  out->dtc_acu_checksum_error = bit(f, 119);
  out->dtc_whldir_fl_invalid = bit(f, 120);
  out->dtc_whlspd_rr_invalid = bit(f, 121);
  out->dtc_whlspd_rl_invalid = bit(f, 122);
  out->dtc_whlspd_fr_invalid = bit(f, 123);
  out->dtc_whlspd_fl_invalid = bit(f, 124);
  out->dtc_yawrate_invalid = bit(f, 125);
  out->dtc_lateralaccel_invalid = bit(f, 126);
  out->dtc_long_accel_invalid = bit(f, 127);
  out->dtc_strwhl_ang_invalid = bit(f, 128);
  out->dtc_epb_invalid = bit(f, 129);
  out->dtc_tcs_invalid = bit(f, 130);
  out->dtc_abs_invalid = bit(f, 131);
  out->dtc_vehspeed_invalid = bit(f, 132);
  out->dtc_whldir_rr_invalid = bit(f, 133);
  out->dtc_whldir_rl_invalid = bit(f, 134);
  out->dtc_whldir_fr_invalid = bit(f, 135);
  out->dtc_gearpos_invalid = bit(f, 143);
  out->msg_group_counter = static_cast<uint8_t>(uraw(f, 151, 8));
  return true;
}

bool decodeRdiHeader(const CanFrame& f, RdiHeader* out) {
  if (out == nullptr || !validFrame(f, kRdiHeaderId, 32)) {
    return false;
  }
  *out = RdiHeader{};
  out->global_timestamp_sec = static_cast<uint32_t>(uraw(f, 31, 32));
  out->global_timestamp_nsec = static_cast<uint32_t>(uraw(f, 63, 32));
  out->local_timestamp_us = static_cast<uint32_t>(uraw(f, 95, 32));
  out->meas_counter = static_cast<uint16_t>(uraw(f, 127, 16));
  out->cycle_counter = static_cast<uint16_t>(uraw(f, 143, 16));
  out->num_clusters = static_cast<uint16_t>(uraw(f, 159, 10));
  out->max_detection_range_m = static_cast<uint8_t>(uraw(f, 175, 8));
  out->ego_velocity_mps = u(f, 183, 11, 0.125, -128.0);
  out->ego_yaw_rate_radps = u(f, 188, 10, 0.001, -0.511);
  out->latency_ms = u(f, 194, 11, 0.1, 30.0);
  out->ego_velocity_std_mps = u(f, 215, 7, 0.1, 0.0);
  out->ego_acceleration_mps2 = u(f, 208, 9, 0.125, -31.875);
  out->ego_curvature_inv_m = u(f, 231, 10, 0.001, -0.511);
  out->ambig_free_doppler_range_mps = u(f, 237, 12, 0.01, 0.0);
  out->task_valid = bit(f, 241);
  out->extended_cycle = bit(f, 240);
  out->msg_group_counter = static_cast<uint8_t>(uraw(f, 255, 8));
  return true;
}

bool decodeRdiTargetFrame(const CanFrame& f, RdiTargetFrame* out) {
  if (out == nullptr || f.id < kRdiFirstDataId || f.id > kRdiLastDataId || f.len != 64) {
    return false;
  }
  *out = RdiTargetFrame{};
  out->frame_id = f.id;
  out->targets.reserve(8);
  const uint16_t base_index = static_cast<uint16_t>((f.id - kRdiFirstDataId) * 8U);
  for (int i = 0; i < 8; ++i) {
    out->targets.push_back(decodeRdiTargetAt(f, i, static_cast<uint16_t>(base_index + i)));
  }
  return true;
}

bool decodeRdiTargets(const CanFrame& f, std::vector<RdiTarget>* out) {
  if (out == nullptr) {
    return false;
  }
  RdiTargetFrame target_frame;
  if (!decodeRdiTargetFrame(f, &target_frame)) {
    return false;
  }
  *out = target_frame.targets;
  return true;
}

bool decodeOdHeader(const CanFrame& f, OdHeader* out) {
  if (out == nullptr || !validFrame(f, kOdHeaderId, 32)) {
    return false;
  }
  *out = OdHeader{};
  out->global_timestamp_sec = static_cast<uint32_t>(uraw(f, 31, 32));
  out->global_timestamp_nsec = static_cast<uint32_t>(uraw(f, 63, 32));
  out->local_timestamp_us = static_cast<uint32_t>(uraw(f, 95, 32));
  out->meas_counter = static_cast<uint16_t>(uraw(f, 127, 16));
  out->cycle_counter = static_cast<uint16_t>(uraw(f, 143, 16));
  out->num_objects = static_cast<uint8_t>(uraw(f, 159, 6));
  out->task_valid = bit(f, 153);
  out->extended_cycle = bit(f, 152);
  out->latency_ms = u(f, 167, 11, 0.1, 30.0);
  out->ego_velocity_mps = u(f, 172, 11, 0.125, -128.0);
  out->ego_yaw_rate_radps = u(f, 177, 10, 0.001, -0.511);
  out->ego_velocity_std_mps = u(f, 199, 7, 0.1, 0.0);
  out->ego_acceleration_mps2 = u(f, 192, 9, 0.125, -31.875);
  out->ego_curvature_inv_m = u(f, 215, 10, 0.001, -0.511);
  out->msg_group_counter = static_cast<uint8_t>(uraw(f, 231, 8));
  return true;
}

bool decodeOdTargets(const CanFrame& f, std::vector<OdTarget>* out) {
  if (out == nullptr || f.id < kOdFirstDataId || f.id > kOdLastDataId || f.len != 64) {
    return false;
  }
  out->clear();
  out->reserve(2);
  const uint8_t base_index = static_cast<uint8_t>((f.id - kOdFirstDataId) * 2U);
  out->push_back(decodeOdTargetAt(f, 0, base_index));
  out->push_back(decodeOdTargetAt(f, 1, static_cast<uint8_t>(base_index + 1U)));
  return true;
}

std::string frameRejectReason(const CanFrame& frame) {
  std::ostringstream oss;
  oss << "unsupported ARS620 frame id=0x" << std::hex << frame.id << std::dec
      << " len=" << static_cast<unsigned>(frame.len);
  return oss.str();
}

}  // namespace ars620_driver
