#include <ars620_driver/pointcloud.h>

#include <sensor_msgs/point_cloud2_iterator.h>

namespace ars620_driver {
namespace {

void initFields(sensor_msgs::PointCloud2* cloud,
                const std::vector<std::tuple<std::string, uint8_t, uint8_t>>& fields) {
  cloud->fields.clear();
  uint32_t offset = 0;
  for (const auto& field_spec : fields) {
    sensor_msgs::PointField field;
    field.name = std::get<0>(field_spec);
    field.offset = offset;
    field.datatype = std::get<1>(field_spec);
    field.count = 1;
    offset += std::get<2>(field_spec);
    cloud->fields.push_back(field);
  }
  cloud->point_step = offset;
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->data.assign(cloud->row_step * cloud->height, 0);
  cloud->is_bigendian = false;
  cloud->is_dense = true;
}

void writeFloat(sensor_msgs::PointCloud2* cloud, size_t point, const std::string& field, float value) {
  sensor_msgs::PointCloud2Iterator<float> it(*cloud, field);
  it += point;
  *it = value;
}

void writeUint32(sensor_msgs::PointCloud2* cloud, size_t point, const std::string& field, uint32_t value) {
  sensor_msgs::PointCloud2Iterator<uint32_t> it(*cloud, field);
  it += point;
  *it = value;
}

}  // namespace

sensor_msgs::PointCloud2 makeRdiPointCloud(const std_msgs::Header& header,
                                           const std::vector<RdiTarget>& targets) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(targets.size());
  initFields(&cloud, {
                         {"x", sensor_msgs::PointField::FLOAT32, 4},
                         {"y", sensor_msgs::PointField::FLOAT32, 4},
                         {"z", sensor_msgs::PointField::FLOAT32, 4},
                         {"range", sensor_msgs::PointField::FLOAT32, 4},
                         {"azimuth", sensor_msgs::PointField::FLOAT32, 4},
                         {"elevation", sensor_msgs::PointField::FLOAT32, 4},
                         {"vrad_rel", sensor_msgs::PointField::FLOAT32, 4},
                         {"rcs", sensor_msgs::PointField::FLOAT32, 4},
                         {"snr", sensor_msgs::PointField::FLOAT32, 4},
                         {"dyn_prop", sensor_msgs::PointField::UINT32, 4},
                         {"quality", sensor_msgs::PointField::UINT32, 4},
                         {"cluster_index", sensor_msgs::PointField::UINT32, 4},
                     });
  for (size_t i = 0; i < targets.size(); ++i) {
    const auto& t = targets[i];
    writeFloat(&cloud, i, "x", static_cast<float>(t.x));
    writeFloat(&cloud, i, "y", static_cast<float>(t.y));
    writeFloat(&cloud, i, "z", static_cast<float>(t.z));
    writeFloat(&cloud, i, "range", static_cast<float>(t.range));
    writeFloat(&cloud, i, "azimuth", static_cast<float>(t.azimuth));
    writeFloat(&cloud, i, "elevation", static_cast<float>(t.elevation));
    writeFloat(&cloud, i, "vrad_rel", static_cast<float>(t.vrad_rel));
    writeFloat(&cloud, i, "rcs", static_cast<float>(t.rcs));
    writeFloat(&cloud, i, "snr", static_cast<float>(t.snr));
    writeUint32(&cloud, i, "dyn_prop", t.dyn_prop);
    writeUint32(&cloud, i, "quality", t.quality);
    writeUint32(&cloud, i, "cluster_index", t.cluster_index);
  }
  return cloud;
}

sensor_msgs::PointCloud2 makeOdPointCloud(const std_msgs::Header& header,
                                          const std::vector<OdTarget>& targets) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(targets.size());
  initFields(&cloud, {
                         {"x", sensor_msgs::PointField::FLOAT32, 4},
                         {"y", sensor_msgs::PointField::FLOAT32, 4},
                         {"z", sensor_msgs::PointField::FLOAT32, 4},
                         {"vx", sensor_msgs::PointField::FLOAT32, 4},
                         {"vy", sensor_msgs::PointField::FLOAT32, 4},
                         {"ax", sensor_msgs::PointField::FLOAT32, 4},
                         {"ay", sensor_msgs::PointField::FLOAT32, 4},
                         {"rcs", sensor_msgs::PointField::FLOAT32, 4},
                         {"length", sensor_msgs::PointField::FLOAT32, 4},
                         {"width", sensor_msgs::PointField::FLOAT32, 4},
                         {"orientation", sensor_msgs::PointField::FLOAT32, 4},
                         {"yaw_rate", sensor_msgs::PointField::FLOAT32, 4},
                         {"object_id", sensor_msgs::PointField::UINT32, 4},
                         {"classification", sensor_msgs::PointField::UINT32, 4},
                         {"dyn_prop", sensor_msgs::PointField::UINT32, 4},
                         {"prob_of_exist", sensor_msgs::PointField::UINT32, 4},
                         {"maintenance_state", sensor_msgs::PointField::UINT32, 4},
                     });
  for (size_t i = 0; i < targets.size(); ++i) {
    const auto& t = targets[i];
    writeFloat(&cloud, i, "x", static_cast<float>(t.x));
    writeFloat(&cloud, i, "y", static_cast<float>(t.y));
    writeFloat(&cloud, i, "z", static_cast<float>(t.z));
    writeFloat(&cloud, i, "vx", static_cast<float>(t.vx));
    writeFloat(&cloud, i, "vy", static_cast<float>(t.vy));
    writeFloat(&cloud, i, "ax", static_cast<float>(t.ax));
    writeFloat(&cloud, i, "ay", static_cast<float>(t.ay));
    writeFloat(&cloud, i, "rcs", static_cast<float>(t.rcs));
    writeFloat(&cloud, i, "length", static_cast<float>(t.length));
    writeFloat(&cloud, i, "width", static_cast<float>(t.width));
    writeFloat(&cloud, i, "orientation", static_cast<float>(t.orientation));
    writeFloat(&cloud, i, "yaw_rate", static_cast<float>(t.yaw_rate));
    writeUint32(&cloud, i, "object_id", t.object_id);
    writeUint32(&cloud, i, "classification", t.classification);
    writeUint32(&cloud, i, "dyn_prop", t.dyn_prop);
    writeUint32(&cloud, i, "prob_of_exist", t.prob_of_exist);
    writeUint32(&cloud, i, "maintenance_state", t.maintenance_state);
  }
  return cloud;
}

}  // namespace ars620_driver
