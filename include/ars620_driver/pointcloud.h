#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <ars620_driver/ars620_decoder.h>

namespace ars620_driver {

sensor_msgs::PointCloud2 makeRdiPointCloud(const std_msgs::Header& header,
                                           const std::vector<RdiTarget>& targets);
sensor_msgs::PointCloud2 makeOdPointCloud(const std_msgs::Header& header,
                                          const std::vector<OdTarget>& targets);

}  // namespace ars620_driver
